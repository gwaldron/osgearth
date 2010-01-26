/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgEarth/Map>
#include <osgEarth/MapLayer>
#include <osgEarth/Registry>


#include <iostream>

using namespace osg;
using namespace osgDB;
using namespace osgEarth;

int main(int argc, char** argv)
{
  osg::ArgumentParser arguments(&argc,argv);

  //One to one test.  Read a single 1 to 1 tile out of a MapLayer
  {
	  //NOTE:  You must run this from the osgearth/tests directory for world.tif to be found at this path.
      Config conf;
      conf.add( "url", "../data/world.tif" );
	  osg::ref_ptr<MapLayer> layer = new MapLayer("test_simple", MapLayer::TYPE_IMAGE, "gdal", conf );

	  osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, layer->getProfile());
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

  //Mosaic test.  Request a tile in the global geodetic profile from a layer with a geographic SRS but a different tiling scheme.
  {
      Config conf;
      conf.add( "url", "http://server.arcgisonline.com/ArcGIS/rest/services/ESRI_Imagery_World_2D/MapServer" );
	  osg::ref_ptr<MapLayer> layer = new MapLayer("test_mosaic", MapLayer::TYPE_IMAGE, "arcgis", conf );

	  osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, osgEarth::Registry::instance()->getGlobalGeodeticProfile());
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

  //Reprojection.  Request a UTM image from a global geodetic profile
  {
      Config conf;
      conf.add( "url", "http://server.arcgisonline.com/ArcGIS/rest/services/ESRI_Imagery_World_2D/MapServer" );
	  osg::ref_ptr<MapLayer> layer = new MapLayer("test_reprojected_utm", MapLayer::TYPE_IMAGE, "arcgis", conf );
	  //Tell the layer that if reprojection is necessary, the reprojected image should be the given tile size.
	  //Otherwise, the optimal tile size will be computed.
	  layer->setReprojectedTileSize( 512);

      osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, Profile::create("epsg:26917", 560725, 4385762, 573866, 4400705));
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

  //Mercator.  Test Mercator fast path.
  {
      Config conf;
      conf.add( "url", "http://tile.openstreetmap.org/" );
      conf.add( "format", "png" );
      conf.add( "tile_size", "256" );
      conf.add( "tms_type", "google" );

	  osg::ref_ptr<MapLayer> layer = new MapLayer("test_mercator", MapLayer::TYPE_IMAGE, "tms", conf );
	  layer->profileConfig() = ProfileConfig( "global-mercator" );
	  layer->setUseMercatorFastPath( true );

	  //Request a mercator image using the mercator fast path, the default
	  osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, osgEarth::Registry::instance()->getGlobalGeodeticProfile());
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  if (!image->getSRS()->isMercator())
	  {
		  osg::notify(osg::NOTICE) << "Error:  Should be using mercator fast path but returned SRS is " << image->getSRS()->getWKT() << std::endl;
	  }
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

    //Mercator.  Request a geodetic reprojected image from a mercator source
  {
      Config conf;
      conf.add( "url", "http://tile.openstreetmap.org/" );
      conf.add( "format", "png" );
      conf.add( "tile_size", "256" );
      conf.add( "tms_type", "google" );

	  osg::ref_ptr<MapLayer> layer = new MapLayer("test_mercator_reprojected", MapLayer::TYPE_IMAGE, "tms", conf );
	  layer->setUseMercatorFastPath( false );
	  layer->setReprojectedTileSize( 256 );
	  layer->setExactCropping( true );
	  layer->profileConfig() = ProfileConfig( "global-mercator" );

	  //Request an image from the mercator source.  Should be reprojected to geodetic
	  osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, osgEarth::Registry::instance()->getGlobalGeodeticProfile());
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  if (!image->getSRS()->isGeographic())
	  {
		  osg::notify(osg::NOTICE) << "Error:  Should have reprojected image to geodetic but returned SRS is  " << image->getSRS()->getWKT() << std::endl;
	  }
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

  return 0;
}
