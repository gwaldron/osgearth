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

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/arcgis/ArcGISOptions>
#include <osgEarthDrivers/tms/TMSOptions>

#include <iostream>

using namespace osg;
using namespace osgDB;
using namespace osgEarth;
using namespace osgEarth::Drivers;

int main(int argc, char** argv)
{
  osg::ArgumentParser arguments(&argc,argv);

  //One to one test.  Read a single 1 to 1 tile out of a MapLayer
  {
      GDALOptions* opt = new GDALOptions();
      opt->url() = "../data/world.tif";
      osg::ref_ptr<MapLayer> layer = new ImageMapLayer( "test_simple", opt );

	  osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, layer->getProfile());
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

  //Mosaic test.  Request a tile in the global geodetic profile from a layer with a geographic SRS but a different tiling scheme.
  {
      ArcGISOptions* opt = new ArcGISOptions();
      opt->url() = "http://server.arcgisonline.com/ArcGIS/rest/services/ESRI_Imagery_World_2D/MapServer";
      osg::ref_ptr<MapLayer> layer = new ImageMapLayer( "test_mosaic", opt );

	  osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, osgEarth::Registry::instance()->getGlobalGeodeticProfile());
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

  //Reprojection.  Request a UTM image from a global geodetic profile
  {
      ArcGISOptions* opt = new ArcGISOptions();
      opt->url() = "http://server.arcgisonline.com/ArcGIS/rest/services/ESRI_Imagery_World_2D/MapServer";
      osg::ref_ptr<MapLayer> layer = new ImageMapLayer( "test_reprojected_utm", opt );

	  //Tell the layer that if reprojection is necessary, the reprojected image should be the given tile size.
	  //Otherwise, the optimal tile size will be computed.
	  layer->setReprojectedTileSize( 512 );

      osg::ref_ptr<TileKey> key = new TileKey(0, 0, 0, 0, Profile::create("epsg:26917", 560725, 4385762, 573866, 4400705));
	  osg::ref_ptr<GeoImage> image = layer->createImage( key );
	  osgDB::writeImageFile(*image->getImage(), layer->getName()+key->str() + std::string(".png"));
  }

  //Mercator.  Test Mercator fast path.
  {
      TMSOptions* opt = new TMSOptions();
      opt->url() = "http://tile.openstreetmap.org";
      opt->format() = "png";
      opt->tileSize() = 256;
      opt->tmsType() = "google";
      osg::ref_ptr<MapLayer> layer = new ImageMapLayer( "test_mercator", opt );

	  layer->profileConfig() = ProfileConfig( "global-mercator" );
      layer->useMercatorFastPath() = true;

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
      TMSOptions* opt = new TMSOptions();
      opt->url() = "http://tile.openstreetmap.org";
      opt->format() = "png";
      opt->tileSize() = 256;
      opt->tmsType() = "google";
      osg::ref_ptr<MapLayer> layer = new ImageMapLayer( "test_mercator_reprojected", opt );

	  layer->useMercatorFastPath() = false;
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

