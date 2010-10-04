/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
      GDALOptions driverOpt;
      driverOpt.url() = "../data/world.tif";

      ImageLayerOptions layerOpt;
      layerOpt.driver() = driverOpt;
      layerOpt.name() = "test_simple";

      osg::ref_ptr<ImageLayer> layer = new ImageLayer( layerOpt );

      TileKey key(0, 0, 0, layer->getProfile());
	  GeoImage image = layer->createImage( key );
	  osgDB::writeImageFile(*image.getImage(), layer->getName()+key.str() + std::string(".png"));
  }

  //Mosaic test.  Request a tile in the global geodetic profile from a layer with a geographic SRS but a different tiling scheme.
  {
      ArcGISOptions driverOpt;
      driverOpt.url() = "http://server.arcgisonline.com/ArcGIS/rest/services/ESRI_Imagery_World_2D/MapServer";

      ImageLayerOptions layerOpt;
      layerOpt.driver() = driverOpt;
      layerOpt.name() = "test_mosaic";

      osg::ref_ptr<ImageLayer> layer = new ImageLayer( layerOpt );

      TileKey key(0, 0, 0, osgEarth::Registry::instance()->getGlobalGeodeticProfile());
	  GeoImage image = layer->createImage( key );
	  osgDB::writeImageFile(*image.getImage(), layer->getName()+key.str() + std::string(".png"));
  }

  //Reprojection.  Request a UTM image from a global geodetic profile
  {
      ArcGISOptions driverOpt;
      driverOpt.url() = "http://server.arcgisonline.com/ArcGIS/rest/services/ESRI_Imagery_World_2D/MapServer";

      ImageLayerOptions layerOpt;
      layerOpt.name() = "test_reprojected_utm";
      layerOpt.driver() = driverOpt;
      layerOpt.reprojectedTileSize() = 512;

      osg::ref_ptr<ImageLayer> layer = new ImageLayer( layerOpt );

      TileKey key(0, 0, 0, Profile::create("epsg:26917", 560725, 4385762, 573866, 4400705));
	  GeoImage image = layer->createImage( key );
	  osgDB::writeImageFile(*image.getImage(), layer->getName()+key.str() + std::string(".png"));
  }


  //Mercator.  Request a geodetic reprojected image from a mercator source
  {
      TMSOptions driverOpt;
      driverOpt.url() = "http://tile.openstreetmap.org";
      driverOpt.format() = "png";
      driverOpt.tileSize() = 256;
      driverOpt.tmsType() = "google";

      ImageLayerOptions layerOpt;
      layerOpt.driver() = driverOpt;
      layerOpt.name() = "test_mercator_reprojected";
      layerOpt.reprojectedTileSize() = 256;
      layerOpt.exactCropping() = true;
      layerOpt.profile() = ProfileOptions( "global-mercator" );

      osg::ref_ptr<ImageLayer> layer = new ImageLayer( layerOpt );

	  //Request an image from the mercator source.  Should be reprojected to geodetic
	  TileKey key(0, 0, 0, osgEarth::Registry::instance()->getGlobalGeodeticProfile());
	  GeoImage image = layer->createImage( key );
	  if (!image.getSRS()->isGeographic())
	  {
		  OE_NOTICE << "Error:  Should have reprojected image to geodetic but returned SRS is  " << image.getSRS()->getWKT() << std::endl;
	  }
	  osgDB::writeImageFile(*image.getImage(), layer->getName()+key.str() + std::string(".png"));
  }

  return 0;
}

