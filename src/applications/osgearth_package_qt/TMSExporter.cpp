/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

#include "TMSExporter.h"

#include <osg/io_utils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile>

#include <osgEarth/Common>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/HTTPClient>
#include <osgEarthUtil/TMSPackager>
#include <osgEarthDrivers/tms/TMSOptions>

#include <iostream>
#include <sstream>
#include <iterator>

using namespace PackageQt;
using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;

#define LC "[TMSExporter] "

TMSExporter::TMSExporter(const std::string& log)
: _dbOptions(""), _maxLevel(~0), _keepEmpties(false), _errorMessage("")
{
}

/** Packages an image layer as a TMS folder. */
bool TMSExporter::exportTMS(MapNode* mapNode, const std::string& path, std::vector< Bounds >& bounds, const std::string& outEarth, bool overwrite, const std::string& extension)
{
  if ( !mapNode )
  {
    _errorMessage = "Invalid MapNode";
    return false;
  }

  // folder to which to write the TMS archive.
  std::string rootFolder = path;

  osg::ref_ptr<osgDB::Options> options = new osgDB::Options(_dbOptions);

  // create a folder for the output
  osgDB::makeDirectory(rootFolder);
  if ( !osgDB::fileExists(rootFolder) )
  {
    _errorMessage = "Failed to create root output folder";
    return false;
  }

  Map* map = mapNode->getMap();

  // fire up a packager:
  TMSPackager packager( map->getProfile(), options);



  //TODO:
  bool verbose = true;

  packager.setVerbose( verbose );
  packager.setOverwrite( overwrite );
  packager.setKeepEmptyImageTiles( _keepEmpties );

  if ( _maxLevel != ~0 )
      packager.setMaxLevel( _maxLevel );

  if (bounds.size() > 0)
  {
      for (unsigned int i = 0; i < bounds.size(); ++i)
      {
          Bounds b = bounds[i];            
          if ( b.isValid() )
              packager.addExtent( GeoExtent(map->getProfile()->getSRS(), b) );
      }
  }    

  
  // new map for an output earth file if necessary.
  osg::ref_ptr<Map> outMap = 0L;
  if ( !outEarth.empty() )
  {
      // copy the options from the source map first
      outMap = new Map(map->getInitialMapOptions());
  }


  // establish the output path of the earth file, if applicable:
  std::string outEarthName = osgDB::getSimpleFileName(outEarth);
  if (outEarthName.length() > 0 && osgEarth::toLower(osgDB::getFileExtension(outEarthName)) != "earth")
    outEarthName += ".earth";

  std::string outEarthFile = osgDB::concatPaths(rootFolder, outEarthName);


  // package any image layers that are enabled:
  ImageLayerVector imageLayers;
  map->getImageLayers( imageLayers );

  unsigned counter = 0;
  
  for( ImageLayerVector::iterator i = imageLayers.begin(); i != imageLayers.end(); ++i, ++counter )
  {
      ImageLayer* layer = i->get();
      //if ( layer->getImageLayerOptions().enabled() == true )
      if ( layer->getEnabled() && layer->getVisible() )
      {
          std::string layerFolder = toLegalFileName( layer->getName() );
          if ( layerFolder.empty() )
              layerFolder = Stringify() << "image_layer_" << counter;

          if ( verbose )
          {
              OE_NOTICE << LC << "Packaging image layer \"" << layerFolder << "\"" << std::endl;
          }

          std::string layerRoot = osgDB::concatPaths( rootFolder, layerFolder );
          TMSPackager::Result r = packager.package( layer, layerRoot, extension );
          if ( r.ok )
          {
              // save to the output map if requested:
              if ( outMap.valid() )
              {
                  // new TMS driver info:
                  TMSOptions tms;
                  tms.url() = URI(
                      osgDB::concatPaths(layerFolder, "tms.xml"),
                      outEarthFile );

                  ImageLayerOptions layerOptions( layer->getName(), tms );
                  layerOptions.mergeConfig( layer->getInitialOptions().getConfig(true) );
                  layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

                  outMap->addImageLayer( new ImageLayer(layerOptions) );
              }
          }
          else
          {
              OE_WARN << LC << r.message << std::endl;
          }
      }
      else if ( verbose )
      {
          OE_NOTICE << LC << "Skipping disabled layer \"" << layer->getName() << "\"" << std::endl;
      }
  }

  // package any elevation layers that are enabled:
  counter = 0;
  ElevationLayerVector elevationLayers;
  map->getElevationLayers( elevationLayers );

  for( ElevationLayerVector::iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i, ++counter )
  {
      ElevationLayer* layer = i->get();
      //if ( layer->getElevationLayerOptions().enabled() == true )
      if ( layer->getEnabled() && layer->getVisible() )
      {
          std::string layerFolder = toLegalFileName( layer->getName() );
          if ( layerFolder.empty() )
              layerFolder = Stringify() << "elevation_layer_" << counter;

          if ( verbose )
          {
              OE_NOTICE << LC << "Packaging elevation layer \"" << layerFolder << "\"" << std::endl;
          }

          std::string layerRoot = osgDB::concatPaths( rootFolder, layerFolder );
          TMSPackager::Result r = packager.package( layer, layerRoot );

          if ( r.ok )
          {
              // save to the output map if requested:
              if ( outMap.valid() )
              {
                  // new TMS driver info:
                  TMSOptions tms;
                  tms.url() = URI(
                      osgDB::concatPaths(layerFolder, "tms.xml"),
                      outEarthFile );

                  ElevationLayerOptions layerOptions( layer->getName(), tms );
                  layerOptions.mergeConfig( layer->getInitialOptions().getConfig(true) );
                  layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

                  outMap->addElevationLayer( new ElevationLayer(layerOptions) );
              }
          }
          else
          {
              OE_WARN << LC << r.message << std::endl;
          }
      }
      else if ( verbose )
      {
          OE_NOTICE << LC << "Skipping disabled layer \"" << layer->getName() << "\"" << std::endl;
      }
  }

  // Finally, write an earth file if requested:
  if ( outMap.valid() )
  {
      MapNodeOptions outNodeOptions = mapNode->getMapNodeOptions();
      osg::ref_ptr<MapNode> outMapNode = new MapNode(outMap.get(), outNodeOptions);
      if ( !osgDB::writeNodeFile(*outMapNode.get(), outEarthFile) )
      {
          OE_WARN << LC << "Error writing earth file to \"" << outEarthFile << "\"" << std::endl;
      }
      else if ( verbose )
      {
          OE_NOTICE << LC << "Wrote earth file to \"" << outEarthFile << "\"" << std::endl;
      }
  }

  return true;
}

