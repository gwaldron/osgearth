/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

TMSExporter::TMSExporter()
: _dbOptions(""), _maxLevel(~0), _keepEmpties(false), _errorMessage(""), _canceled(false)
{  
}

/** Packages image and elevation layers as a TMS. */
int TMSExporter::exportTMS(MapNode* mapNode, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth, bool overwrite, const std::string& extension)
{     
  Map* map = mapNode->getMap();

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

  std::string outEarthFile = osgDB::concatPaths(path, outEarthName);

  
  // Create the TMS packager.
  TMSPackager packager;

  /*
  MultithreadedTileVisitor* v = new MultithreadedTileVisitor();
  v->setNumThreads(16);
  packager.setVisitor( v );
  */

  // Make the output directory if it doesn't exist  
  osgDB::makeDirectory(path);
  packager.setDestination(path);

  // Setup the osgDB options for the packager.
  osg::ref_ptr<osgDB::Options> options = new osgDB::Options(_dbOptions);
  packager.setWriteOptions( options.get() );
  
  // Add all the bounds
  for (unsigned int i = 0; i < bounds.size(); i++)
  {
      packager.getTileVisitor()->addExtent( osgEarth::GeoExtent(map->getProfile()->getSRS(), bounds[i]));
  }
  packager.setExtension(extension);  
  packager.getTileVisitor()->setProgressCallback( _progress.get() );
  packager.getTileVisitor()->setMaxLevel(_maxLevel);

  // Compute the total number of layers we are going to operate on.
  unsigned int totalLayers = map->getNumImageLayers() + map->getNumElevationLayers();  

  unsigned int layerNum = 1;
  
  // Package each image layer
  for (unsigned int i = 0; i < map->getNumImageLayers(); i++)
  {            
      osg::ref_ptr< ImageLayer > layer = map->getImageLayerAt(i);      
      std::stringstream buf;
      buf << "Packaging " << layer->getName() << " (" << layerNum << " of " << totalLayers << ")";
      _progress->setStatus(QString::fromStdString( buf.str()));
      packager.run(layer.get(), map->getProfile());
      packager.writeXML(layer.get(), map->getProfile());
      if (outMap)
      {
          std::string layerFolder = toLegalFileName( layer->getName() );

          // new TMS driver info:
          TMSOptions tms;
          tms.url() = URI(
              osgDB::concatPaths( layerFolder, "tms.xml" ),
              outEarthFile );

          ImageLayerOptions layerOptions( layer->getName(), tms );
          layerOptions.mergeConfig( layer->getInitialOptions().getConfig( true ) );
          layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

          outMap->addImageLayer( new ImageLayer( layerOptions ) );
      }
      layerNum++;
  }

  // Package each elevation layer
  for (unsigned int i = 0; i < map->getNumElevationLayers(); i++)
  {
      osg::ref_ptr< ElevationLayer > layer = map->getElevationLayerAt(i);      
      std::stringstream buf;
      buf << "Packaging " << layer->getName() << " (" << layerNum << " of " << totalLayers << ")";
      _progress->setStatus(QString::fromStdString( buf.str()));
      packager.run(layer.get(), map->getProfile());
      packager.writeXML(layer.get(), map->getProfile());

      if( outMap.valid() )
      {
          std::string layerFolder = toLegalFileName( layer->getName());

          // new TMS driver info:
          TMSOptions tms;
          tms.url() = URI(
              osgDB::concatPaths( layerFolder, "tms.xml" ),
              outEarthFile );

          ElevationLayerOptions layerOptions( layer->getName(), tms );
          layerOptions.mergeConfig( layer->getInitialOptions().getConfig( true ) );
          layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

          outMap->addElevationLayer( new ElevationLayer( layerOptions ) );
      }

      layerNum++;
  }

  // Write out an earth file if it was requested
    // Finally, write an earth file if requested:
    if( outMap.valid() )
    {
        MapNodeOptions outNodeOptions = mapNode->getMapNodeOptions();
        osg::ref_ptr<MapNode> outMapNode = new MapNode( outMap.get(), outNodeOptions );
        if( !osgDB::writeNodeFile( *outMapNode.get(), outEarthFile ) )
        {
            OE_WARN << LC << "Error writing earth file to \"" << outEarthFile << "\"" << std::endl;
        }
    }


  // Tell the progress dialog that we're finished and it can close
  _progress->complete();

  return 0;
}

#if 0
void TMSExporter::packageTaskComplete(int id)
{
    /*
  if (_progress.valid())
  {
      OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _m );

      _percentComplete += 1.0 - _taskProgress[id];
      _taskProgress[id] = 1.0;

      if ( _progress->isCanceled() || _progress->reportProgress(_percentComplete, _totalTasks) )
          cancel(_progress->message());
  }
  */
}
#endif

void TMSExporter::cancel(const std::string& message)
{
  _canceled = true;
  _errorMessage = message;
}

