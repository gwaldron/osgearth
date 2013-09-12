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

namespace
{
  /** Packaging task for a single layer. */
  struct PackageLayer
  {
    void init(osgEarth::Map* map, osgEarth::ImageLayer* layer, osgDB::Options* options, const std::string& rootFolder, const std::string& layerFolder, bool verbose, bool overwrite, bool keepEmpties, unsigned int maxLevel, const std::string& extension, osgEarth::ProgressCallback* callback, std::vector< osgEarth::Bounds >& bounds)
    {
      _map = map;
      _imageLayer = layer;
      _options = options;
      _rootFolder = rootFolder;
      _layerFolder = layerFolder;
      _verbose = verbose;
      _overwrite = overwrite;
      _keepEmpties = keepEmpties;
      _maxLevel = maxLevel;
      _extension = extension;
      _bounds = bounds;
      _callback = callback;
      _packageResult.ok = false;
    }

    void init(osgEarth::Map* map, osgEarth::ElevationLayer* layer, osgDB::Options* options, const std::string& rootFolder, const std::string& layerFolder, bool verbose, bool overwrite, bool keepEmpties, unsigned int maxLevel, const std::string& extension, osgEarth::ProgressCallback* callback, std::vector< osgEarth::Bounds >& bounds)
    {
      _map = map;
      _elevationLayer = layer;
      _options = options;
      _rootFolder = rootFolder;
      _layerFolder = layerFolder;
      _verbose = verbose;
      _overwrite = overwrite;
      _keepEmpties = keepEmpties;
      _maxLevel = maxLevel;
      _extension = extension;
      _bounds = bounds;
      _callback = callback;
      _packageResult.ok = false;
    }

    void execute()
    {
      TMSPackager packager( _map->getProfile(), _options);

      packager.setVerbose( _verbose );
      packager.setOverwrite( _overwrite );
      packager.setKeepEmptyImageTiles( _keepEmpties );

      if ( _maxLevel != ~0 )
          packager.setMaxLevel( _maxLevel );

      if (_bounds.size() > 0)
      {
          for (unsigned int i = 0; i < _bounds.size(); ++i)
          {
              Bounds b = _bounds[i];            
              if ( b.isValid() )
                  packager.addExtent( osgEarth::GeoExtent(_map->getProfile()->getSRS(), b) );
          }
      }

      std::string layerRoot = osgDB::concatPaths( _rootFolder, _layerFolder );

      if (_imageLayer.valid())
      {
        if (_verbose)
          OE_NOTICE << LC << "Packaging image layer \"" << _layerFolder << "\"" << std::endl;

        _packageResult = packager.package( _imageLayer.get(), layerRoot, _callback, _extension );
      }
      else if (_elevationLayer.valid())
      {
        if (_verbose)
          OE_NOTICE << LC << "Packaging elevation layer \"" << _layerFolder << "\"" << std::endl;

        _packageResult = packager.package( _elevationLayer.get(), layerRoot, _callback );
      }
    }

    osg::ref_ptr<osgEarth::Map> _map;
    osg::ref_ptr<osgEarth::ImageLayer> _imageLayer;
    osg::ref_ptr<osgEarth::ElevationLayer> _elevationLayer;
    osg::ref_ptr<osgDB::Options> _options;
    std::string _rootFolder;
    std::string _layerFolder;
    bool _verbose;
    bool _overwrite;
    bool _keepEmpties;
    unsigned int _maxLevel;
    std::string _extension;
    std::vector< osgEarth::Bounds > _bounds;
    osg::ref_ptr<osgEarth::ProgressCallback> _callback;
    TMSPackager::Result _packageResult;
  };
}


TMSExporter::TMSExporter(const std::string& log)
: _dbOptions(""), _maxLevel(~0), _keepEmpties(false), _errorMessage("")
{
  unsigned num = 2 * OpenThreads::GetNumberOfProcessors();
  _taskService = new osgEarth::TaskService("TMS Packager", num);
}

/** Packages image and elevation layers as a TMS. */
int TMSExporter::exportTMS(MapNode* mapNode, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth, bool overwrite, const std::string& extension)
{
  if ( !mapNode )
  {
    _errorMessage = "Invalid MapNode";
    if (_progress.valid()) _progress->onCompleted();
    return 0;
  }

  // folder to which to write the TMS archive.
  std::string rootFolder = path;

  osg::ref_ptr<osgDB::Options> options = new osgDB::Options(_dbOptions);

  // create a folder for the output
  osgDB::makeDirectory(rootFolder);
  if ( !osgDB::fileExists(rootFolder) )
  {
    _errorMessage = "Failed to create root output folder";
    if (_progress.valid()) _progress->onCompleted();
    return 0;
  }

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

  std::string outEarthFile = osgDB::concatPaths(rootFolder, outEarthName);
  

  // semaphore and tasks collection for multithreading
  osgEarth::Threading::MultiEvent semaphore;
  osgEarth::TaskRequestVector tasks;
  int taskCount = 0;


  // package any image layers that are enabled and visible
  ImageLayerVector imageLayers;
  map->getImageLayers( imageLayers );

  unsigned imageCount = 0;
  for( ImageLayerVector::iterator i = imageLayers.begin(); i != imageLayers.end(); ++i, ++imageCount )
  {
      ImageLayer* layer = i->get();

      if ( layer->getEnabled() && layer->getVisible() )
      {
          std::string layerFolder = toLegalFileName( layer->getName() );
          if ( layerFolder.empty() )
              layerFolder = Stringify() << "image_layer_" << imageCount;

          ParallelTask<PackageLayer>* task = new ParallelTask<PackageLayer>( &semaphore );
          task->init(map, layer, options, rootFolder, layerFolder, true, overwrite, _keepEmpties, _maxLevel, extension, new PackageLayerProgressCallback(this, taskCount), bounds);
          tasks.push_back(task);
          taskCount++;
      }
  }

  // package any elevation layers that are enabled and visible
  ElevationLayerVector elevationLayers;
  map->getElevationLayers( elevationLayers );

  int elevCount = 0;
  for( ElevationLayerVector::iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i, ++elevCount )
  {
      ElevationLayer* layer = i->get();
      if ( layer->getEnabled() && layer->getVisible() )
      {
          std::string layerFolder = toLegalFileName( layer->getName() );
          if ( layerFolder.empty() )
              layerFolder = Stringify() << "elevation_layer_" << elevCount;

          ParallelTask<PackageLayer>* task = new ParallelTask<PackageLayer>( &semaphore );
          task->init(map, layer, options, rootFolder, layerFolder, true, overwrite, _keepEmpties, _maxLevel, extension, new PackageLayerProgressCallback(this, taskCount), bounds);
          tasks.push_back(task);
          taskCount++;
      }
  }


  // Run all the tasks in parallel
  _totalTasks = taskCount;
  _percentComplete = 0.0;

  semaphore.reset( _totalTasks );
  _taskProgress = std::vector<double>(_totalTasks, 0.0);

  for( TaskRequestVector::iterator i = tasks.begin(); i != tasks.end(); ++i )
        _taskService->add( i->get() );

  // Wait for them to complete
  semaphore.wait();


  // Add successfully packaged layers to the new map object and
  // write out the .earth file (if requested)
  if (outMap.valid())
  {
    for( TaskRequestVector::iterator i = tasks.begin(); i != tasks.end(); ++i )
    {
      PackageLayer* p = dynamic_cast<PackageLayer*>(i->get());
      if (p)
      {
        if (p->_packageResult.ok)
        {
          TMSOptions tms;
          tms.url() = URI(osgDB::concatPaths(p->_layerFolder, "tms.xml"), outEarthFile );

          if (p->_imageLayer.valid())
          {
            ImageLayerOptions layerOptions( p->_imageLayer->getName(), tms );
            layerOptions.mergeConfig( p->_imageLayer->getInitialOptions().getConfig(true) );
            layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

            outMap->addImageLayer( new ImageLayer(layerOptions) );
          }
          else
          {
            ElevationLayerOptions layerOptions( p->_elevationLayer->getName(), tms );
            layerOptions.mergeConfig( p->_elevationLayer->getInitialOptions().getConfig(true) );
            layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

            outMap->addElevationLayer( new ElevationLayer(layerOptions) );
          }
        }
        else
        {
          OE_WARN << LC << p->_packageResult.message << std::endl;
        }
      }
    }
  }

  if ( outMap.valid() )
  {
      MapNodeOptions outNodeOptions = mapNode->getMapNodeOptions();
      osg::ref_ptr<MapNode> outMapNode = new MapNode(outMap.get(), outNodeOptions);
      if ( !osgDB::writeNodeFile(*outMapNode.get(), outEarthFile) )
      {
          OE_WARN << LC << "Error writing earth file to \"" << outEarthFile << "\"" << std::endl;
      }
      else
      {
          OE_NOTICE << LC << "Wrote earth file to \"" << outEarthFile << "\"" << std::endl;
      }
  }


  // Mark the progress callback as completed
  if (_progress.valid()) _progress->onCompleted();

  return elevCount + imageCount;
}

void TMSExporter::packageTaskProgress(int id, double complete)
{
  if (_progress.valid())
  {
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _m );

    _percentComplete += complete - _taskProgress[id];
    _taskProgress[id] = complete;

    _progress->reportProgress(_percentComplete, _totalTasks);
  }
}

void TMSExporter::packageTaskComplete(int id)
{
  if (_progress.valid())
  {
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _m );

    _percentComplete += 1.0 - _taskProgress[id];
    _taskProgress[id] = 1.0;

    //if ( _progress.valid() && (_progress->isCanceled() || _progress->reportProgress(_completedTasks, _totalTasks)) )
    //{
    //    //Canceled
    //}

    _progress->reportProgress(_percentComplete, _totalTasks);
  }
}

