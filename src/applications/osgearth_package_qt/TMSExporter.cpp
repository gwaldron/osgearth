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
    /*
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
      if (_callback->isCanceled())
        return;

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

      if (!_packageResult.ok)
      {
        _callback->message() = _packageResult.message;
        _callback->cancel();
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
  */
}


TMSExporter::TMSExporter()
: _dbOptions(""), _maxLevel(~0), _keepEmpties(false), _errorMessage(""), _canceled(false)
{  
}

/** Packages image and elevation layers as a TMS. */
int TMSExporter::exportTMS(MapNode* mapNode, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth, bool overwrite, const std::string& extension)
{     
  Map* map = mapNode->getMap();
  TMSPackager packager;
  osgDB::makeDirectory(path);
  packager.setDestination(path);
  
  // Add all the bounds
  for (unsigned int i = 0; i < bounds.size(); i++)
  {
      packager.getTileVisitor()->addExtent( osgEarth::GeoExtent(map->getProfile()->getSRS(), bounds[i]));
  }
  packager.setExtension(extension);
  //packager.getTileVisitor()->setProgressCallback(new PackageLayerProgressCallback(this, 0) );  
  packager.getTileVisitor()->setProgressCallback( _progress.get() );
  packager.getTileVisitor()->setMaxLevel(_maxLevel);

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
      layerNum++;
  }

  // Tell the progress dialog that we're finished and it can close
  _progress->complete();

  return 0;
}

#if 0
void TMSExporter::packageTaskProgress(int id, double complete)
{

  if (_progress.valid())
  {
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _mutex );
    _progress->reportProgress(complete

    /*
    _percentComplete += complete - _taskProgress[id];
    _taskProgress[id] = complete;

    if ( _progress->isCanceled() || _progress->reportProgress(_percentComplete, _totalTasks) )
        cancel(_progress->message());
        */
  }
}

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

