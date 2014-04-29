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

#ifndef TILER_TOOL_TMSEXPORTER_H
#define TILER_TOOL_TMSEXPORTER_H 1

#include <vector>

#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/Progress>
#include "ExportProgress.h"

namespace PackageQt
{
  class TMSExporter
  {
  public:

    TMSExporter();

    int exportTMS(osgEarth::MapNode* mapNode, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth="", bool overwrite=false, const std::string& extension="");

    std::string getDBOptions() { return _dbOptions; }
    void setDBOptions(const std::string& options) { _dbOptions = options; }
    unsigned getMaxLevel() { return _maxLevel; }
    void setMaxLevel(unsigned level) { _maxLevel = level; }

    bool getKeepEmpties() { return _keepEmpties; }
    void setKeepEmpties(bool keep) { _keepEmpties = keep; }

    std::string getErrorMessage() { return _errorMessage; }

    void setProgressCallback(ExportProgressCallback* progress) { _progress = progress; }

    void cancel(const std::string& message="");
    bool isCanceled() { return _canceled; }

  protected:
    //friend class PackageLayerProgressCallback;

      /*
    void packageTaskProgress(int id, double percentComplete);
    void packageTaskComplete(int id);
    */

  private:

    std::string _dbOptions;
    unsigned _maxLevel;
    bool _keepEmpties;
    std::string _errorMessage;
    bool _canceled;

    OpenThreads::Mutex _mutex;   

    double _percentComplete;

    osg::ref_ptr<ExportProgressCallback> _progress;
  };


  class TMSExporterWorkerThread : public osg::Referenced, public OpenThreads::Thread
  {
  public:
    TMSExporterWorkerThread(TMSExporter* exporter, osgEarth::MapNode* mapNode, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth="", bool overwrite=false, const std::string& extension="")
      : OpenThreads::Thread(), _exporter(exporter), _mapNode(mapNode), _path(path), _bounds(bounds), _outEarth(outEarth), _overwrite(overwrite), _extension(extension)
    { }

    void run()
    {
      if (_exporter)
      {
        _exporter->exportTMS(_mapNode, _path, _bounds, _outEarth, _overwrite, _extension);        
      }
    }

  private:
    TMSExporter* _exporter;
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    std::string _path;
    std::vector< osgEarth::Bounds > _bounds;
    std::string _outEarth;
    bool _overwrite;
    std::string _extension;
  };
}

#endif //TILER_TOOL_TMSEXPORTER_H

