/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
  /**
   * Shim between the QT exporter and the TMSPackager
   */
  class TMSExporter
  {
  public:    

    TMSExporter();    

    int exportTMS(osgEarth::MapNode* mapNode, const std::string& earthFilePath, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth="", bool overwrite=false, const std::string& extension="");

    std::string getDBOptions() { return _dbOptions; }
    void setDBOptions(const std::string& options) { _dbOptions = options; }
    unsigned getMaxLevel() { return _maxLevel; }
    void setMaxLevel(unsigned level) { _maxLevel = level; }

    bool getKeepEmpties() { return _keepEmpties; }
    void setKeepEmpties(bool keep) { _keepEmpties = keep; }
  
    ExportProgressCallback* getProgressCallback() const { return _progress; }
    void setProgressCallback(ExportProgressCallback* progress) { _progress = progress; }

    unsigned int getConcurrency() const;
    void setConcurrency(unsigned int concurrency);

    double getExportTime();

    enum ProcessingMode {
        MODE_SINGLE,
        MODE_MULTITHREADED,
        MODE_MULTIPROCESS        
    };

    ProcessingMode getProcessingMode() const { return _mode;}
    void setProcessingMode(ProcessingMode mode) { _mode = mode; }

  private:

    std::string _dbOptions;
    unsigned _maxLevel;
    bool _keepEmpties;
    unsigned int _concurrency;
    ProcessingMode _mode;

    OpenThreads::Mutex _mutex;       

    osg::ref_ptr<ExportProgressCallback> _progress;

    double _totalTimeS;
  };


  class TMSExporterWorkerThread : public osg::Referenced, public OpenThreads::Thread
  {
  public:
    TMSExporterWorkerThread(TMSExporter* exporter, osgEarth::MapNode* mapNode, const std::string& earthFilePath, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth="", bool overwrite=false, const std::string& extension="")
      : OpenThreads::Thread(), _exporter(exporter), _mapNode(mapNode), _earthFilePath(earthFilePath), _path(path), _bounds(bounds), _outEarth(outEarth), _overwrite(overwrite), _extension(extension)
    { }

    void run()
    {
      if (_exporter)
      {
        _exporter->exportTMS(_mapNode, _earthFilePath, _path, _bounds, _outEarth, _overwrite, _extension);        
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
    std::string _earthFilePath;
  };
}

#endif //TILER_TOOL_TMSEXPORTER_H

