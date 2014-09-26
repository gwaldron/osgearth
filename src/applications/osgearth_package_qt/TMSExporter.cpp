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
#include <osgEarth/FileUtils>
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
    : _dbOptions(""),
    _maxLevel(~0),
    _keepEmpties(false),
    _concurrency(1),
    _mode(MODE_SINGLE),
    _totalTimeS(0.0)
{  
}

unsigned int TMSExporter::getConcurrency() const
{
    return _concurrency;
}

void TMSExporter::setConcurrency(unsigned int concurrency)
{
    _concurrency = concurrency;
}

double TMSExporter::getExportTime()
{
    return _totalTimeS;
}

/** Packages image and elevation layers as a TMS. */
int TMSExporter::exportTMS(MapNode* mapNode, const std::string& earthFilePath, const std::string& path, std::vector< osgEarth::Bounds >& bounds, const std::string& outEarth, bool overwrite, const std::string& extension)
{   
    _totalTimeS = 0.0;
    osg::Timer_t startTime = osg::Timer::instance()->tick();

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

    std::string tmpEarth;

    // Setup the visitor with the concurrency level and processing mode if it's set.
    if (_concurrency > 1)
    {
        if (_mode == MODE_MULTIPROCESS)
        {
            // Write out a temp earth file so the processes can work on it.
            // Determine the output path.  If we loaded from an earth file write out the temp file right next to it so relative paths will work the same.
            // Otherwise use the temp path
            OE_NOTICE << "Earth file path " << earthFilePath << std::endl;
            if (!earthFilePath.empty())
            {
                std::string root = osgDB::getFilePath(earthFilePath);
                root += "/";
                tmpEarth = getTempName(root, ".earth");
            }
            else
            {
                tmpEarth = getTempName(getTempPath(), ".earth");
            }
            OE_INFO << "Writing to " << tmpEarth << std::endl;
            osgDB::writeNodeFile(*mapNode, tmpEarth );         
            MultiprocessTileVisitor* v = new MultiprocessTileVisitor();
            v->setEarthFile(tmpEarth);
            v->setNumProcesses(_concurrency);
            packager.setVisitor(v);
        }
        else if (_mode == MODE_MULTITHREADED)
        {
            MultithreadedTileVisitor* v = new MultithreadedTileVisitor();          
            v->setNumThreads(_concurrency);
            packager.setVisitor(v);          
        }
    }

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
    packager.setOverwrite(overwrite);
    packager.getTileVisitor()->setProgressCallback( _progress.get() );
    packager.getTileVisitor()->setMaxLevel(_maxLevel);

    // Compute the total number of layers we are going to operate on.
    unsigned int totalLayers = map->getNumImageLayers() + map->getNumElevationLayers();  

    unsigned int layerNum = 1;

    // Package each image layer
    for (unsigned int i = 0; i < map->getNumImageLayers(); i++)
    {            
        // Don't continue if the export has been canceled
        if (_progress->isCanceled())
        {
            break;
        }
        osg::ref_ptr< ImageLayer > layer = map->getImageLayerAt(i);      
        std::stringstream buf;
        buf << "Packaging " << layer->getName() << " (" << layerNum << " of " << totalLayers << ")";
        OE_NOTICE << buf.str() << std::endl;
        _progress->setStatus(buf.str());
        packager.run(layer.get(), map);
        packager.writeXML(layer.get(), map);
        if (outMap)
        {
            std::string layerFolder = toLegalFileName( packager.getLayerName() );

            // new TMS driver info:
            TMSOptions tms;
            tms.url() = URI(
                osgDB::concatPaths( layerFolder, "tms.xml" ),
                outEarthFile );

            ImageLayerOptions layerOptions( packager.getLayerName(), tms );
            layerOptions.mergeConfig( layer->getInitialOptions().getConfig( true ) );
            layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

            outMap->addImageLayer( new ImageLayer( layerOptions ) );
        }
        layerNum++;
    }

    // Package each elevation layer
    for (unsigned int i = 0; i < map->getNumElevationLayers(); i++)
    {
        // Don't continue if the export has been canceled
        if (_progress->isCanceled())
        {
            break;
        }

        osg::ref_ptr< ElevationLayer > layer = map->getElevationLayerAt(i);      
        std::stringstream buf;
        buf << "Packaging " << layer->getName() << " (" << layerNum << " of " << totalLayers << ")";
        OE_NOTICE << buf.str() << std::endl;
        _progress->setStatus(buf.str());
        packager.run(layer.get(), map);
        packager.writeXML(layer.get(), map);

        if( outMap.valid() )
        {
            std::string layerFolder = toLegalFileName( packager.getLayerName() );

            // new TMS driver info:
            TMSOptions tms;
            tms.url() = URI(
                osgDB::concatPaths( layerFolder, "tms.xml" ),
                outEarthFile );

            ElevationLayerOptions layerOptions( packager.getLayerName(), tms );
            layerOptions.mergeConfig( layer->getInitialOptions().getConfig( true ) );
            layerOptions.cachePolicy() = CachePolicy::NO_CACHE;

            outMap->addElevationLayer( new ElevationLayer( layerOptions ) );
        }

        layerNum++;
    }

    // Don't continue if the export has been canceled
    if (!_progress->isCanceled())
    {
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
    }    

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    _totalTimeS = osg::Timer::instance()->delta_s(startTime, endTime );


    // Tell the progress dialog that we're finished and it can close
    _progress->complete();

    // Remove the temp earth file.
    if (!tmpEarth.empty())
    {
        remove(tmpEarth.c_str());
    }

    return 0;
}