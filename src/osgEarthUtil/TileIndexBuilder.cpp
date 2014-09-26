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

#include <osgEarthUtil/TileIndexBuilder>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/Progress>
#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

using namespace osgDB;
using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace std;

TileIndexBuilder::TileIndexBuilder()
{
}

void TileIndexBuilder::setProgressCallback( osgEarth::ProgressCallback* progress )
{
    _progress = progress;
}

void TileIndexBuilder::build(const std::string& indexFilename, const osgEarth::SpatialReference* srs)
{
    expandFilenames();

    if (!srs)
    {
        srs = osgEarth::SpatialReference::create("wgs84");
    }

    osg::ref_ptr< osgEarth::Util::TileIndex > index = osgEarth::Util::TileIndex::create( indexFilename, srs );

    _indexFilename = indexFilename;
    std::string indexDir = getFilePath( _indexFilename );    
    
    unsigned int total = _expandedFilenames.size();

    for (unsigned int i = 0; i < _expandedFilenames.size(); i++)
    {   
        std::string filename = _expandedFilenames[ i ];        

        GDALOptions opt;
        opt.url() = filename;
        
        osg::ref_ptr< ImageLayer > layer = new ImageLayer( ImageLayerOptions("", opt) );        

        bool ok = false;
                
        if ( layer.valid() )        
        {            
            osg::ref_ptr< TileSource > source = layer->getTileSource();
            if (source.valid())
            {
                for (DataExtentList::iterator itr = source->getDataExtents().begin(); itr != source->getDataExtents().end(); ++itr)
                {
                    // We want the filename as it is relative to the index file                
                    std::string relative = getPathRelative( indexDir, filename );                
                    index->add( relative, *itr);    
                    ok = true;
                }                
            }
        }        

        if (_progress.valid())
        {
            std::stringstream buf;
            if (ok)
            {
                buf << "Processed ";
            }
            else
            {
                buf << "Skipped ";
            }

            buf << filename;
            _progress->reportProgress( (double)i+1, (double)total, buf.str() );
        }
    }

    osg::Timer_t end = osg::Timer::instance()->tick();    
}

void TileIndexBuilder::expandFilenames()
{
    // Expand the filenames since they might contain directories    
    for (unsigned int i = 0; i < _filenames.size(); i++)
    {
        std::string filename = _filenames[i];
        if (osgDB::fileType(filename) == osgDB::DIRECTORY)
        {            
            CollectFilesVisitor v;
            v.traverse( filename );
            for (unsigned int j = 0; j < v.filenames.size(); j++)
            {
                _expandedFilenames.push_back( v.filenames[ j ] );
            }
        }   
        else
        {
            _expandedFilenames.push_back( filename );
        }
    }
}

