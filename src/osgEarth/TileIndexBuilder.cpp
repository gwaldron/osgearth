/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/TileIndexBuilder>
#include <osgEarth/FileUtils>
#include <osgEarth/Progress>
#include <osgEarth/GDAL>
#include <osgDB/FileUtils>

using namespace osgDB;
using namespace osgEarth;
using namespace osgEarth::Contrib;

TileIndexBuilder::TileIndexBuilder()
{
}

void TileIndexBuilder::setProgressCallback( ProgressCallback* progress )
{
    _progress = progress;
}

void TileIndexBuilder::build(const std::string& indexFilename, const SpatialReference* srs)
{
    expandFilenames();

    if (!srs)
    {
        srs = osgEarth::SpatialReference::create("wgs84");
    }

    osg::ref_ptr< TileIndex > index = TileIndex::create( indexFilename, srs );

    _indexFilename = indexFilename;
    std::string indexDir = getFilePath( _indexFilename );
    
    unsigned int total = _expandedFilenames.size();

    for (unsigned int i = 0; i < _expandedFilenames.size(); i++)
    {   
        std::string filename = _expandedFilenames[ i ];

        // Who deletes this?
        GDALImageLayer* layer = new GDALImageLayer();
        layer->setURL(filename);

        bool ok = false;

        DataExtentList dataExtents;
        layer->getDataExtents(dataExtents);
                
        if ( layer->open().isOK() )
        {
            for (DataExtentList::const_iterator itr = dataExtents.begin(); itr != dataExtents.end(); ++itr)
            {
                // We want the filename as it is relative to the index file                
                std::string relative = getPathRelative(indexDir, filename);
                index->add(relative, *itr);
                ok = true;
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

