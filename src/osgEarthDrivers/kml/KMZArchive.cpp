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
#include "KMZArchive"
#ifdef SUPPORT_KMZ

#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/ObjectWrapper>
#include <osgEarth/HTTPClient>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>

#define LC "[KMZArchive] "

using namespace osgEarth;

namespace
{
    URI downloadToCache( const URI& uri )
    {
        // get a handle on the file cache. This is a temporary setup just to get things
        // working.
        static Threading::Mutex s_fcMutex;

        static URIContext s_cache;
        if ( s_cache.empty() )
        {
            Threading::ScopedMutexLock exclusiveLock(s_fcMutex);
            if ( s_cache.empty() )
            {
                const char* osgCacheDir = ::getenv("OSG_FILE_CACHE");
                if ( osgCacheDir )
                    s_cache = URIContext(std::string(osgCacheDir) + "/");
                else
                    s_cache = URIContext("osgearth_kmz_cache/");
            }
        }

        URI cachedFile( osgDB::getSimpleFileName(uri.full()), s_cache);

        if ( !osgDB::fileExists(cachedFile.full()) )
        {
            osgDB::makeDirectoryForFile(cachedFile.full());
            HTTPClient::download( uri.full(), cachedFile.full() );
        }

        if ( osgDB::fileExists(cachedFile.full()) )
            return cachedFile;

        return URI();
    }
}

//------------------------------------------------------------------------

KMZArchive::KMZArchive( const URI& archiveURI ) :
_archiveURI( archiveURI ),
_buf( 0L ),
_bufsize( 1024000 )
{
    supportsExtension( "kmz", "KMZ" );

    // download the KMZ to a local cache - cannot open zip files remotely.
    URI localURI( archiveURI );
    if ( osgDB::containsServerAddress(archiveURI.full()) )
    {
        localURI = downloadToCache( archiveURI );
    }

    _uf = unzOpen( localURI.full().c_str() );
    _buf = (void*)new char[_bufsize];
}

KMZArchive::~KMZArchive()
{
    if ( _buf )
        delete [] _buf;
}

void 
KMZArchive::close()
{
    _uf = 0;
}

/** Get the file name which represents the archived file.*/
std::string 
KMZArchive::getArchiveFileName() const
{
    return _archiveURI.base();
}

/** Get the file name which represents the master file recorded in the Archive.*/
std::string
KMZArchive::getMasterFileName() const 
{
    return "doc.kml";
}

/** return true if file exists in archive.*/
bool
KMZArchive::fileExists(const std::string& filename) const
{
    //todo
    return false;
}

/** return type of file. */
osgDB::FileType
KMZArchive::getFileType(const std::string& filename) const
{
    return osgDB::REGULAR_FILE;
}

/** Get the full list of file names available in the archive.*/
bool 
KMZArchive::getFileNames(FileNameList& fileNames) const
{
    return false;
}

/** return the contents of a directory.
* returns an empty array on any error.*/
osgDB::DirectoryContents 
KMZArchive::getDirectoryContents(const std::string& dirName) const
{
    return osgDB::DirectoryContents();
}

/** reads a file from the archive into an io buffer. */
bool 
KMZArchive::readToBuffer( const std::string& fileInZip, std::ostream& iobuf ) const
{
    // help from:
    // http://bytes.com/topic/c/answers/764381-reading-contents-zip-files


    //OE_INFO << LC << "Attempting to read \"" << fileInZip << "\" from \"" << _archiveURI.base() << "\"" << std::endl;


    int err = UNZ_OK;
    unz_file_info file_info;
    char filename_inzip[2048];
    bool got_file_info = false;

    if ( _uf == 0 )
    {
        OE_WARN << LC << "Archive is not open." << std::endl;
        return false;
    }

    if ( fileInZip == ".kml" )
    {
        // special case; first try the master file (doc.kml), then failing that, look
        // for the first KML file in the archive.
        if ( unzLocateFile( _uf, "doc.kml", 0 ) != 0 )
        {
            if ( unzGoToFirstFile( _uf ) != UNZ_OK )
            {
                OE_WARN << LC << "Archive is empty" << std::endl;
                return false;
            }

            while( err == UNZ_OK )
            {
                if ( unzGetCurrentFileInfo( _uf, &file_info, filename_inzip, sizeof(filename_inzip), 0L, 0, 0L, 0) )
                {
                    OE_WARN << LC << "Error with zipfile " << _archiveURI.base() << std::endl;
                    return false;
                }

                got_file_info = true;
                std::string lc = osgEarth::toLower( std::string(filename_inzip) );
                if ( endsWith( lc, ".kml" ) )
                {
                    break;
                }

                err = unzGoToNextFile( _uf );
            }

            if ( err != UNZ_OK )
            {
                OE_WARN << LC << "No KML file found in archive" << std::endl;
                return false;
            }
        }
    }

    else if ( unzLocateFile( _uf, fileInZip.c_str(), 0 ) )
    {
        OE_WARN << LC << "Failed to locate '" << fileInZip << "' in '" << _archiveURI.base() << "'" << std::endl;
        return false;
    }

    if ( !got_file_info )
    {
        if ( unzGetCurrentFileInfo( _uf, &file_info, filename_inzip, sizeof(filename_inzip), 0L, 0, 0L, 0) )
        {
            OE_WARN << LC << "Error with zipfile " << _archiveURI.base() << std::endl;
            return false;
        }
    }

    err = unzOpenCurrentFilePassword( _uf, 0L );
    if ( err != UNZ_OK )
    {
        OE_WARN << LC << "unzOpenCurrentFilePassword failed" << std::endl;
        return false;
    }

    do
    {
        err = unzReadCurrentFile( _uf, _buf, _bufsize );
        if ( err < 0 )
        {
            OE_WARN << LC << "Error in unzReadCurrentFile" << std::endl;
            break;
        }
        if ( err > 0 )
        {
            for( unsigned i=0; i<(unsigned)err; ++i )
            {
                iobuf.put( *(((char*)_buf)+i) );
            }
        }
    }
    while( err > 0 );

    err = unzCloseCurrentFile( _uf );
    if ( err != UNZ_OK )
    {
        //ignore it...
    }

    return true;
}

bool
KMZArchive::isAcceptable(const std::string& filename, const osgDB::Options* options) const
{
    if (!options ||
        options->getDatabasePathList().size() == 0 ||
        options->getDatabasePathList()[0] != _archiveURI.full() )
    {
        //OE_INFO << "Rejected: " << filename << std::endl;
        return false;
    }
    return true;
}

osgDB::ReaderWriter::ReadResult
KMZArchive::readImage(const std::string& filename, const osgDB::Options* options) const
{
    if ( isAcceptable(filename, options) )
    {
        osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(
            osgDB::getLowerCaseFileExtension( filename ) );
        if ( rw )
        {
            std::stringstream iobuf;
            if ( readToBuffer( filename, iobuf ) )
            {
                osg::ref_ptr<osgDB::Options> myOptions = Registry::instance()->cloneOrCreateOptions(options);
                URIContext(*_archiveURI).add(filename).apply( myOptions.get() );
                return rw->readImage( iobuf, myOptions.get() );
            }
            else return ReadResult::ERROR_IN_READING_FILE;
        }
    }
    return ReadResult::FILE_NOT_HANDLED;
}

osgDB::ReaderWriter::ReadResult
KMZArchive::readNode(const std::string& filename, const osgDB::Options* options) const
{
    if ( isAcceptable(filename, options) )
    {
        osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(
            osgDB::getLowerCaseFileExtension( filename ) );
        if ( rw )
        {
            std::stringstream iobuf;
            if ( readToBuffer( filename, iobuf ) )
            {
                osg::ref_ptr<osgDB::Options> myOptions = Registry::instance()->cloneOrCreateOptions(options);
                URIContext(*_archiveURI).add(filename).apply( myOptions.get() );
                return rw->readNode( iobuf, myOptions.get() );
            }
            else return ReadResult::ERROR_IN_READING_FILE;
        }
    }
    return ReadResult::FILE_NOT_HANDLED;
}

osgDB::ReaderWriter::ReadResult
KMZArchive::readObject(const std::string& filename, const osgDB::Options* options) const
{
    if ( isAcceptable(filename, options) )
    {
        osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(
            osgDB::getLowerCaseFileExtension( filename ) );
        if ( rw )
        {
            std::stringstream iobuf;
            if ( readToBuffer( filename, iobuf ) )
            {
                osg::ref_ptr<osgDB::Options> myOptions = Registry::instance()->cloneOrCreateOptions(options);
                URIContext(*_archiveURI).add(filename).apply( myOptions.get() );
                return rw->readObject( iobuf, myOptions.get() );
            }
            else return ReadResult::ERROR_IN_READING_FILE;
        }
    }
    return ReadResult::FILE_NOT_HANDLED;
}

#endif // SUPPORT_KMZ
