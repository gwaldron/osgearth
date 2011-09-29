/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/ObjectWrapper>
#include <osgDB/FileCache>
#include <osgDB/Registry>
#include <osgDB/FileUtils>
#include <osgDB/Archive>
#include <osgEarth/ThreadingUtils>

#include "KMLOptions"
#include "KMLReader"

#define LC "[ReaderWriterKML] "

using namespace osgEarth::Drivers;

#ifdef OSGEARTH_HAVE_MINIZIP
#    include "unzip.h"
#    define SUPPORT_KMZ
#endif

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


//---------------------------------------------------------------------------

#ifdef SUPPORT_KMZ

struct KMZArchive : public osgDB::Archive
{
    KMZArchive( const URI& archiveURI )
        : _archiveURI( archiveURI ),
          _buf( 0L ),
          _bufsize( 1024000 )
    {
        // todo: download the KMZ to the cache in the URI context.

        URI localURI( archiveURI );
        if ( osgDB::containsServerAddress(archiveURI.full()) )
        {
            localURI = downloadToCache( archiveURI );
        }

        _uf = unzOpen( localURI.full().c_str() );
        _buf = (void*)new char[_bufsize];
    }

    virtual ~KMZArchive()
    {
        if ( _buf )
            delete [] _buf;
    }

    void close()
    {
        _uf = 0;
    }

    /** Get the file name which represents the archived file.*/
    virtual std::string getArchiveFileName() const
    {
        return _archiveURI.base();
    }

    /** Get the file name which represents the master file recorded in the Archive.*/
    virtual std::string getMasterFileName() const 
    {
        return "doc.kml";
    }

    /** return true if file exists in archive.*/
    virtual bool fileExists(const std::string& filename) const
    {
        //todo
        return false;
    }

    /** return type of file. */
    osgDB::FileType getFileType(const std::string& filename) const
    {
        return osgDB::REGULAR_FILE;
    }

    typedef osgDB::DirectoryContents FileNameList;

    /** Get the full list of file names available in the archive.*/
    bool getFileNames(FileNameList& fileNames) const
    {
        return false;
    }

    /** return the contents of a directory.
      * returns an empty array on any error.*/
    osgDB::DirectoryContents getDirectoryContents(const std::string& dirName) const
    {
        return osgDB::DirectoryContents();
    }

    /** reads a file from the archive into an io buffer. */
    bool readToBuffer( const std::string& fileInZip, std::ostream& iobuf ) const
    {
        // help from:
        // http://bytes.com/topic/c/answers/764381-reading-contents-zip-files

        int err = UNZ_OK;
        unz_file_info file_info;
        char filename_inzip[2048];

        if ( _uf == 0 )
        {
            OE_WARN << LC << "Archive is not open." << std::endl;
            return false;
        }

        if ( unzLocateFile( _uf, fileInZip.c_str(), 0 ) )
        {
            OE_WARN << LC << "Failed to locate '" << fileInZip << "' in '" << _archiveURI.base() << "'" << std::endl;
            return false;
        }

        if ( unzGetCurrentFileInfo( _uf, &file_info, filename_inzip, sizeof(filename_inzip), 0L, 0, 0L, 0) )
        {
            OE_WARN << LC << "Error with zipfile " << _archiveURI.base() << std::endl;
            return false;
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
                for( unsigned i=0; i<err; ++i )
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

    virtual ReadResult readImage(const std::string& filename, const Options* options =NULL) const
    {
        std::stringstream iobuf;
        if ( readToBuffer( filename, iobuf ) )
        {
            osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(
                osgDB::getLowerCaseFileExtension( filename ) );
            return rw ? rw->readImage( iobuf, options ) : ReadResult::FILE_NOT_HANDLED;
        }
        else return ReadResult::ERROR_IN_READING_FILE;
    }

    virtual ReadResult readNode(const std::string& filename, const Options* options =NULL) const
    {
        std::stringstream iobuf;
        if ( readToBuffer( filename, iobuf ) )
        {
            osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(
                osgDB::getLowerCaseFileExtension( filename ) );
            return rw ? rw->readNode( iobuf, options ) : ReadResult::FILE_NOT_HANDLED;
        }
        else return ReadResult::ERROR_IN_READING_FILE;
    }

    // stubbed out
    ReadResult readObject(const std::string&, const Options* =NULL) const { return ReadResult::NOT_IMPLEMENTED; }
    ReadResult readShader(const std::string&, const Options* =NULL) const { return ReadResult::NOT_IMPLEMENTED; }
    ReadResult readHeightField(const std::string&, const Options* =NULL) const { return ReadResult::NOT_IMPLEMENTED; }
    WriteResult writeObject(const osg::Object&, const std::string&,const Options* =NULL) const { return WriteResult::NOT_IMPLEMENTED; }
    WriteResult writeImage(const osg::Image&, const std::string&,const Options* =NULL) const { return WriteResult::NOT_IMPLEMENTED; }
    WriteResult writeHeightField(const osg::HeightField&,const std::string&,const Options* =NULL) const { return WriteResult::NOT_IMPLEMENTED; }
    WriteResult writeNode(const osg::Node&, const std::string&,const Options* =NULL) const { return WriteResult::NOT_IMPLEMENTED; }
    WriteResult writeShader(const osg::Shader&, const std::string&, const Options* =NULL) const { return WriteResult::NOT_IMPLEMENTED; }

private:
    URI            _archiveURI;
    unzFile        _uf;
    void*          _buf;
    unsigned       _bufsize;
};

#endif // SUPPORT_KMZ

//---------------------------------------------------------------------------

struct ReaderWriterKML : public osgDB::ReaderWriter
{
    ReaderWriterKML()
    {
        supportsExtension( "kml", "KML" );

#ifdef SUPPORT_KMZ
        supportsExtension( "kmz", "KMZ" );
#endif // SUPPORT_KMZ
    }

    ReadResult readObject(const std::string& url, const Options* options) const
    {
        return readNode( url, options );
    }

    ReadResult readNode(const std::string& url, const Options* options) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension(url);
        if ( !acceptsExtension(ext) )
            return ReadResult::FILE_NOT_HANDLED;

        OE_INFO << LC << "Reading " << url << std::endl;

        if ( ext == "kmz" )
        {
            return readNodeFromKMZ( url, options );
        }
        else
        {
            // propagate the source URI along to the stream reader
            osg::ref_ptr<osgDB::Options> myOptions = options ?
                new osgDB::Options( *options ) : new osgDB::Options();

            // propagate the URI context along to the stream reader.
            URIContext(url).store( myOptions.get() );

            URIStream in( url );
            return readNode( in, myOptions.get() );
        }
    }

    ReadResult readNode(std::istream& in, const Options* options ) const
    {
        if ( !options )
            return ReadResult("Missing required MapNode option");

        // this plugin requires that you pass in a MapNode* in options.
        MapNode* mapNode = const_cast<MapNode*>(
            static_cast<const MapNode*>( options->getPluginData("osgEarth::MapNode")) );
        if ( !mapNode )
            return ReadResult("Missing required MapNode option");

        // grab the KMLOptions if present
        const KMLOptions* kmlOptions =
            static_cast<const KMLOptions*>(options->getPluginData("osgEarth::KMLOptions") );

        // Grab the URIContext from the options (since we're reading from a stream)
        URIContext uriContext( options );

        // fire up a KML reader and parse the data.
        KMLReader reader( mapNode, kmlOptions );
        osg::Node* node = reader.read( in, uriContext );
        return ReadResult(node);
    }

    ReadResult readNodeFromKMZ( const std::string& url, const Options* options ) const
    {
#ifdef SUPPORT_KMZ
        
        // first, break the url into its components. A KMZ url can have (a) just a KMZ file,
        // (b) an internal file prepended to the KMZ filename with a '!' delimiter. For example:
        //
        // "archive.kmz" => loads "doc.kml" from "archive.kmz".
        // "file.png|archive.kmz" => loads "file.png" from "archive.kmz".

        StringVector parts;
        StringTokenizer( url, parts, "|", "", false);

        if ( parts.size() >= 1 )
        {
            std::string archivePart = parts[parts.size()-1];
            std::string filePart;
            if ( parts.size() >= 2 )
            {
                parts.resize( parts.size()-1 );
                std::string filePart = joinStrings( parts, '|' );
            }

            osg::ref_ptr<KMZArchive>& kmz = const_cast<ReaderWriterKML*>(this)->_archive.get();
            if ( !kmz.valid() )
                kmz = new KMZArchive( URI(archivePart) );

            // use the KMZ's "master filename" as a default if necessary.
            if ( filePart.empty() )
                filePart = kmz->getMasterFileName();

            // propagate the URI context.            
            osg::ref_ptr<osgDB::Options> myOptions = options ? new osgDB::Options( *options ) : new osgDB::Options();
            URIContext(filePart, archivePart).store(myOptions.get());

            // and read.
            return kmz->readNode(filePart, myOptions.get());
        }
        else
        {
            OE_WARN << LC << "Illegal KMZ url: " << url << std::endl;
            return ReadResult::FILE_NOT_FOUND;
        }

#else
        OE_WARN << LC << "KMZ support is not enabled. You must build osgEarth with MINIZIP." << std::endl;
        return ReadResult::ERROR_IN_READING_FILE;
#endif // SUPPORT_KMZ
    }

private:

    Threading::PerThread< osg::ref_ptr<KMZArchive> > _archive;
};

REGISTER_OSGPLUGIN( kml, ReaderWriterKML )
