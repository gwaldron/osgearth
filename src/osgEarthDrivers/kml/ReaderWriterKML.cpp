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

#include "KMLOptions"
#include "KMLReader"

#define LC "[ReaderWriterKML] "

using namespace osgEarth::Drivers;

#ifdef OSGEARTH_HAVE_MINIZIP

#include "unzip.h"

namespace
{
    URI downloadKMZ( const URI& uri )
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
                    s_cache = std::string(osgCacheDir) + "/";
                else
                    s_cache = "osgearth_kmz_cache/";
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

    /**
     * Extracts a single file from a zip archive
     * (http://bytes.com/topic/c/answers/764381-reading-contents-zip-files)
     */
    std::string readFromZip( const std::string& url, const std::string& fileInZip )
    {
        URI fileURI(url);
        if ( osgDB::containsServerAddress(url) )
        {
            fileURI = downloadKMZ( url );
        }

        if ( fileURI.empty() ) {
            OE_WARN << LC << "KMZ download failed" << std::endl;
            return "";
        }

        std::string zipFile = fileURI.full();

        int err = UNZ_OK;
        unsigned size_buf = 5242880;
        void* buf;
        std::string sout;
        char filename_inzip[1024];
        unz_file_info file_info;

        unzFile uf = unzOpen( zipFile.c_str() );
        if ( uf == 0L )
        {
            OE_WARN << LC << "Failed to open zipped file" << std::endl;
            return sout;
        }

        if ( unzLocateFile( uf, fileInZip.c_str(), 0 ) )
        {
            OE_WARN << LC << "Failed to locate '" << fileInZip << "' in '" << zipFile << "'" << std::endl;
            return sout;
        }

        if ( unzGetCurrentFileInfo( uf, &file_info, filename_inzip, sizeof(filename_inzip), 0L, 0, 0L, 0) )
        {
            OE_WARN << LC << "Error with zipfile " << zipFile << std::endl;
            return sout;
        }

        buf = (void*)new char[size_buf];
        if ( buf == 0L )
        {
            OE_WARN << LC << "Out of memory" << std::endl;
            return sout;
        }

        err = unzOpenCurrentFilePassword( uf, 0L );
        if ( err != UNZ_OK )
        {
            OE_WARN << LC << "unzOpenCurrentFilePassword failed" << std::endl;
            return sout;
        }

        do
        {
            err = unzReadCurrentFile( uf, buf, size_buf );
            if ( err < 0 )
            {
                OE_WARN << LC << "Error in unzReadCurrentFile" << std::endl;
                sout = "";
                break;
            }
            if ( err > 0 )
            {
                for( unsigned i=0; i<err; ++i )
                {
                    sout.push_back( *(((char*)buf)+i) );
                }
            }
        }
        while( err > 0 );

        err = unzCloseCurrentFile( uf );
        if ( err != UNZ_OK )
        {
            //ignore it...
        }

        delete [] buf;
        return sout;
    }
}
#endif // OSGEARTH_HAVE_MINIZIP

//---------------------------------------------------------------------------

struct ReaderWriterKML : public osgDB::ReaderWriter
{
    ReaderWriterKML()
    {
        supportsExtension( "kml", "KML" );
        supportsExtension( "kmz", "KMZ" );
    }

    ReadResult readObject(const std::string& uri, const Options* options) const
    {
        return readNode( uri, options );
    }

    ReadResult readNode(const std::string& uri, const Options* options) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension(uri);
        if ( !acceptsExtension(ext) )
            return ReadResult::FILE_NOT_HANDLED;

        if ( ext == "kmz" )
        {
#ifdef OSGEARTH_HAVE_MINIZIP
            std::string data = readFromZip( uri, "doc.kml" );
            if ( !data.empty() )
            {
                std::stringstream buf(data);
                return readNode( buf, options );
            }
            else
            {
                OE_WARN << LC << "Failed to process KMZ file!" << std::endl;
                return ReadResult::ERROR_IN_READING_FILE;
            }
#else
            OE_WARN << LC << "KMZ support is not enabled. You must build osgEarth with MINIZIP." << std::endl;
            return ReadResult::ERROR_IN_READING_FILE;
#endif // OSGEARTH_HAVE_MINIZIP
        }

        // propagate the source URI along to the stream reader
        osg::ref_ptr<osgDB::Options> myOptions = options ?
            new osgDB::Options( *options ) : new osgDB::Options();
        myOptions->setPluginData( "osgEarth::ReaderWriterKML::ref_uri", (void*)&uri );

        URIStream in( uri );
        return readNode( in, options );
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

        // an optional URI context for resolving relative paths:
        URIContext uriContext;
        const std::string* cxValue = static_cast<const std::string*>( options->getPluginData( "osgEarth::ReaderWriterKML::ref_uri") );
        if ( cxValue )
            uriContext = *cxValue;

        // fire up a KML reader and parse the data.
        KMLReader reader( mapNode, kmlOptions );
        osg::Node* node = reader.read( in, uriContext );
        return ReadResult(node);
    }
};

REGISTER_OSGPLUGIN( kml, ReaderWriterKML )
