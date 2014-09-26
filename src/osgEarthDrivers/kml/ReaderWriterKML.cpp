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
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/FileCache>
#include <osgDB/Registry>
#include <osgDB/FileUtils>
#include <osgDB/Archive>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>

#include "KMLOptions"
#include "KMLReader"
#include "KMZArchive"

#define LC "[ReaderWriterKML] "

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth_kml;

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

    osgDB::ReaderWriter::ReadResult readObject(const std::string& url, const osgDB::Options* options) const
    {
        return readNode( url, options );
    }

    osgDB::ReaderWriter::ReadResult readObject(std::istream& in, const osgDB::Options* dbOptions ) const
    {
        return readNode(in, dbOptions);
    }

    osgDB::ReaderWriter::ReadResult readNode(const std::string& url, const osgDB::Options* dbOptions) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension(url);
        if ( !acceptsExtension(ext) )
            return ReadResult::FILE_NOT_HANDLED;

        if ( ext == "kmz" )
        {
            return URI(url + "/.kml").readNode( dbOptions ).releaseNode();
        }
        else
        {
            // propagate the source URI along to the stream reader
            osg::ref_ptr<osgDB::Options> myOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
            URIContext(url).apply( myOptions.get() );
            return readNode( URIStream(url), myOptions.get() );
        }
    }

    osgDB::ReaderWriter::ReadResult readNode(std::istream& in, const osgDB::Options* options ) const
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

        // fire up a KML reader and parse the data.
        KMLReader reader( mapNode, kmlOptions );
        osg::Node* node = reader.read( in, options );
        return ReadResult(node);
    }

#ifdef SUPPORT_KMZ

    osgDB::ReaderWriter::ReadResult openArchive( const std::string& url, ArchiveStatus status, unsigned int dummy, const osgDB::Options* options =0L ) const
    {
        // Find the archive for this thread. We store one KMZ archive instance per thread 
        // so that the minizip library can work in parallel
        osg::ref_ptr<KMZArchive>& kmz = const_cast<ReaderWriterKML*>(this)->_archive.get();
        if ( !kmz.valid() )
            kmz = new KMZArchive( URI(url) );

        return ReadResult( kmz.release() );
    }

private:

    Threading::PerThread< osg::ref_ptr<KMZArchive> > _archive;

#endif // SUPPORT_KMZ
};

REGISTER_OSGPLUGIN( kml, ReaderWriterKML )
