/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/FileCache>
#include <osgDB/Registry>
#include <osgDB/FileUtils>
#include <osgDB/Archive>
#include <osgEarth/Containers>
#include <osgEarth/Registry>
#include <osgEarth/Threading>

#include "KMLOptions"
#include "KMLReader"
#include "KMZArchive"

#define LC "[ReaderWriterKML] "

using namespace osgEarth;
using namespace osgEarth_kml;

//---------------------------------------------------------------------------

struct ReaderWriterKML : public osgDB::ReaderWriter
{
    ReaderWriterKML()
    {
        supportsExtension( "kml", "KML" );
        supportsExtension( "kmz", "KMZ" );
        osgDB::Registry::instance()->addArchiveExtension("kmz");

        // Not supported for archives:
        //osgDB::Registry::instance()->addFileExtensionAlias("kmz", "zip"); 
    }

    osgDB::ReaderWriter::ReadResult readObject(const std::string& url, const osgDB::Options* options) const override
    {
        return readNode( url, options );
    }

    osgDB::ReaderWriter::ReadResult readObject(std::istream& in, const osgDB::Options* dbOptions ) const override
    {
        return readNode(in, dbOptions);
    }

    osgDB::ReaderWriter::ReadResult readNode(const std::string& url, const osgDB::Options* dbOptions) const override
    {
        std::string ext = osgDB::getLowerCaseFileExtension(url);
        if ( !acceptsExtension(ext) )
            return ReadResult::FILE_NOT_HANDLED;

        if ( ext == "kmz" )
        {
            // redirect to the archive.
            return URI(url + "/doc.kml").readNode(dbOptions).releaseNode();
        }
        else
        {
            // propagate the source URI along to the stream reader
            OE_DEBUG << LC << "Reading KML from " << url << std::endl;
            osg::ref_ptr<osgDB::Options> myOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);
            URIContext(url).store( myOptions.get() );
            return readNode( URIStream(url), myOptions.get() );
        }
    }

    osgDB::ReaderWriter::ReadResult readNode(std::istream& in, const osgDB::Options* options ) const override
    {
        // this plugin can receive an optional MapNode* in options.
        MapNode* mapNode = const_cast<MapNode*>(
            static_cast<const MapNode*>( options->getPluginData("osgEarth::MapNode")) );

        // grab the KMLOptions if present
        const KML::KMLOptions* kmlOptions =
            static_cast<const KML::KMLOptions*>(options->getPluginData("osgEarth::KMLOptions") );

        // fire up a KML reader and parse the data.
        KMLReader reader( mapNode, kmlOptions );
        osg::Node* node = reader.read( in, options );
        return ReadResult(node);
    }

    osgDB::ReaderWriter::ReadResult openArchive(const std::string& url, ArchiveStatus status, unsigned blockSizeHint, const osgDB::Options* options =0L) const override
    {
        if ( osgDB::getLowerCaseFileExtension(url) == "kmz" )
        {
            OE_NOTICE << LC << "Opening KMZ archive at \"" << url << "\"\n";
            return new KMZArchive( URI(url), options );
        }
        else return ReadResult::FILE_NOT_HANDLED;
    }
};

REGISTER_OSGPLUGIN( kml, ReaderWriterKML )
