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
#include "KMLOptions"
#include "KMLReader"

#define LC "[ReaderWriterKML] "

using namespace osgEarth::Drivers;


struct KMLReaderWriter : public osgDB::ReaderWriter
{
    KMLReaderWriter()
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

        // an optional URI context for resolving relative paths:
        URIContext uriContext;
        const std::string* cxValue = static_cast<const std::string*>( options->getPluginData( "osgEarth::ReaderWriterKML::ref_uri") );
        if ( cxValue )
            uriContext = *cxValue;

        // fire up a KML reader and parse the data.
        KMLReader reader( mapNode );
        osg::Node* node = reader.read( in, uriContext );
        return ReadResult(node);
    }
};

REGISTER_OSGPLUGIN( kml, KMLReaderWriter )
