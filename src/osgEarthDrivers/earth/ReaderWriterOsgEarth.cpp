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
#include "EarthFileSerializer"
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>
#include <sstream>

using namespace osgEarth;

#define LC "[ReaderWriterEarth] "

class ReaderWriterEarth : public osgDB::ReaderWriter
{
    public:
        ReaderWriterEarth() {}

        virtual const char* className()
        {
            return "OSG Earth ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "earth" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            return readNode( file_name, options );
        }

        virtual ReadResult readObject(std::istream& in, const Options* options) const
        {
            return readNode( in, options );
        }

        virtual WriteResult writeNode(const osg::Node& node, const std::string& fileName, const Options* options ) const
        {
            if ( !acceptsExtension( osgDB::getFileExtension(fileName) ) )
                return WriteResult::FILE_NOT_HANDLED;

            std::ofstream out( fileName.c_str());
            if ( out.is_open() )
                return writeNode( node, out, options );

            return WriteResult::ERROR_IN_WRITING_FILE;            
        }

        virtual WriteResult writeNode(const osg::Node& node, std::ostream& out, const Options* options ) const
        {
            osg::Node* searchNode = const_cast<osg::Node*>( &node );
            MapNode* mapNode = MapNode::findMapNode( searchNode );
            if ( !mapNode )
                return WriteResult::ERROR_IN_WRITING_FILE; // i.e., no MapNode found in the graph.

            // serialize the map node to a generic Config object:
            EarthFileSerializer2 ser;
            Config conf = ser.serialize( mapNode );

            // dump that Config out as XML.
            osg::ref_ptr<XmlDocument> xml = new XmlDocument( conf );
            xml->store( out );

            return WriteResult::FILE_SAVED;
        }

        virtual ReadResult readNode(const std::string& fileName, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( fileName );
            if ( !acceptsExtension( ext ) )
                return ReadResult::FILE_NOT_HANDLED;

            // See if the filename starts with server: and strip it off.  This will trick OSG into
            // passing in the filename to our plugin instead of using the CURL plugin if the filename
            // contains a URL.  So, if you want to read a URL, you can use the following format
            // osgDB::readNodeFile("server:http://myserver/myearth.earth").  This should only be
            // necessary for the first level as the other files will have a tilekey prepended to them.
            if ((fileName.length() > 7) && (fileName.substr(0, 7) == "server:"))
                return readNode(fileName.substr(7), options);            

            if ( fileName == "__globe.earth" )
            {
                return ReadResult( new MapNode() );
            }

            else if ( fileName == "__cube.earth" )
            {
                MapOptions options;
                options.coordSysType() = MapOptions::CSTYPE_GEOCENTRIC_CUBE;
                return ReadResult( new MapNode( new Map(options) ) );
            }

            else
            {
                osgEarth::ReadResult r = URI(fileName).readString( options, CachePolicy::NO_CACHE );
                if ( r.failed() )
                    return ReadResult::ERROR_IN_READING_FILE;

                // Since we're now passing off control to the stream, we have to pass along the
                // reference URI as well..
                osg::ref_ptr<Options> myOptions = options ? 
                    static_cast<Options*>(options->clone(osg::CopyOp::DEEP_COPY_ALL)) : 
                    new Options();

                URIContext( fileName ).store( myOptions.get() );
                //myOptions->setPluginData( "__ReaderWriterOsgEarth::ref_uri", (void*)&fileName );

                std::stringstream in( r.getString() );
                return readNode( in, myOptions.get() );
            }
        }

        virtual ReadResult readNode(std::istream& in, const Options* options ) const
        {
            // pull the URI context from the options structure (since we're reading
            // from an "anonymous" stream here)
            URIContext uriContext( options ); 
            //if ( uriContext.empty() && options && options->getDatabasePathList().size() > 0 )
            //    uriContext = URIContext( options->getDatabasePathList().front() + "/" );

            osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in, uriContext );            
            if ( !doc.valid() )
                return ReadResult::ERROR_IN_READING_FILE;

            Config docConf = doc->getConfig();

            // support both "map" and "earth" tag names at the top level
            Config conf;
            if ( docConf.hasChild( "map" ) )
                conf = docConf.child( "map" );
            else if ( docConf.hasChild( "earth" ) )
                conf = docConf.child( "earth" );

            MapNode* mapNode =0L;
            if ( !conf.empty() )
            {
                // see if we were given a reference URI to use:
                std::string refURI = uriContext.referrer();                                

                if ( conf.value("version") == "2" )
                {
                    OE_INFO << LC << "Detected a version 2.x earth file" << std::endl;
                    EarthFileSerializer2 ser;
                    mapNode = ser.deserialize( conf, refURI );
                }
                else
                {
                    OE_INFO << LC << "Detected a version 1.x earth file" << std::endl;
                    EarthFileSerializer1 ser;
                    mapNode = ser.deserialize( conf, refURI );
                }
            }

            return ReadResult(mapNode);
        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
