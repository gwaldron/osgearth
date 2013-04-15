/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>
#include <sstream>
#include <osgEarthUtil/Common>

using namespace osgEarth_osgearth;
using namespace osgEarth;

#define LC "[ReaderWriterEarth] "

// Macros to determine the filename for dependent libs.
#define Q2(x) #x
#define Q(x)  Q2(x)

#if (defined(_DEBUG) || defined(QT_DEBUG)) && defined(OSGEARTH_DEBUG_POSTFIX)
#   define LIBNAME_UTIL_POSTFIX Q(OSGEARTH_DEBUG_POSTFIX)
#elif defined(OSGEARTH_RELEASE_POSTFIX)
#   define LIBNAME_UTIL_POSTFIX Q(OSGEARTH_RELEASE_POSTFIX)
#else
#   define LIBNAME_UTIL_POSTFIX ""
#endif

#if defined(WIN32)
#   define LIBNAME_UTIL "osgEarthUtil"
#   define LIBNAME_UTIL_EXTENSION ".dll"
#else
#   define LIBNAME_UTIL "libosgEarthUtil"
#   if defined(__APPLE__)
#       define LIBNAME_UTIL_EXTENSION ".dylib"
#   else
#       define LIBNAME_UTIL_EXTENSION ".so"
#   endif
#endif


class ReaderWriterEarth : public osgDB::ReaderWriter
{
    public:
        ReaderWriterEarth()
        {
            // force the loading of other osgEarth libraries that might be needed to 
            // deserialize an earth file. 
            // osgEarthUtil: contains ColorFilter implementations
            OE_DEBUG << LC << "Forced load: " << LIBNAME_UTIL LIBNAME_UTIL_POSTFIX LIBNAME_UTIL_EXTENSION << std::endl;
            osgDB::Registry::instance()->loadLibrary( LIBNAME_UTIL LIBNAME_UTIL_POSTFIX LIBNAME_UTIL_EXTENSION );
        }

        virtual const char* className()
        {
            return "OSG Earth ReaderWriter";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "earth" );
        }

        virtual ReadResult readObject(const std::string& file_name, const osgDB::Options* options) const
        {
            return readNode( file_name, options );
        }

        virtual ReadResult readObject(std::istream& in, const osgDB::Options* options) const
        {
            return readNode( in, options );
        }

        virtual WriteResult writeNode(const osg::Node& node, const std::string& fileName, const osgDB::Options* options ) const
        {
            if ( !acceptsExtension( osgDB::getFileExtension(fileName) ) )
                return WriteResult::FILE_NOT_HANDLED;

            std::ofstream out( fileName.c_str());
            if ( out.is_open() )
                return writeNode( node, out, options );

            return WriteResult::ERROR_IN_WRITING_FILE;            
        }

        virtual WriteResult writeNode(const osg::Node& node, std::ostream& out, const osgDB::Options* options ) const
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

        virtual ReadResult readNode(const std::string& fileName, const osgDB::Options* options) const
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
                osgEarth::ReadResult r = URI(fileName).readString( options );
                if ( r.failed() )
                    return ReadResult::ERROR_IN_READING_FILE;

                // Since we're now passing off control to the stream, we have to pass along the
                // reference URI as well..
                osg::ref_ptr<osgDB::Options> myOptions = Registry::instance()->cloneOrCreateOptions(options);

                URIContext( fileName ).apply( myOptions.get() );

                std::stringstream in( r.getString() );
                return readNode( in, myOptions.get() );
            }
        }

        virtual ReadResult readNode(std::istream& in, const osgDB::Options* options ) const
        {
            // pull the URI context from the options structure (since we're reading
            // from an "anonymous" stream here)
            URIContext uriContext( options ); 

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

                if ( conf.value("version") == "1" )
                {
                    OE_INFO << LC << "Detected a version 1.x earth file" << std::endl;
                    EarthFileSerializer1 ser;
                    mapNode = ser.deserialize( conf, refURI );
                }

                else
                {
                    if ( conf.value("version") != "2" )
                        OE_INFO << LC << "No valid earth file version; assuming version='2'" << std::endl;

                    EarthFileSerializer2 ser;
                    mapNode = ser.deserialize( conf, refURI );
                }
            }

            return ReadResult(mapNode);
        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
