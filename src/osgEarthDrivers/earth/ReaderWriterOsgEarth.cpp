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

        virtual ReadResult readNode(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            // See if the filename starts with server: and strip it off.  This will trick OSG into
            // passing in the filename to our plugin instead of using the CURL plugin if the filename
            // contains a URL.  So, if you want to read a URL, you can use the following format
            // osgDB::readNodeFile("server:http://myserver/myearth.earth").  This should only be
            // necessary for the first level as the other files will have a tilekey prepended to them.
            if ((file_name.length() > 7) && (file_name.substr(0, 7) == "server:"))
            {
                return readNode(file_name.substr(7), options);
            }

            osg::Node* node = 0;

            if ( file_name == "__globe.earth" )
            {
                return ReadResult( new MapNode() );
            }

            else if ( file_name == "__cube.earth" )
            {
                MapOptions options;
                options.coordSysType() = MapOptions::CSTYPE_GEOCENTRIC_CUBE;
                return ReadResult( new MapNode( new Map(options) ) );
            }

            else
            {
                std::string buf;
                if ( HTTPClient::readString( file_name, buf ) != HTTPClient::RESULT_OK )
                {
                    return ReadResult::ERROR_IN_READING_FILE;
                }

                std::stringstream in( buf );
                osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
                if ( doc.valid() )
                {
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
                        if ( conf.value("version") == "2" )
                        {
                            OE_INFO << LC << "Detected a version 2.x earth file" << std::endl;
                            EarthFileSerializer2 ser;
                            mapNode = ser.deserialize( conf, file_name );
                        }
                        else
                        {
                            OE_INFO << LC << "Detected a version 1.x earth file" << std::endl;
                            EarthFileSerializer1 ser;
                            mapNode = ser.deserialize( conf, file_name );
                        }
                    }
                    return ReadResult(mapNode);
                }
            }

            return ReadResult::FILE_NOT_FOUND;
        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
