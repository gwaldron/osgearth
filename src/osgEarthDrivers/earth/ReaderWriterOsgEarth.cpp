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

#include <osgEarth/MapNode>
#include <osgEarth/Map>
#include <osgEarth/EarthFile>
#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>
#include <stdlib.h>
#include <algorithm>
#include <map>

using namespace osgEarth;

#define TO_LOWER( S ) std::transform( S.begin(), S.end(), S.begin(), ::tolower )

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
            return (osgDB::equalCaseInsensitive( extension, "earth" ) ||
                    osgDB::equalCaseInsensitive( extension, "earth_tile" ));
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

            //See if the filename starts with server: and strip it off.  This will trick OSG into passing in the filename to our plugin
            //instead of using the CURL plugin if the filename contains a URL.  So, if you want to read a URL, you can use the following format
            //osgDB::readNodeFile("server:http://myserver/myearth.earth").  This should only be necessary for the first level as the other files will have
            //a tilekey prepended to them.
            if ((file_name.length() > 7) && (file_name.substr(0, 7) == "server:"))
            {
                return readNode(file_name.substr(7), options);
            }

            osg::Node* node = 0;

            //Reading a earth file defintion
            if (ext == "earth")
            {
                EarthFile earthFile;

                bool success = true;
                
                if ( file_name == "__globe.earth" )
                {
                    earthFile.setMap( new Map(Map::CSTYPE_GEOCENTRIC) );
                }
                else if ( file_name == "__flat.earth" )
                {
                    earthFile.setMap( new Map(Map::CSTYPE_PROJECTED) );
                }
                else if ( file_name == "__cube.earth" )
                {
                    earthFile.setMap( new Map(Map::CSTYPE_GEOCENTRIC_CUBE) );
                }
                else
                {
                    success = earthFile.readXML( file_name );
                }

                if ( success )
                {
                    MapEngineProperties props = earthFile.getMapEngineProperties();

                    if ( options )
                    {
                        // check is an engine properties object was supplied in the Options structure. If so,
                        // merge it in. Note that the properties will override those in the earth file.

                        const MapEngineProperties* userProps = static_cast<const MapEngineProperties*>(
                            options->getPluginData( MapEngineProperties::OPTIONS_TAG ) );

                        if ( userProps )
                        {
                            props.merge( *userProps );
                        }
                    }

                    osg::ref_ptr<MapNode> mapNode = new MapNode( earthFile.getMap(), props );

                    //Create the root node for the scene
                    node = mapNode.release();
                }
                else
                {
                    return ReadResult::FILE_NOT_FOUND;
                }                
            }

            // Reading a specific tile from an existing TileBuilder
            else if (ext == "earth_tile")
            {
                std::string tileDef = osgDB::getNameLessExtension(file_name);
                //OE_NOTICE << "Reading Tile " << tileDef << std::endl;

                //The tile definition is formatted FACE_LOD_X_Y.MAPENGINE_ID

                unsigned int lod, x, y, id;
                sscanf(tileDef.c_str(), "%d_%d_%d.%d", &lod, &x, &y, &id);

                //Get the Map from the cache.  It is important that we use a ref_ptr here
                //to prevent the Map from being deleted while it is is still in use.
                //osg::ref_ptr<MapEngine> engine = MapEngine::getMapEngineById( id );
                osg::ref_ptr<MapNode> mapNode = MapNode::getMapNodeById( id );

                if ( mapNode.valid() )
                {
                    const Profile* profile = mapNode->getMap()->getProfile();
                    osg::ref_ptr<TileKey> key = new TileKey( lod, x, y, profile );

                    if ( ext == "earth_tile" )
                    {
                        bool populateLayers = mapNode->getEngine()->getEngineProperties().loadingPolicy()->mode() 
                            == LoadingPolicy::MODE_STANDARD;

                        node = mapNode->getEngine()->createSubTiles(
                            mapNode->getMap(),
                            mapNode->getTerrain(),
                            key.get(),
                            populateLayers );

                        //Blacklist the tile if we couldn't load it
                        if (!node)
                        {
                            OE_DEBUG << "Blacklisting " << file_name << std::endl;
                            osgEarth::Registry::instance()->blacklist(file_name);
                        }

                    }
                }
                else
                {
                    OE_NOTICE << "Error:  Could not find Map with id=" << id << std::endl;
                }
            }


            return node ? ReadResult(node) : ReadResult::FILE_NOT_FOUND;                     
        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
