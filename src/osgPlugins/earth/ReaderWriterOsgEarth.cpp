/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/ProjectedTileBuilder>
#include <osgEarth/MapConfig>
#include <osgEarth/Mercator>
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
                //osg::notify(osg::NOTICE) << "Reading Earth File " << std::endl;

                //Read the map file from the filename
                osg::ref_ptr<MapConfig> map = MapConfigReaderWriter::readXml( file_name );

                if ( map.valid() )
                {
                    //Create the TileBuilder.
                    TileBuilder* tile_builder = TileBuilder::create( map.get());

                    //Check to see that the TileBuilder is valid.
                    if (!tile_builder->isValid())
                        return ReadResult::FILE_NOT_HANDLED;

                    if (tile_builder->getDataProfile().getProfileType() == TileGridProfile::GLOBAL_GEODETIC)
                    {
                        osg::notify(osg::INFO) << "Geodetic" << std::endl;
                    }
                    else if (tile_builder->getDataProfile().getProfileType() == TileGridProfile::GLOBAL_MERCATOR)
                    {
                        osg::notify(osg::INFO) << "Mercator" << std::endl;
                    }
                    else if (tile_builder->getDataProfile().getProfileType() == TileGridProfile::PROJECTED)
                    {
                        osg::notify(osg::INFO) << "Projected" << std::endl;
                    }

                    //Create the root node for the scene
                    node = tile_builder->createRootNode();

                    //Register the TileBuilder with osgEarth
                    if (tile_builder && node)
                    {
                        TileBuilder::registerTileBuilder(tile_builder, node);
                    }
                }
                else
                {
                    return ReadResult::FILE_NOT_FOUND;
                }                
            }
            //Reading a specific tile from an existing TileBuilder
            else if (ext == "earth_tile")
            {
                std::string tileDef = osgDB::getNameLessExtension(file_name);
                //osg::notify(osg::NOTICE) << "Reading Tile " << tileDef << std::endl;

                //The tile definition is formatted LOD_X_Y.TILEBUILDER_ID

                unsigned int lod, x, y, tileBuilderId;
                sscanf(tileDef.c_str(), "%d_%d_%d.%d", &lod, &x, &y,&tileBuilderId);

                //Get the TileBuilder from the cache
                TileBuilder* tile_builder = TileBuilder::getTileBuilderById(tileBuilderId);

                if (tile_builder)
                {
                  osg::ref_ptr<TileKey> key = new TileKey(x, y, lod, tile_builder->getDataProfile());
                  node = tile_builder->createNode(key.get());
                }
                else
                {
                    osg::notify(osg::NOTICE) << "Error:  Could not find TileBuilder with id=" << tileBuilderId << std::endl;
                }
            }

            return node ? ReadResult(node) : ReadResult::FILE_NOT_FOUND;                     
        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
