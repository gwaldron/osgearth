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
#include <osgEarth/GeographicTileBuilder>
#include <osgEarth/MapConfig>
#include <osgEarth/PlateCarre>
#include <osgEarth/Mercator>
#include <osgEarth/Projected>
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
    private:
        typedef std::map<std::string, osg::ref_ptr<TileBuilder> > TileBuilderMap;
        TileBuilderMap tile_builders;

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

            //See if the filename starts with a . and strip it off.  This will trick OSG into passing in the filename to our plugin
            //instead of using the CURL plugin if the filename contains a URL.  So, if you want to read a URL, you can use the following format
            //osgDB::readNodeFile(".http://myserver/myearth.earth").  This should only be necessary for the first level as the other files will have
            //a tilekey prepended to them.
            if (file_name.length() > 0 && file_name[0] == '.')
            {
                return readNode(file_name.substr(1), options);
            }

            TileBuilder* tile_builder = NULL;
            osg::ref_ptr<TileKey> key;

            // try to strip off a cell key:
            unsigned int i = file_name.find_first_of( '.' );
            if ( i >= file_name.length() - 1 )
                return ReadResult::FILE_NOT_HANDLED;
            
            std::string earth_file = file_name.substr( i+1 );
            TileBuilderMap::const_iterator k = tile_builders.find( earth_file );
            
            if ( k == tile_builders.end() )
            {
                osg::ref_ptr<MapConfig> map = MapConfigReaderWriter::readXml( file_name );
                if ( map.valid() )
                {
                    //Create the TileBuilder.
                    tile_builder = TileBuilder::create( map.get(), file_name );
                    //Check to see that the TileBuilder is valid.
                    if (!tile_builder->valid()) return ReadResult::FILE_NOT_HANDLED;

                    key = tile_builder->getDataProfile().getTileKey( "" );
                    if (dynamic_cast<PlateCarreTileKey*>(key.get()))
                    {
                        osg::notify(osg::NOTICE) << "Geodetic" << std::endl;
                    }
                    else if (dynamic_cast<MercatorTileKey*>(key.get()))
                    {
                        osg::notify(osg::NOTICE) << "Mercator" << std::endl;
                    }
                    else if (dynamic_cast<ProjectedTileKey*>(key.get()))
                    {
                        osg::notify(osg::NOTICE) << "Projected" << std::endl;
                    }

                    // cache the builder
                    const_cast<ReaderWriterEarth*>(this)->tile_builders[ file_name ] = tile_builder;
                }
                else
                {
                    return ReadResult::FILE_NOT_FOUND;
                }                
            }
            else
            {
                tile_builder = k->second.get();
                key = tile_builder->getDataProfile().getTileKey( file_name.substr( 0, i ) );
            }

            osg::Node* node = tile_builder->createNode( key.get() );
            return node? ReadResult( node ) : ReadResult::FILE_NOT_FOUND;
        }
};

REGISTER_OSGPLUGIN(earth, ReaderWriterEarth)
