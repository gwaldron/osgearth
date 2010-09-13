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
#include "OSGTerrainEngineNode"
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

class OSGTerrainTileLoader : public osgDB::ReaderWriter
{
public:
    OSGTerrainTileLoader() {}

    virtual const char* className()
    {
        return "osgEarth OSGTerrainTileLoader";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "earth_osgterrain_tile" );
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

        std::string tileDef = osgDB::getNameLessExtension(file_name);

        unsigned int lod, x, y, id;
        sscanf(tileDef.c_str(), "%d_%d_%d.%d", &lod, &x, &y, &id);

        //Get the Map from the cache.  It is important that we use a ref_ptr here
        //to prevent the Map from being deleted while it is is still in use.
        //osg::ref_ptr<MapEngine> engine = MapEngine::getMapEngineById( id );
        osg::ref_ptr<OSGTerrainEngineNode> engineNode = OSGTerrainEngineNode::getEngineById( id );

        if ( engineNode.valid() )
        {
            const Profile* profile = engineNode->getMap()->getProfile();
            osg::ref_ptr<TileKey> key = new TileKey( lod, x, y, profile );

            bool populateLayers = engineNode->getTileFactory()->getEngineProperties().loadingPolicy()->mode() 
                == LoadingPolicy::MODE_STANDARD;

            node = engineNode->getTileFactory()->createSubTiles(
                engineNode->getMap(),
                engineNode->getTerrain(),
                key.get(),
                populateLayers );

            //Blacklist the tile if we couldn't load it
            if (!node)
            {
                OE_DEBUG << "Blacklisting " << file_name << std::endl;
                osgEarth::Registry::instance()->blacklist(file_name);
            }
        }
        else
        {
            OE_NOTICE << "Error:  Could not find Map with id=" << id << std::endl;
        }
 
       return node ? ReadResult(node) : ReadResult::FILE_NOT_FOUND;
    }
};

REGISTER_OSGPLUGIN(earth_osgterrain_tile, OSGTerrainTileLoader)
