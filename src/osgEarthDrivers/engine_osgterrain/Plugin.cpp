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
#include "OSGTerrainOptions"
#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>

using namespace osgEarth::Drivers;

class OSGTerrainEnginePlugin : public osgDB::ReaderWriter
{
public:
    OSGTerrainEnginePlugin() {}

    virtual const char* className()
    {
        return "osgEarth osgTerrain Engine";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return
            osgDB::equalCaseInsensitive( extension, "osgearth_engine_osgterrain" ) ||
            osgDB::equalCaseInsensitive( extension, "osgearth_osgterrain_tile" ) ||
            osgDB::equalCaseInsensitive( extension, "test_osgearth_engine_osgterrain" );
    }

    virtual ReadResult readObject(const std::string& uri, const Options* options) const
    {
        if ( "osgearth_engine_osgterrain" == osgDB::getFileExtension( uri ) )
        {
            if ( "earth" ==  osgDB::getNameLessExtension( osgDB::getFileExtension( uri ) ) )
            {
                return readNode( uri, options );
            }
            else
            {
                OSGTerrainOptions terrainOpts;
                return ReadResult( new OSGTerrainEngineNode() );
            }
        }
        else
        {
            return readNode( uri, options );
        }
    }       

    virtual ReadResult readNode(const std::string& uri, const Options* options) const
    {
        // Handler for PagedLOD targets created by the OSGTileFactory:
        if ( "osgearth_osgterrain_tile" == osgDB::getFileExtension( uri ) )
        {            
            //See if the filename starts with server: and strip it off.  This will trick OSG into passing in the filename to our plugin
            //instead of using the CURL plugin if the filename contains a URL.  So, if you want to read a URL, you can use the following format
            //osgDB::readNodeFile("server:http://myserver/myearth.earth").  This should only be necessary for the first level as the other files will have
            //a tilekey prepended to them.
            if ((uri.length() > 7) && (uri.substr(0, 7) == "server:"))
            {
                return readNode(uri.substr(7), options);
            }

            osg::Node* node = 0;

            std::string tileDef = osgDB::getNameLessExtension(uri);

            unsigned int lod, x, y, id;
            sscanf(tileDef.c_str(), "%d_%d_%d.%d", (int*)&lod, (int*)&x, (int*)&y, (int*)&id);

            //Get the Map from the cache.  It is important that we use a ref_ptr here
            //to prevent the Map from being deleted while it is is still in use.
            //osg::ref_ptr<MapEngine> engine = MapEngine::getMapEngineById( id );
            osg::ref_ptr<OSGTerrainEngineNode> engineNode = OSGTerrainEngineNode::getEngineByUID( (UID)id );

            if ( engineNode.valid() )
            {
                const Profile* profile = engineNode->getMap()->getProfile();
                TileKey key( lod, x, y, profile );

                bool populateLayers = engineNode->getTileFactory()->getTerrainOptions().loadingPolicy()->mode() 
                    == LoadingPolicy::MODE_STANDARD;

                // create a map frame so we can safely create tiles from this dbpager thread
                MapFrame mapf( engineNode->getMap(), Map::TERRAIN_LAYERS, "dbpager::earth plugin" );

                node = engineNode->getTileFactory()->createSubTiles(
                    mapf,
                    engineNode->getTerrain(),
                    key,
                    populateLayers );

                //Blacklist the tile if we couldn't load it
                if (!node)
                {
                    OE_DEBUG << "Blacklisting " << uri << std::endl;
                    osgEarth::Registry::instance()->blacklist(uri);
                }
            }
            else
            {
                OE_NOTICE << "Error: Could not find Map with id=" << id << std::endl;
            }

            return node ? ReadResult(node) : ReadResult::FILE_NOT_FOUND;
        }

        return ReadResult::FILE_NOT_HANDLED;
    }
};

REGISTER_OSGPLUGIN(osgearth_engine_osgterrain, OSGTerrainEnginePlugin)
