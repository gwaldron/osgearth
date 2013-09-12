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
#include "OSGTerrainEngineNode"
#include "OSGTerrainOptions"
#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>

#define LC "[osgterrain_engine Plugin] "

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth::Drivers;

class osgEarth_OSGTerrainEnginePlugin : public osgDB::ReaderWriter
{
public:
    osgEarth_OSGTerrainEnginePlugin() {}

    virtual const char* className()
    {
        return "osgEarth osgTerrain Engine";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return
            osgDB::equalCaseInsensitive( extension, "osgearth_engine_osgterrain" ) ||
            osgDB::equalCaseInsensitive( extension, "osgearth_osgterrain_tile" );
    }

    virtual ReadResult readObject(const std::string& uri, const Options* options) const
    {
        if ( "osgearth_engine_osgterrain" == osgDB::getFileExtension( uri ) )
        {
            if ( "earth" == osgDB::getNameLessExtension( osgDB::getFileExtension( uri ) ) )
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
        static int s_tileCount = 0;
        static double s_tileTime = 0.0;
        static osg::Timer_t s_startTime;

        if ( "osgearth_osgterrain_tile" == osgDB::getFileExtension(uri) )
        {
            if ( s_tileCount == 0 )
                s_startTime = osg::Timer::instance()->tick();

            // See if the filename starts with server: and strip it off.  This will trick OSG
            // into passing in the filename to our plugin instead of using the CURL plugin if
            // the filename contains a URL.  So, if you want to read a URL, you can use the
            // following format: osgDB::readNodeFile("server:http://myserver/myearth.earth").
            // This should only be necessary for the first level as the other files will have
            // a tilekey prepended to them.
            if ((uri.length() > 7) && (uri.substr(0, 7) == "server:"))
                return readNode(uri.substr(7), options);

            // parse the tile key and engine ID:
            std::string tileDef = osgDB::getNameLessExtension(uri);
            unsigned int lod, x, y, engineID;
            sscanf(tileDef.c_str(), "%d/%d/%d.%d", &lod, &x, &y, &engineID);

            // find the appropriate engine:
            osg::ref_ptr<OSGTerrainEngineNode> engineNode;
            OSGTerrainEngineNode::getEngineByUID( (UID)engineID, engineNode );
            if ( engineNode.valid() )
            {
                osg::Timer_t start = osg::Timer::instance()->tick();

                // assemble the key and create the node:
                const Profile* profile = engineNode->getMap()->getProfile();
                TileKey key( lod, x, y, profile );
                osg::ref_ptr< osg::Node > node = engineNode->createNode( key );
                
                // Blacklist the tile if we couldn't load it
                if ( !node.valid() )
                {
                    OE_DEBUG << LC << "Blacklisting " << uri << std::endl;
                    osgEarth::Registry::instance()->blacklist( uri );
                    return ReadResult::FILE_NOT_FOUND;
                }
                else
                {   
                    // make safe ref/unref so we can reference from multiple threads
                    node->setThreadSafeRefUnref( true );

                    // notify the Terrain interface of a new tile
                    osg::Timer_t start = osg::Timer::instance()->tick();
                    engineNode->getTerrain()->notifyTileAdded(key, node.get());
                    osg::Timer_t end = osg::Timer::instance()->tick();

                    //OE_DEBUG << "Took " << osg::Timer::instance()->delta_m(start, end) << "ms to fire terrain callbacks" << std::endl;
                }

                return ReadResult( node.get(), ReadResult::FILE_LOADED );
            }
            else
            {
                return ReadResult::FILE_NOT_FOUND;
            }
        }
        else
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
    }
};

REGISTER_OSGPLUGIN(osgearth_engine_osgterrain, osgEarth_OSGTerrainEnginePlugin)
