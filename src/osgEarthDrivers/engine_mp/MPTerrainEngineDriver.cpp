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
#include "MPTerrainEngineNode"
#include "MPTerrainEngineOptions"
#include "TileNode"
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>

#define LC "[engine_mp driver] "


using namespace osgEarth::Drivers;
using namespace osgEarth_engine_mp;

/**
 * osgEarth driver for the MP terrain engine.
 */
class osgEarth_MPTerrainEngineDriver : public osgDB::ReaderWriter
{
public:
    osgEarth_MPTerrainEngineDriver() { }

    virtual const char* className()
    {
        return "osgEarth MP Terrain Engine";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return
            osgDB::equalCaseInsensitive( extension, "osgearth_engine_mp" ) ||
            osgDB::equalCaseInsensitive( extension, "osgearth_engine_mp_tile" ) ||
            osgDB::equalCaseInsensitive( extension, "osgearth_engine_mp_standalone_tile" );
    }

    virtual ReadResult readObject(const std::string& uri, const Options* options) const
    {
        if ( "osgearth_engine_mp" == osgDB::getFileExtension( uri ) )
        {
            if ( "earth" == osgDB::getNameLessExtension( osgDB::getFileExtension( uri ) ) )
            {
                return readNode( uri, options );
            }
            else
            {
                MPTerrainEngineOptions terrainOpts;
                OE_INFO << LC << "Activated!" << std::endl;
                return ReadResult( new MPTerrainEngineNode() );
            }
        }
        else
        {
            return readNode( uri, options );
        }
    }    

    virtual ReadResult readNode(const std::string& uri, const Options* options) const
    {
        std::string ext = osgDB::getFileExtension(uri);
        if ( acceptsExtension(ext) )
        {
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
            osg::ref_ptr<MPTerrainEngineNode> engineNode;
            MPTerrainEngineNode::getEngineByUID( (UID)engineID, engineNode );
            if ( engineNode.valid() )
            {
                osg::Timer_t start = osg::Timer::instance()->tick();

                // see if we have a progress tracker
                ProgressCallback* progress = 
                    options ? const_cast<ProgressCallback*>(
                    dynamic_cast<const ProgressCallback*>(options->getUserData())) : 0L;

                // assemble the key and create the node:
                const Profile* profile = engineNode->getMap()->getProfile();
                TileKey key( lod, x, y, profile );

                osg::ref_ptr<osg::Node> node;

                if ( "osgearth_engine_mp_tile" == ext )
                {
                    node = engineNode->createNode(key, progress);
                }
                else if ( "osgearth_engine_mp_standalone_tile" == ext )
                {
                    node = engineNode->createStandaloneNode(key, progress);
                }


#if 0
                osg::Timer_t end = osg::Timer::instance()->tick();

                //if ( osgEarth::getNotifyLevel() >= osg::DEBUG_INFO )
                {
                    static Threading::Mutex s_statsMutex;
                    static std::vector<double> s_times;
                    Threading::ScopedMutexLock lock(s_statsMutex);
                    s_times.push_back( osg::Timer::instance()->delta_s(start, end) );
                    if ( s_times.size() % 50 == 0 )
                    {
                        double t = 0.0;
                        for(unsigned i=0; i<s_times.size(); ++i)
                            t += s_times[i];
                        OE_NOTICE << LC << "Average time = " << (t/s_times.size()) << " s." << std::endl;
                    }
                }
#endif

                
                // Deal with failed loads.
                if ( !node.valid() )
                {
                    if ( key.getLOD() == 0 || (progress && progress->isCanceled()) )
                    {
                        // the tile will ask again next time.
                        return ReadResult::FILE_NOT_FOUND;
                    }
                    else
                    {
                        // the parent tile will never ask again as long as it remains in memory.
                        node = new InvalidTileNode( key );
                    }
                }
                else
                {   
                    // notify the Terrain interface of a new tile
                    osg::Timer_t start = osg::Timer::instance()->tick();
                    engineNode->getTerrain()->notifyTileAdded(key, node.get());
                    osg::Timer_t end = osg::Timer::instance()->tick();
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

REGISTER_OSGPLUGIN(osgearth_engine_mp, osgEarth_MPTerrainEngineDriver)
