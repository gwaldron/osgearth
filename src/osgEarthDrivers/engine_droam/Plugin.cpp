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
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgEarth/MapNode>
#include <string>
#include "DRoamNode"

class DRoamEnginePlugin : public osgDB::ReaderWriter
{
public:
    DRoamEnginePlugin() {}

    virtual const char* className()
    {
        return "OSG Earth D-ROAM Engine";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "engine_droam" );
    }

    virtual ReadResult readNode(const std::string& uri, const Options* options) const
    {
        if ( "osgearth_engine_droam" == osgDB::getFileExtension( uri ) )
        {
            std::string earthFile = osgDB::getNameLessExtension( uri );

            osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(earthFile);
            osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode(node.get());
            if ( mapNode )
            {
                return ReadResult( new DRoamNode( mapNode->getMap() ) );
            }
            else
            {
                return ReadResult::FILE_NOT_FOUND;
            }
        }
        else
            return ReadResult::FILE_NOT_HANDLED;
    }           
};

REGISTER_OSGPLUGIN(engine_droam, DRoamEnginePlugin)
