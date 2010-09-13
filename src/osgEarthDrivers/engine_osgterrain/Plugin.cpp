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
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgEarth/EarthFile>
#include <string>

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
        return osgDB::equalCaseInsensitive( extension, "engine_osgterrain" );
    }

    virtual ReadResult readNode(const std::string& uri, const Options* options) const
    {
        std::string earthFile = osgDB::getNameLessExtension( uri );
        osgEarth::EarthFile ef;
        if ( ef.readXML( earthFile ) )
        {
            //todo: pull this from the earthfile
            OSGTerrainOptions terrainOpts;
            return ReadResult( new OSGTerrainEngineNode( ef.getMap(), terrainOpts ) );
        }
        else
        {
            return ReadResult::FILE_NOT_FOUND;
        }
    }           
};

REGISTER_OSGPLUGIN(engine_osgterrain, OSGTerrainEnginePlugin)
