/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include "BYOTerrainEngineNode"
#include "BYOTerrainEngineOptions"
#include <osgDB/FileNameUtils>

#define LC "[engine_byo driver] "

using namespace osgEarth::Drivers;
using namespace osgEarth_engine_byo;

/**
 * osgEarth driver for the "Bring Your Own" terrain engine.
 */
class osgEarth_BYOTerrainEngineDriver : public osgDB::ReaderWriter
{
public:
    osgEarth_BYOTerrainEngineDriver() {}

    virtual const char* className()
    {
        return "osgEarth BYO Terrain Engine";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return
            osgDB::equalCaseInsensitive( extension, "osgearth_engine_byo" );
    }

    virtual ReadResult readObject(const std::string& uri, const Options* options) const
    {
        if ( "osgearth_engine_byo" == osgDB::getFileExtension( uri ) )
        {
            OE_INFO << LC << "Activated!" << std::endl;
            return ReadResult( new BYOTerrainEngineNode() );
        }
        else
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
    }    

    virtual ReadResult readNode(const std::string& uri, const Options* options) const
    {
        return readObject(uri, options);
    }
};

REGISTER_OSGPLUGIN(osgearth_engine_byo, osgEarth_BYOTerrainEngineDriver)
