/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "CoreyTerrainEngineNode"
#include <osgEarth/Utils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <string>

#define LC "[engine_corey] "

namespace osgEarth { namespace Corey
{
    /**
     * osgEarth driver for the Rex terrain engine.
     */
    class CoreyTerrainEngineDriver : public osgDB::ReaderWriter
    {
    public:
        CoreyTerrainEngineDriver()
        {
            //nop
        }

        virtual const char* className() const
        {
            return "osgEarth Corey Terrain Engine";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_engine_corey" );
        }

        virtual ReadResult readObject(const std::string& uri, const Options* options) const
        {
            if ( "osgearth_engine_corey" == osgDB::getFileExtension( uri ) )
            {
                OE_INFO << LC << "Activated!" << std::endl;
                return ReadResult( new CoreyTerrainEngineNode() );
            }
            else
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
        }    
    };

    REGISTER_OSGPLUGIN(osgearth_engine_corey, CoreyTerrainEngineDriver);

} } // namespace osgEarth::Corey
