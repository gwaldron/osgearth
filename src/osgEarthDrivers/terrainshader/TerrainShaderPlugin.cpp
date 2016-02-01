/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include "TerrainShaderOptions"
#include "TerrainShaderExtension"

#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace TerrainShader
{
    /**
     * Plugin entry point
     */
    class TerrainShaderPlugin : public osgDB::ReaderWriter
    {
    public: // Plugin stuff

        TerrainShaderPlugin() {
            supportsExtension( "osgearth_terrainshader", "osgEarth Terrain Shader Extension Plugin" );
        }
        
        const char* className() {
            return "osgEarth Terrain Shader Extension Plugin";
        }

        virtual ~TerrainShaderPlugin() { }

        ReadResult readObject(const std::string& filename, const osgDB::Options* dbOptions) const
        {
          if ( !acceptsExtension(osgDB::getLowerCaseFileExtension(filename)) )
                return ReadResult::FILE_NOT_HANDLED;

          return ReadResult( new TerrainShaderExtension(Extension::getConfigOptions(dbOptions)) );
        }
    };

    REGISTER_OSGPLUGIN(osgearth_terrainshader, TerrainShaderPlugin)

} } // namespace osgEarth::BumpMap
