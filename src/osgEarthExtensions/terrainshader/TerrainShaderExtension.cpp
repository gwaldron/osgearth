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
#include "TerrainShaderExtension"
#include <osgEarth/TerrainEffect>
#include <osgEarth/MapNode>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderLoader>

using namespace osgEarth;
using namespace osgEarth::TerrainShader;

#define LC "[TerrainShaderExtension] "

namespace
{
    class GLSLEffect : public TerrainEffect
    {
    public:
        GLSLEffect(const std::vector<TerrainShaderOptions::Code>& code,
                   const osgDB::Options*                          dbOptions ) : _code(code), _dbOptions(dbOptions)
        {
            for(unsigned i=0; i<code.size(); ++i)
            {
                std::string fn = code[i]._uri.isSet() ? code[i]._uri->full() : "$code." + i;
                _package.add( fn, code[i]._source );
            }
        }

        void onInstall(TerrainEngineNode* engine)
        {
            if ( !engine ) return;

            VirtualProgram* vp = VirtualProgram::getOrCreate(engine->getOrCreateStateSet());
            _package.loadAll( vp, _dbOptions.get() );
        }

        void onUninstall(TerrainEngineNode* engine)
        {
            if ( engine && engine->getStateSet() )
            {
                VirtualProgram* vp = VirtualProgram::get(engine->getStateSet());
                if ( vp )
                {
                    _package.unloadAll( vp, _dbOptions.get() );
                }
            }
        }

        std::vector<TerrainShaderOptions::Code> _code;
        ShaderPackage                           _package;
        osg::ref_ptr<const osgDB::Options>      _dbOptions;
    };
}


TerrainShaderExtension::TerrainShaderExtension()
{
    //nop
}

TerrainShaderExtension::TerrainShaderExtension(const TerrainShaderOptions& options) :
_options( options )
{
    //nop
}

TerrainShaderExtension::~TerrainShaderExtension()
{
    //nop
}

void
TerrainShaderExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
TerrainShaderExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }
    _effect = new GLSLEffect( _options.code(), _dbOptions.get() );
    mapNode->getTerrainEngine()->addEffect( _effect.get() );
    
    OE_INFO << LC << "Installed.\n";

    return true;
}

bool
TerrainShaderExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode && _effect.valid() )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
        _effect = 0L;
    }

    return true;
}

