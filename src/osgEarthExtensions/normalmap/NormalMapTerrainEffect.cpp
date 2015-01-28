/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "NormalMapTerrainEffect"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainTileNode>

#include "NormalMapShaders"

#define LC "[NormalMap] "

#define NORMAL_SAMPLER "oe_nmap_normalTex"
#define NORMAL_MATRIX  "oe_nmap_normalTexMatrix"

using namespace osgEarth;
using namespace osgEarth::NormalMap;

namespace
{
    class NormalTexInstaller : public TerrainTileNodeCallback
    {
    public:
        NormalTexInstaller(NormalMapTerrainEffect* effect, int unit)
            : _effect(effect), _unit(unit) { }
        
    public: // TileNodeCallback
        void operator()(const TileKey& key, osg::Node* node)
        {
            TerrainTileNode* tile = osgEarth::findTopMostNodeOfType<TerrainTileNode>(node);
            if ( !tile )
                return;

            osg::Texture* tex = tile->getNormalTexture();
            if ( tex )
            {
                osg::StateSet* ss = node->getOrCreateStateSet();
                ss->setTextureAttribute(_unit, tex);
                
                osg::RefMatrixf* mat = tile->getNormalTextureMatrix();
                osg::Matrixf m = mat? *mat : osg::Matrixf::identity();

                ss->addUniform(new osg::Uniform(NORMAL_MATRIX, m) );
            }
            else
            {
                OE_WARN << LC << "trouble.\n";
            }
        }

    private:
        osg::observer_ptr<NormalMapTerrainEffect> _effect;
        int _unit;
    };
}


NormalMapTerrainEffect::NormalMapTerrainEffect(const osgDB::Options* dbOptions) :
_normalMapUnit( -1 )
{
    //nop
}

void
NormalMapTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        engine->requireNormalTextures();

        engine->getResources()->reserveTextureImageUnit(_normalMapUnit);
        OE_INFO << LC << "Normal unit = " << _normalMapUnit << "\n";
        engine->addTileNodeCallback( new NormalTexInstaller(this, _normalMapUnit) );

        // configure shaders
        std::string vertShader = ShaderLoader::loadSource(
            Shaders::VertexShaderFile, Shaders::VertexShaderSource);

        std::string fragShader = ShaderLoader::loadSource(
            Shaders::FragmentShaderFile, Shaders::FragmentShaderSource);

        // shader components
        osg::StateSet* stateset = engine->getTerrainStateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setFunction( "oe_nmap_vertex",   vertShader, ShaderComp::LOCATION_VERTEX_MODEL );
        vp->setFunction( "oe_nmap_fragment", fragShader, ShaderComp::LOCATION_FRAGMENT_LIGHTING, -2.0f);
        
        stateset->addUniform( new osg::Uniform(NORMAL_SAMPLER, _normalMapUnit) );
    }
}


void
NormalMapTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getStateSet();
    if ( stateset )
    {
        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            vp->removeShader( "oe_nmap_vertex" );
            vp->removeShader( "oe_nmap_fragment" );
        }
        stateset->removeUniform( NORMAL_SAMPLER );
    }
    
    if ( _normalMapUnit >= 0 )
    {
        engine->getResources()->releaseTextureImageUnit( _normalMapUnit );
        _normalMapUnit = -1;
    }
}
