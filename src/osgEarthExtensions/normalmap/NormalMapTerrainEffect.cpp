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

#include "NormalMapShaders"

#define LC "[NormalMap] "

#define NORMALMAP_SAMPLER "oe_nmap_tex"

// Tile LOD offset of the "Level 0" NormalMapting scale. This is necessary
// to get rid of precision issues when scaling the NormalMaps up high.
//#define L0_OFFSET "10.0"

using namespace osgEarth;
using namespace osgEarth::NormalMap;

NormalMapTerrainEffect::NormalMapTerrainEffect(const osgDB::Options* dbOptions)
{
    _scaleUniform = new osg::Uniform("oe_nmap_scale", osg::Uniform::FLOAT);
    _scaleUniform->set( 1.0f );

    _intensityUniform = new osg::Uniform("oe_nmap_intensity", osg::Uniform::FLOAT);
    _intensityUniform->set( 1.0f );
}

void
NormalMapTerrainEffect::setNormalMapImage(osg::Image* image)
{
    if ( image )
    {
        _normalMapTex = new osg::Texture2D(image);
        _normalMapTex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        _normalMapTex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        _normalMapTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        _normalMapTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        _normalMapTex->setMaxAnisotropy(1.0f);
        _normalMapTex->setUnRefImageDataAfterApply(true);
        _normalMapTex->setResizeNonPowerOfTwoHint(false);
    }
    else
    {
        OE_WARN << LC << "Failed to load the bump map texture\n";
    }
}

void
NormalMapTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine && _normalMapTex.valid() )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();

        // install the NormalMap texture array:
        if ( engine->getTextureCompositor()->reserveTextureImageUnit(_normalMapUnit) )
        {
            // NormalMap sampler
            _normalMapTexUniform = stateset->getOrCreateUniform(NORMALMAP_SAMPLER, osg::Uniform::SAMPLER_2D);
            _normalMapTexUniform->set( _normalMapUnit );
            stateset->setTextureAttribute( _normalMapUnit, _normalMapTex.get(), osg::StateAttribute::ON );

            // configure shaders
            std::string vertShader = NormalMapVertexShader;
            std::string fragShader = NormalMapFragmentShader;

            // shader components
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
            vp->setFunction( "oe_nmap_vertex",   vertShader, ShaderComp::LOCATION_VERTEX_VIEW );
            vp->setFunction( "oe_nmap_fragment", fragShader, ShaderComp::LOCATION_FRAGMENT_LIGHTING, -1.0f);

            stateset->addUniform( _scaleUniform.get() );
            stateset->addUniform( _intensityUniform.get() );
        }
    }
}


void
NormalMapTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getStateSet();
    if ( stateset )
    {
        if ( _normalMapTex.valid() )
        {
            stateset->removeUniform( _scaleUniform.get() );
            stateset->removeUniform( _intensityUniform.get() );
            stateset->removeUniform( _normalMapTexUniform.get() );
            stateset->removeTextureAttribute( _normalMapUnit, osg::StateAttribute::TEXTURE );
        }

        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            vp->removeShader( "oe_nmap_vertex" );
            vp->removeShader( "oe_nmap_fragment" );
        }
    }
    
    if ( _normalMapUnit >= 0 )
    {
        engine->getTextureCompositor()->releaseTextureImageUnit( _normalMapUnit );
        _normalMapUnit = -1;
    }
}
