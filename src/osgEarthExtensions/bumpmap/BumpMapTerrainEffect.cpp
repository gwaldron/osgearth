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
#include "BumpMapTerrainEffect"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainTileNode>

#include "BumpMapShaders"

#define LC "[BumpMap] "

#define BUMP_SAMPLER   "oe_bumpmap_tex"

using namespace osgEarth;
using namespace osgEarth::BumpMap;


BumpMapTerrainEffect::BumpMapTerrainEffect(const osgDB::Options* dbOptions)
{
    _scaleUniform     = new osg::Uniform("oe_bumpmap_scale", 1.0f);
    _intensityUniform = new osg::Uniform("oe_bumpmap_intensity", 1.0f);
}

void
BumpMapTerrainEffect::setBumpMapImage(osg::Image* image)
{
    if ( image )
    {
        _bumpMapTex = new osg::Texture2D(image);
        _bumpMapTex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
        _bumpMapTex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
        _bumpMapTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        _bumpMapTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        _bumpMapTex->setMaxAnisotropy(1.0f);
        _bumpMapTex->setUnRefImageDataAfterApply(true);
        _bumpMapTex->setResizeNonPowerOfTwoHint(false);
    }
    else
    {
        OE_WARN << LC << "Failed to load the bump map texture\n";
    }
}

void
BumpMapTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine && _bumpMapTex.valid() )
    {
        osg::StateSet* stateset = engine->getOrCreateStateSet();

        // install the NormalMap texture array:
        if ( engine->getTextureCompositor()->reserveTextureImageUnit(_bumpMapUnit) )
        {
            // NormalMap sampler
            _bumpMapTexUniform = stateset->getOrCreateUniform(BUMP_SAMPLER, osg::Uniform::SAMPLER_2D);
            _bumpMapTexUniform->set( _bumpMapUnit );
            stateset->setTextureAttribute( _bumpMapUnit, _bumpMapTex.get(), osg::StateAttribute::ON );

            // configure shaders
            std::string vertShader = ShaderLoader::loadSource(
                Shaders::VertexShaderFile, Shaders::VertexShaderSource);

            std::string fragShader = ShaderLoader::loadSource(
                Shaders::FragmentShaderFile, Shaders::FragmentShaderSource);

            // shader components
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
            vp->setFunction( "oe_bumpmap_vertex",   vertShader, ShaderComp::LOCATION_VERTEX_MODEL );
            vp->setFunction( "oe_bumpmap_fragment", fragShader, ShaderComp::LOCATION_FRAGMENT_LIGHTING, -1.0f);

            stateset->addUniform( _scaleUniform.get() );
            stateset->addUniform( _intensityUniform.get() );
        }
    }
}


void
BumpMapTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getStateSet();
    if ( stateset )
    {
        if ( _bumpMapTex.valid() )
        {
            stateset->removeUniform( _scaleUniform.get() );
            stateset->removeUniform( _intensityUniform.get() );
            stateset->removeUniform( _bumpMapTexUniform.get() );
            stateset->removeTextureAttribute( _bumpMapUnit, osg::StateAttribute::TEXTURE );
        }

        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            vp->removeShader( "oe_bumpmap_vertex" );
            vp->removeShader( "oe_bumpmap_fragment" );
        }
    }
    
    if ( _bumpMapUnit >= 0 )
    {
        engine->getTextureCompositor()->releaseTextureImageUnit( _bumpMapUnit );
        _bumpMapUnit = -1;
    }
}
