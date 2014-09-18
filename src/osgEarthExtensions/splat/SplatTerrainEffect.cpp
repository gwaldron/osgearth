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
#include "SplatTerrainEffect"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ImageUtils>
#include <osgEarth/URI>

#include "NoiseShaders"
#include "SplatShaders"

#define LC "[Splat] "

#define COVERAGE_SAMPLER "oe_coverage_tex"
#define SPLAT_SAMPLER    "oe_splat_tex"
#define SPLAT_FUNC       "oe_splat_getTexel"

using namespace osgEarth;
using namespace osgEarth::Extensions::Splat;

SplatTerrainEffect::SplatTerrainEffect(SplatCatalog*         catalog,
                                       SplatCoverageLegend*  legend,
                                       const osgDB::Options* dbOptions) :
_legend( legend ),
_ok    ( false )
{
    if ( catalog )
    {
        _ok = catalog->createTextureAndIndex( dbOptions, _splatTex, _splatTexIndex );
        if ( !_ok )
        {
            OE_WARN << LC << "Failed to create texture array from splat catalog\n";
        }
    }

    _startLODUniform  = new osg::Uniform("oe_splat_L0",        1.0f);
    _scaleUniform     = new osg::Uniform("oe_splat_scale",     8.0f);
    _intensityUniform = new osg::Uniform("oe_splat_intensity", 1.0f);
    _warpUniform      = new osg::Uniform("oe_splat_warp",    0.004f);
    _samplesUniform   = new osg::Uniform("oe_splat_samples",   1.0f);
}

void
SplatTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine && _ok )
    {
        if ( !_coverageLayer.valid() )
        {
            OE_WARN << LC << "No coverage layer set\n";
            return;
        }

        osg::StateSet* stateset = engine->getOrCreateStateSet();

        // install the splat texture array:
        if ( engine->getTextureCompositor()->reserveTextureImageUnit(_splatTexUnit) )
        {
            // splat sampler
            _splatTexUniform = stateset->getOrCreateUniform( SPLAT_SAMPLER, osg::Uniform::SAMPLER_2D_ARRAY );
            _splatTexUniform->set( _splatTexUnit );
            stateset->setTextureAttribute( _splatTexUnit, _splatTex.get(), osg::StateAttribute::ON );

            // coverage sampler
            _coverageTexUniform = stateset->getOrCreateUniform( COVERAGE_SAMPLER, osg::Uniform::SAMPLER_2D );
            _coverageTexUniform->set( _coverageLayer->shareImageUnit().get() );

            // control uniforms
            stateset->addUniform( _startLODUniform.get() );
            stateset->addUniform( _scaleUniform.get() );
            stateset->addUniform( _intensityUniform.get() );
            stateset->addUniform( _warpUniform.get() );
            stateset->addUniform( _samplesUniform.get() );

            // configure shaders
            std::string vertexShader = splatVertexShader;
            osgEarth::replaceIn( vertexShader, "$COVERAGE_TEXMAT_UNIFORM", _coverageLayer->shareTexMatUniformName().get() );

            // shader components
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
            vp->setFunction( "oe_splat_vertex",   vertexShader,        ShaderComp::LOCATION_VERTEX_VIEW );
            vp->setFunction( "oe_splat_fragment", splatFragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING, -1.0 );

            // support shaders
            osg::Shader* noiseShader = new osg::Shader(osg::Shader::FRAGMENT, noise4Dshaders);
            vp->setShader( NOISE_FUNC, noiseShader );

            // sampling function
            std::string sfunc = generateSamplingFunction();
            osg::Shader* sfuncShader = new osg::Shader(osg::Shader::FRAGMENT, sfunc);
            vp->setShader( SPLAT_FUNC, sfuncShader );
        }
    }
}


void
SplatTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    osg::StateSet* stateset = engine->getStateSet();
    if ( stateset )
    {
        if ( _splatTexUniform.valid() )
        {
            stateset->removeUniform( _startLODUniform.get() );
            stateset->removeUniform( _scaleUniform.get() );
            stateset->removeUniform( _warpUniform.get() );
            stateset->removeUniform( _samplesUniform.get() );
            stateset->removeUniform( _intensityUniform.get() );
            stateset->removeUniform( _splatTexUniform.get() );
            stateset->removeUniform( _coverageTexUniform.get() );
            stateset->removeTextureAttribute( _splatTexUnit, osg::StateAttribute::TEXTURE );
        }

        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( vp )
        {
            vp->removeShader( "oe_splat_vertex" );
            vp->removeShader( "oe_splat_fragment" );
            vp->removeShader( SPLAT_FUNC );
            vp->removeShader( NOISE_FUNC );
        }
    }
    
    if ( _splatTexUnit >= 0 )
    {
        engine->getTextureCompositor()->releaseTextureImageUnit( _splatTexUnit );
        _splatTexUnit = -1;
    }
}


std::string
SplatTerrainEffect::generateSamplingFunction()
{
    std::stringstream buf;
    buf <<
        "#version " GLSL_VERSION_STR "\n"
        "#extension GL_EXT_texture_array : enable\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform sampler2DArray " << SPLAT_SAMPLER << ";\n"
        "float " NOISE_FUNC "(in vec2);\n"
        "vec4 " SPLAT_FUNC "(in float v, in vec2 splat_tc) \n"
        "{\n"
        "    float i = -1.0;\n";

    unsigned count = 0;
    const SplatCoverageLegend::Predicates& preds = _legend->getPredicates();
    for(SplatCoverageLegend::Predicates::const_iterator p = preds.begin(); p != preds.end(); ++p, ++count)
    {
        if ( p->get()->_exactValue.isSet() )
        {
            if ( count > 0 )
                buf << "    else ";
            else
                buf << "    ";

            //buf << "if (v <= float(" << p->get()->_exactValue.get() << ")) i="
            buf << "if (abs(v-float(" << p->get()->_exactValue.get() << "))<0.01) i = "
                << "float(" << _splatTexIndex[p->get()->_mappedClassName.get()] << ");\n";
        }
    }

    buf << "    vec4 texel = texture2DArray(" SPLAT_SAMPLER ", vec3(splat_tc, max(i,0.0)));\n"
        << "    if ( i < 0.0 ) texel.a = 0.0; \n" //texel = vec4(1,0,0,1); \n"
        << "    return texel; \n"
        << "}\n";

    return buf.str();
}
