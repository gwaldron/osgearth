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

#define COVERAGE_SAMPLER "oe_splat_coverage_tex"
#define SPLAT_SAMPLER    "oe_splat_tex"
#define SPLAT_FUNC       "oe_splat_getTexel"

// Tile LOD offset of the "Level 0" splatting scale. This is necessary
// to get rid of precision issues when scaling the splats up high.
//#define L0_OFFSET "10.0"

using namespace osgEarth;
using namespace osgEarth::Splat;

SplatTerrainEffect::SplatTerrainEffect(SplatCatalog*         catalog,
                                       SplatCoverageLegend*  legend,
                                       const osgDB::Options* dbOptions) :
_legend     ( legend ),
_ok         ( false ),
_renderOrder( -1.0f )
{
    if ( catalog )
    {
        _ok = catalog->createSplatTextureDef(dbOptions, _splatDef);
        if ( !_ok )
        {
            OE_WARN << LC << "Failed to create texture array from splat catalog\n";
        }
    }

    _scaleOffsetUniform = new osg::Uniform("oe_splat_scaleOffset", 0.0f);
    _intensityUniform   = new osg::Uniform("oe_splat_intensity",   1.0f);
    _warpUniform        = new osg::Uniform("oe_splat_warp",      0.004f);
    _blurUniform        = new osg::Uniform("oe_splat_blur",        1.0f);
    _snowUniform        = new osg::Uniform("oe_splat_snow",    10000.0f);
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
            stateset->setTextureAttribute( _splatTexUnit, _splatDef._texture.get(), osg::StateAttribute::ON );

            // coverage sampler
            _coverageTexUniform = stateset->getOrCreateUniform( COVERAGE_SAMPLER, osg::Uniform::SAMPLER_2D );
            _coverageTexUniform->set( _coverageLayer->shareImageUnit().get() );

            // control uniforms
            stateset->addUniform( _scaleOffsetUniform.get() );
            stateset->addUniform( _intensityUniform.get() );
            stateset->addUniform( _warpUniform.get() );
            stateset->addUniform( _blurUniform.get() );
            stateset->addUniform( _snowUniform.get() );

            // configure shaders
            std::string vertexShader = splatVertexShader;
            std::string fragmentShader = splatFragmentShader;

            osgEarth::replaceIn( vertexShader, "$COVERAGE_TEXMAT_UNIFORM", _coverageLayer->shareTexMatUniformName().get() );

            // shader components
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
            vp->setFunction( "oe_splat_vertex",   vertexShader,   ShaderComp::LOCATION_VERTEX_VIEW );
            vp->setFunction( "oe_splat_fragment", fragmentShader, ShaderComp::LOCATION_FRAGMENT_COLORING, _renderOrder );

            // support shaders
            osg::Shader* noiseShader = new osg::Shader(osg::Shader::FRAGMENT, noiseShaders);
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
            stateset->removeUniform( _scaleOffsetUniform.get() );
            stateset->removeUniform( _warpUniform.get() );
            stateset->removeUniform( _blurUniform.get() );
            stateset->removeUniform( _snowUniform.get() );
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

#define IND "    "

std::string
SplatTerrainEffect::generateSamplingFunction()
{
    std::stringstream buf;
    buf <<
        "#version " GLSL_VERSION_STR "\n"
        "#extension GL_EXT_texture_array : enable\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform sampler2DArray " << SPLAT_SAMPLER << ";\n";
    
    // This must match the def in SplatShaders
    // reminder: struct defs cannot include newlines (for GLES)
    buf <<
        "struct oe_SplatEnv { "
        " float range; "
        " float elevation; "
        "}; \n";

    buf <<
        "vec4 " SPLAT_FUNC "(in float v, in vec2 splat_tc, in oe_SplatEnv env) \n"
        "{\n"
        IND "float i = -1.0;\n";

    unsigned count = 0;
    const SplatCoverageLegend::Predicates& preds = _legend->getPredicates();
    for(SplatCoverageLegend::Predicates::const_iterator p = preds.begin(); p != preds.end(); ++p, ++count)
    {
        const CoverageValuePredicate* pred = p->get();

        if ( pred->_exactValue.isSet() )
        {
            if ( count > 0 ) buf << IND "else ";
            else             buf << IND ;

            buf << "if (abs(v-float(" << pred->_exactValue.get() << "))<0.001) { \n";
            
            const std::string& className = pred->_mappedClassName.get();
            const SplatTextureLUT::const_iterator i = _splatDef._lut.find(className);
            if ( i != _splatDef._lut.end() )
            {
                int selectorCount = 0;
                const SplatIndexSelectorSet& selectors = i->second;

                OE_DEBUG << LC << "Class " << className << " has " << selectors.size() << " selectors.\n";

                for(SplatIndexSelectorSet::const_iterator j = selectors.begin(); j != selectors.end(); ++j)
                {
                    const std::string& expression = j->first;
                    unsigned           index      = j->second;

                    if ( selectorCount > 0 ) buf << IND IND "else ";
                    else                     buf << IND IND ;

                    if ( !expression.empty() )
                    {
                        buf << "if (" << expression << ") ";
                    }

                    buf << "i = float(" << index << ");"
                        << "\n";

                    ++selectorCount;

                    // once we find an empty expression, we are finished because any
                    // subsequent selectors are unreachable.
                    if ( expression.empty() )
                        break;
                }
            }

            buf << IND "}\n";
        }
    }

    buf << IND "vec4 texel = texture2DArray(" SPLAT_SAMPLER ", vec3(splat_tc, max(i,0.0)));\n"
        << IND "if ( i < 0.0 ) texel.a = 0.0; \n"
        << IND "return texel; \n"
        << "}\n";

    return buf.str();
}
