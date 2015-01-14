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
#include <osgEarth/ShaderUtils>
#include <osgEarthUtil/SimplexNoise>

#include <osgDB/WriteFile>

#include "SplatShaders"

#define LC "[Splat] "

#define COVERAGE_SAMPLER "oe_splat_coverage_tex"
#define SPLAT_SAMPLER    "oe_splat_tex"
#define NOISE_SAMPLER    "oe_splat_noise_tex"

using namespace osgEarth;
using namespace osgEarth::Splat;

SplatTerrainEffect::SplatTerrainEffect(const BiomeVector&    biomes,
                                       SplatCoverageLegend*  legend,
                                       const osgDB::Options* dbOptions) :
_biomes     ( biomes ),
_legend     ( legend ),
_renderOrder( -1.0f ),
_ok         ( false )
{
    if ( biomes.size() == 0 )
    {
        OE_WARN << LC << "Internal: no biomes.\n";
    }

    for(unsigned b = 0; b < biomes.size(); ++b)
    {
        const Biome& biome = biomes[b];
        SplatTextureDef def;

        if ( biome.getCatalog() )
        {
            if ( biome.getCatalog()->createSplatTextureDef(dbOptions, def) )
            {
                // install the sampling function.
                installCoverageSamplingFunction( def );
            }
            else
            {
                OE_WARN << LC << "Failed to create a texture for a catalog (" 
                    << biome.getCatalog()->name().get() << ")\n";
            }

        }
        else
        {
            OE_WARN << LC << "Biome \""
                << biome.name().get() << "\"" 
                << " has an empty catalog and will be ignored.\n";
        }

        // put it on the list either way, since the vector indicies of biomes
        // and texturedefs need to line up
        _textureDefs.push_back( def );

        if ( !_ok )
            _ok = def._texture.valid();
    }

    createUniforms();
}

#if 0
SplatTerrainEffect::SplatTerrainEffect(SplatCatalog*         catalog,
                                       SplatCoverageLegend*  legend,
                                       const osgDB::Options* dbOptions) :
_legend     ( legend ),
_ok         ( false ),
_renderOrder( -1.0f )
{
    if ( catalog )
    {
        SplatTextureDef def;
        _ok = catalog->createSplatTextureDef(dbOptions, def);
        if ( _ok )
        {
            _textureDefs.push_back( def );
        }
        else
        {
            OE_WARN << LC << "Failed to create texture array from splat catalog\n";
        }
    }

    createUniforms();
}
#endif

void
SplatTerrainEffect::createUniforms()
{
    _scaleOffsetUniform    = new osg::Uniform("oe_splat_scaleOffset",      0.0f);
    _intensityUniform      = new osg::Uniform("oe_splat_intensity",        1.0f);
    _warpUniform           = new osg::Uniform("oe_splat_warp",             0.0f);
    _blurUniform           = new osg::Uniform("oe_splat_blur",             1.0f);
    _snowMinElevUniform    = new osg::Uniform("oe_splat_snowMinElevation", 10000.0f);
    _snowPatchinessUniform = new osg::Uniform("oe_splat_snowPatchiness",   2.0f);

    _edit = (::getenv("OSGEARTH_SPLAT_EDIT") != 0L);
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

        engine->requireElevationTextures();

        // install the splat texture array:
        if ( engine->getTextureCompositor()->reserveTextureImageUnit(_splatTexUnit) )
        {
            osg::StateSet* stateset = new osg::StateSet();

            // splat sampler
            _splatTexUniform = stateset->getOrCreateUniform( SPLAT_SAMPLER, osg::Uniform::SAMPLER_2D_ARRAY );
            _splatTexUniform->set( _splatTexUnit );
            stateset->setTextureAttribute( _splatTexUnit, _textureDefs[0]._texture.get() );

            // coverage sampler
            _coverageTexUniform = stateset->getOrCreateUniform( COVERAGE_SAMPLER, osg::Uniform::SAMPLER_2D );
            _coverageTexUniform->set( _coverageLayer->shareImageUnit().get() );

            // noise sampler
            if (engine->getTextureCompositor()->reserveTextureImageUnit(_noiseTexUnit))
            {
                OE_INFO << LC << "Noise texture -> unit " << _noiseTexUnit << "\n";
                _noiseTex = createNoiseTexture();
                stateset->setTextureAttribute( _noiseTexUnit, _noiseTex.get() );
                _noiseTexUniform = stateset->getOrCreateUniform( NOISE_SAMPLER, osg::Uniform::SAMPLER_2D );
                _noiseTexUniform->set( _noiseTexUnit );
            }

            // control uniforms
            stateset->addUniform( _scaleOffsetUniform.get() );
            stateset->addUniform( _intensityUniform.get() );
            stateset->addUniform( _warpUniform.get() );
            stateset->addUniform( _blurUniform.get() );
            stateset->addUniform( _snowMinElevUniform.get() );
            stateset->addUniform( _snowPatchinessUniform.get() );

            stateset->getOrCreateUniform("oe_splat_freq", osg::Uniform::FLOAT)->set(32.0f);
            stateset->getOrCreateUniform("oe_splat_pers", osg::Uniform::FLOAT)->set(0.8f);
            stateset->getOrCreateUniform("oe_splat_lac",  osg::Uniform::FLOAT)->set(2.2f);
            stateset->getOrCreateUniform("oe_splat_octaves", osg::Uniform::FLOAT)->set(7.0f);

            stateset->getOrCreateUniform("oe_splat_blending_range", osg::Uniform::FLOAT)->set(250000.0f);
            stateset->getOrCreateUniform("oe_splat_detail_range", osg::Uniform::FLOAT)->set(1000000.0f);

            // Configure the vertex shader:
            std::string vertexShaderModel = ShaderLoader::loadSource(
                Shaders::SplatVertModelFile, Shaders::SplatVertModelSource );

            std::string vertexShaderView = ShaderLoader::loadSource(
                Shaders::SplatVertViewFile, Shaders::SplatVertViewSource );

            osgEarth::replaceIn( vertexShaderView, "$COVERAGE_TEXMAT_UNIFORM", _coverageLayer->shareTexMatUniformName().get() );
            
            // Configure the fragment shader:
            std::string fragmentShader = ShaderLoader::loadSource(
                Shaders::SplatFragFile,
                Shaders::SplatFragSource );

            if ( _edit )
                osgEarth::replaceIn( fragmentShader, "$SPLAT_EDIT", "#define SPLAT_EDIT 1\n" );
            else
                osgEarth::replaceIn( fragmentShader, "$SPLAT_EDIT", "" );

            // shader components
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
            vp->setFunction( "oe_splat_vertex_model", vertexShaderModel, ShaderComp::LOCATION_VERTEX_MODEL );
            vp->setFunction( "oe_splat_vertex_view",  vertexShaderView,  ShaderComp::LOCATION_VERTEX_VIEW );
            vp->setFunction( "oe_splat_fragment",     fragmentShader,    ShaderComp::LOCATION_FRAGMENT_COLORING, _renderOrder );

            // support shaders
            std::string noiseShaderSource = ShaderLoader::loadSource( Shaders::NoiseFile, Shaders::NoiseSource );
            osg::Shader* noiseShader = new osg::Shader(osg::Shader::FRAGMENT, noiseShaderSource);
            vp->setShader( "oe_splat_noiseshaders", noiseShader );

            _biomeSelector = new BiomeSelector(
                _biomes,
                _textureDefs,
                stateset,
                _splatTexUnit );

            engine->addCullCallback( _biomeSelector.get() );
        }
    }
}


void
SplatTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        if ( _noiseTexUnit >= 0 )
        {
            engine->getTextureCompositor()->releaseTextureImageUnit( _noiseTexUnit );
            _noiseTexUnit = -1;
        }
    
        if ( _splatTexUnit >= 0 )
        {
            engine->getTextureCompositor()->releaseTextureImageUnit( _splatTexUnit );
            _splatTexUnit = -1;
        }

        if ( _biomeSelector.valid() )
        {
            engine->removeCullCallback( _biomeSelector.get() );
            _biomeSelector = 0L;
        }
    }
}

#define IND "    "

void
SplatTerrainEffect::installCoverageSamplingFunction(SplatTextureDef& textureDef)
{
    if ( !textureDef._texture.valid() )
    {
        OE_WARN << LC << "Internal: texture is not set; cannot create a sampling function\n";
        return;
    }

    std::stringstream
        weightBuf,
        primaryBuf,
        detailBuf,
        saturationBuf,
        thresholdBuf,
        slopeBuf;

    unsigned
        primaryCount    = 0,
        detailCount     = 0,
        saturationCount = 0,
        thresholdCount  = 0,
        slopeCount      = 0;

    const SplatCoverageLegend::Predicates& preds = _legend->getPredicates();
    for(SplatCoverageLegend::Predicates::const_iterator p = preds.begin(); p != preds.end(); ++p)
    {
        const CoverageValuePredicate* pred = p->get();

        if ( pred->_exactValue.isSet() )
        {
            // Look up by class name:
            const std::string& className = pred->_mappedClassName.get();
            const SplatLUT::const_iterator i = textureDef._splatLUT.find(className);
            if ( i != textureDef._splatLUT.end() )
            {
                // found it; loop over the range selectors:
                int selectorCount = 0;
                const SplatSelectorVector& selectors = i->second;

                OE_DEBUG << LC << "Class " << className << " has " << selectors.size() << " selectors.\n";

                for(SplatSelectorVector::const_iterator selector = selectors.begin();
                    selector != selectors.end();
                    ++selector)
                {
                    const std::string&    expression = selector->first;
                    const SplatRangeData& rangeData  = selector->second;

                    std::string val = pred->_exactValue.get();

                    weightBuf
                        << IND "float w" << val
                        << " = (1.0-clamp(abs(value-" << val << ".0),0.0,1.0));\n";

                    // Primary texture index:
                    if ( primaryCount == 0 )
                        primaryBuf << IND "primary += ";
                    else
                        primaryBuf << " + ";

                    // the "+1" is because "primary" starts out at -1.
                    primaryBuf << "w"<<val << "*" << (rangeData._textureIndex + 1) << ".0";
                    primaryCount++;

                    // Detail texture index:
                    if ( rangeData._detail.isSet() )
                    {
                        if ( detailCount == 0 )
                            detailBuf << IND "detail += ";
                        else
                            detailBuf << " + ";
                        // the "+1" is because "detail" starts out at -1.
                        detailBuf << "w"<<val << "*" << (rangeData._detail->_textureIndex + 1) << ".0";
                        detailCount++;

                        if ( rangeData._detail->_saturation.isSet() )
                        {
                            if ( saturationCount == 0 )
                                saturationBuf << IND "saturation += ";
                            else
                                saturationBuf << " + ";
                            saturationBuf << "w"<<val << "*" << rangeData._detail->_saturation.get();
                            saturationCount++;
                        }

                        if ( rangeData._detail->_threshold.isSet() )
                        {
                            if ( thresholdCount == 0 )
                                thresholdBuf << IND "threshold += ";
                            else
                                thresholdBuf << " + ";
                            thresholdBuf << "w"<<val << "*" << rangeData._detail->_threshold.get();
                            thresholdCount++;
                        }

                        if ( rangeData._detail->_slope.isSet() )
                        {
                            if ( slopeCount == 0 )
                                slopeBuf << IND "minSlope += ";
                            else
                                slopeBuf << " + ";
                            slopeBuf << "w"<<val << "*" << rangeData._detail->_slope.get();
                            slopeCount++;
                        }
                    }                    
                }
            }
        }
    }

    if ( primaryCount > 0 )
        primaryBuf << ";\n";

    if ( detailCount > 0 )
        detailBuf << ";\n";

    if ( saturationCount > 0 )
        saturationBuf << ";\n";

    if ( thresholdCount > 0 )
        thresholdBuf << ";\n";

    if ( slopeCount > 0 )
        slopeBuf << ";\n";

    std::string code = ShaderLoader::loadSource(
        Shaders::SplatGetRenderInfoFile, 
        Shaders::SplatGetRenderInfoSource);

    std::string codeToInject = Stringify()
        << weightBuf.str()
        << primaryBuf.str()
        << detailBuf.str()
        << saturationBuf.str()
        << thresholdBuf.str()
        << slopeBuf.str();

    osgEarth::replaceIn(code, "$CODE_INJECTION_POINT", codeToInject);

    textureDef._samplingFunction = code;

    OE_INFO << LC << "Sampling function = \n" << code << "\n\n";
}

osg::Texture*
SplatTerrainEffect::createNoiseTexture() const
{
    // FUTURE PLANS:
    // 1. Use an GL_RGBA texture to store 4 different noise maps in one texture,
    //    likely with different fractal parameters
    // 2. Move some of this logic into a "procedural" sublibrary

    const int size = 1024;
    const int slices = 1;

    GLenum type = slices==1?GL_LUMINANCE : slices==3?GL_RGB : GL_RGBA;
    
    osg::Image* image = new osg::Image();
    image->allocateImage(size, size, 1, type, GL_UNSIGNED_BYTE);

    const float F[4] = { 1.0f, 2.0f, 4.0f, 8.0f };
    const float P[4] = { 0.8f, 0.6f, 0.8f, 0.9f };
    const float L[4] = { 2.2f, 2.0f, 3.0f, 4.0f };
    
    for(int k=0; k<slices; ++k)
    {
        // Configure the noise function:
        osgEarth::Util::SimplexNoise noise;
        noise.setNormalize( true );
        noise.setRange( 0.0, 1.0 );
        noise.setFrequency( F[k] );
        noise.setPersistence( P[k] );
        noise.setLacunarity( L[k] );
        noise.setOctaves( 24 );

        float nmin = 10.0f;
        float nmax = -10.0f;

        // write repeating noise to the image:
        ImageUtils::PixelReader read ( image );
        ImageUtils::PixelWriter write( image );
        for(int t=0; t<size; ++t)
        {
            double rt = (double)t/size;
            for(int s=0; s<size; ++s)
            {
                double rs = (double)s/(double)size;

                double n = noise.getTiledValue(rs, rt);

                n = osg::clampBetween(n, 0.0, 1.0);

                if ( n < nmin ) nmin = n;
                if ( n > nmax ) nmax = n;
                osg::Vec4f v = read(s, t);
                v[k] = n;
                write(v, s, t);
            }
        }
   
        // histogram stretch to [0..1]
        for(int x=0; x<size*size; ++x)
        {
            int s = x%size, t = x/size;
            osg::Vec4f v = read(s, t);
            v[k] = osg::clampBetween((v[k]-nmin)/(nmax-nmin), 0.0f, 1.0f);
            write(v, s, t);
        }

        OE_INFO << LC << "Noise: MIN = " << nmin << "; MAX = " << nmax << "\n";
    }


    std::string filename("noise.png");
    osgDB::writeImageFile(*image, filename);
    OE_NOTICE << LC << "Wrote noise texture to " << filename << "\n";

    // make a texture:
    osg::Texture2D* tex = new osg::Texture2D( image );
    tex->setWrap(tex->WRAP_S, tex->REPEAT);
    tex->setWrap(tex->WRAP_T, tex->REPEAT);
    tex->setFilter(tex->MIN_FILTER, tex->LINEAR_MIPMAP_LINEAR);
    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
    tex->setMaxAnisotropy( 1.0f );
    tex->setUnRefImageDataAfterApply( true );

    return tex;
}
