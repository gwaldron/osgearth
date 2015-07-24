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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "SplatTerrainEffect"
#include "SplatOptions"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ImageUtils>
#include <osgEarth/URI>
#include <osgEarth/ShaderLoader>
#include <osgEarthUtil/SimplexNoise>

#include <osg/Texture2D>
#include <osgDB/WriteFile>

#include "SplatShaders"

#define LC "[Splat] "

#define COVERAGE_SAMPLER "oe_splat_coverageTex"
#define SPLAT_SAMPLER    "oe_splatTex"
#define NOISE_SAMPLER    "oe_splat_noiseTex"

using namespace osgEarth;
using namespace osgEarth::Splat;

SplatTerrainEffect::SplatTerrainEffect(const BiomeVector&    biomes,
                                       SplatCoverageLegend*  legend,
                                       const osgDB::Options* dbOptions) :
_biomes     ( biomes ),
_legend     ( legend ),
_renderOrder( -1.0f ),
_ok         ( false ),
_editMode   ( false ),
_gpuNoise   ( false )
{
    if ( biomes.size() == 0 )
    {
        OE_WARN << LC << "Internal: no biomes.\n";
    }

    // Create a texture def for each biome.
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
        {
            _ok = def._texture.valid();
        }
    }

    SplatOptions def;

    _scaleOffsetUniform    = new osg::Uniform("oe_splat_scaleOffsetInt",   *def.scaleLevelOffset());
    _warpUniform           = new osg::Uniform("oe_splat_warp",             *def.coverageWarp());
    _blurUniform           = new osg::Uniform("oe_splat_blur",             *def.coverageBlur());
    _useBilinearUniform    = new osg::Uniform("oe_splat_useBilinear",      (def.bilinearSampling()==true?1.0f:0.0f));
    _noiseScaleUniform     = new osg::Uniform("oe_splat_noiseScale",       12.0f);

    _editMode = (::getenv("OSGEARTH_SPLAT_EDIT") != 0L);
    _gpuNoise = (::getenv("OSGEARTH_SPLAT_GPU_NOISE") != 0L);
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

        // Do not need this until/unless the splatting algorithm uses elevation.
        //engine->requireElevationTextures();

        // install the splat texture array:
        if ( engine->getResources()->reserveTextureImageUnit(_splatTexUnit, "Splat Coverage") )
        {
            osg::StateSet* stateset;

#ifdef REX // note, rex doesn't support the biome selector cull callback yet b/c of the render binning
            if ( _biomes.size() == 1 )
                stateset = engine->getSurfaceStateSet();
            else
                stateset = new osg::StateSet();
#else
            stateset = new osg::StateSet();
#endif

            // TODO: reinstate "biomes"
            //osg::StateSet* stateset = new osg::StateSet();

            // splat sampler
            _splatTexUniform = stateset->getOrCreateUniform( SPLAT_SAMPLER, osg::Uniform::SAMPLER_2D_ARRAY );
            _splatTexUniform->set( _splatTexUnit );
            stateset->setTextureAttribute( _splatTexUnit, _textureDefs[0]._texture.get() );

            // coverage sampler
            _coverageTexUniform = stateset->getOrCreateUniform( COVERAGE_SAMPLER, osg::Uniform::SAMPLER_2D );
            _coverageTexUniform->set( _coverageLayer->shareImageUnit().get() );

            // control uniforms
            stateset->addUniform( _scaleOffsetUniform.get() );
            stateset->addUniform( _warpUniform.get() );
            stateset->addUniform( _blurUniform.get() );
            stateset->addUniform( _noiseScaleUniform.get() );
            stateset->addUniform( _useBilinearUniform.get() );

            stateset->addUniform(new osg::Uniform("oe_splat_detailRange",  1000000.0f));


            Shaders package;

            package.define( "SPLAT_EDIT",        _editMode );
            package.define( "SPLAT_GPU_NOISE",   _gpuNoise );
            package.define( "OE_USE_NORMAL_MAP", engine->normalTexturesRequired() );

            package.replace( "$COVERAGE_TEXMAT_UNIFORM", _coverageLayer->shareTexMatUniformName().get() );
            
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
            package.load( vp, package.VertModel );
            package.load( vp, package.VertView );
            package.load( vp, package.Frag );
            package.load( vp, package.Util );

            // GPU noise is expensive, so only use it to tweak noise function values that you
            // can later bake into the noise texture generator.
            if ( _gpuNoise )
            {                
                //osgEarth::replaceIn( fragmentShader, "#undef SPLAT_GPU_NOISE", "#define SPLAT_GPU_NOISE" );

                // Use --uniform on the command line to tweak these values:
                stateset->addUniform(new osg::Uniform("oe_splat_freq",   32.0f));
                stateset->addUniform(new osg::Uniform("oe_splat_pers",    0.8f));
                stateset->addUniform(new osg::Uniform("oe_splat_lac",     2.2f));
                stateset->addUniform(new osg::Uniform("oe_splat_octaves", 8.0f));
            }
            else // use a noise texture (the default)
            {
                if (engine->getResources()->reserveTextureImageUnit(_noiseTexUnit, "Splat Noise"))
                {
                    _noiseTex = createNoiseTexture();
                    stateset->setTextureAttribute( _noiseTexUnit, _noiseTex.get() );
                    _noiseTexUniform = stateset->getOrCreateUniform( NOISE_SAMPLER, osg::Uniform::SAMPLER_2D );
                    _noiseTexUniform->set( _noiseTexUnit );
                }
            }

            if ( _gpuNoise )
            {
                // support shaders
                std::string noiseShaderSource = ShaderLoader::load( package.Noise, package );
                osg::Shader* noiseShader = new osg::Shader(osg::Shader::FRAGMENT, noiseShaderSource);
                vp->setShader( "oe_splat_noiseshaders", noiseShader );
            }

#ifdef REX
            // TODO: I disabled BIOMES temporarily because the callback impl applies the splatting shader
            // to the land cover bin as well as the surface bin, which we do not want -- find another way!
            if ( _biomes.size() == 1 )
            {
                // install his biome's texture set:
                stateset->setTextureAttribute(_splatTexUnit, _textureDefs[0]._texture.get());

                // install this biome's sampling function. Use cloneOrCreate since each
                // stateset needs a different shader set in its VP.
                VirtualProgram* vp = VirtualProgram::cloneOrCreate( stateset );
                osg::Shader* shader = new osg::Shader(osg::Shader::FRAGMENT, _textureDefs[0]._samplingFunction);
                vp->setShader( "oe_splat_getRenderInfo", shader );
            }

            else
#endif
            {
                //OE_WARN << LC << "Multi-biome setup needs re-implementing (reminder)\n";

                // install the cull callback that will select the appropriate
                // state based on the position of the camera.
                _biomeSelector = new BiomeSelector(
                    _biomes,
                    _textureDefs,
                    stateset,
                    _splatTexUnit );

                engine->addCullCallback( _biomeSelector.get() );
            }
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
            engine->getResources()->releaseTextureImageUnit( _noiseTexUnit );
            _noiseTexUnit = -1;
        }
    
        if ( _splatTexUnit >= 0 )
        {
            engine->getResources()->releaseTextureImageUnit( _splatTexUnit );
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
        brightnessBuf,
        contrastBuf,
        thresholdBuf,
        slopeBuf;

    unsigned
        primaryCount    = 0,
        detailCount     = 0,
        brightnessCount = 0,
        contrastCount   = 0,
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

                        if ( rangeData._detail->_brightness.isSet() )
                        {
                            if ( brightnessCount == 0 )
                                brightnessBuf << IND "brightness += ";
                            else
                                brightnessBuf << " + ";
                            brightnessBuf << "w"<<val << "*" << rangeData._detail->_brightness.get();
                            brightnessCount++;
                        }

                        if ( rangeData._detail->_contrast.isSet() )
                        {
                            if ( contrastCount == 0 )
                                contrastBuf << IND "contrast += ";
                            else
                                contrastBuf << " + ";
                            contrastBuf << "w"<<val << "*" << rangeData._detail->_contrast.get();
                            contrastCount++;
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
                                slopeBuf << IND "slope += ";
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

    if ( brightnessCount > 0 )
        brightnessBuf << ";\n";

    if ( contrastCount > 0 )
        contrastBuf << ";\n";

    if ( thresholdCount > 0 )
        thresholdBuf << ";\n";

    if ( slopeCount > 0 )
        slopeBuf << ";\n";

    Shaders package;
    std::string code = ShaderLoader::load(
        package.FragGetRenderInfo,
        package);

    std::string codeToInject = Stringify()
        << IND
        << weightBuf.str()
        << primaryBuf.str()
        << detailBuf.str()
        << brightnessBuf.str()
        << contrastBuf.str()
        << thresholdBuf.str()
        << slopeBuf.str();

    osgEarth::replaceIn(code, "$CODE_INJECTION_POINT", codeToInject);

    textureDef._samplingFunction = code;

    OE_DEBUG << LC << "Sampling function = \n" << code << "\n\n";
}

osg::Texture*
SplatTerrainEffect::createNoiseTexture() const
{
    const int size = 1024;
    const int slices = 1;

    GLenum type = slices > 2 ? GL_RGBA : GL_LUMINANCE;
    
    osg::Image* image = new osg::Image();
    image->allocateImage(size, size, 1, type, GL_UNSIGNED_BYTE);

    // 0 = rocky mountains..
    // 1 = warping...
    const float F[4] = { 4.0f, 16.0f, 4.0f, 8.0f };
    const float P[4] = { 0.8f,  0.6f, 0.8f, 0.9f };
    const float L[4] = { 2.2f,  1.7f, 3.0f, 4.0f };
    
    for(int k=0; k<slices; ++k)
    {
        // Configure the noise function:
        osgEarth::Util::SimplexNoise noise;
        noise.setNormalize( true );
        noise.setRange( 0.0, 1.0 );
        noise.setFrequency( F[k] );
        noise.setPersistence( P[k] );
        noise.setLacunarity( L[k] );
        noise.setOctaves( 8 );

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

#if 0
    std::string filename("noise.png");
    osgDB::writeImageFile(*image, filename);
    OE_NOTICE << LC << "Wrote noise texture to " << filename << "\n";
#endif

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
