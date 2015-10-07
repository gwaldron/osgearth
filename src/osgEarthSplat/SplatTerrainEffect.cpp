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
#include "BiomeRegion"
#include "NoiseTextureFactory"

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
#define NOISE_SAMPLER    "oe_noise_tex"

using namespace osgEarth;
using namespace osgEarth::Splat;

SplatTerrainEffect::SplatTerrainEffect() :
_renderOrder ( -1.0f ),
_editMode    ( false ),
_gpuNoise    ( false )
{
    _scaleOffsetUniform = new osg::Uniform("oe_splat_scaleOffsetInt", 0 );
    _warpUniform        = new osg::Uniform("oe_splat_warp",           0.0f );
    _blurUniform        = new osg::Uniform("oe_splat_blur",           1.0f );
    _useBilinearUniform = new osg::Uniform("oe_splat_useBilinear",    1.0f );
    _noiseScaleUniform  = new osg::Uniform("oe_splat_noiseScale",    12.0f );

    _editMode = (::getenv("OSGEARTH_SPLAT_EDIT") != 0L);
    _gpuNoise = (::getenv("OSGEARTH_SPLAT_GPU_NOISE") != 0L);
}

void
SplatTerrainEffect::setDBOptions(const osgDB::Options* dbo)
{
    _dbo = dbo;
}

void
SplatTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        // Check that we have coverage data (required for now - later masking data will be an option)
        if ( !_coverage.valid() || !_coverage->hasLayer() )
        {
            OE_WARN << LC << "ILLEGAL: coverage data is required\n";
            return;
        }

        if ( !_surface.valid() )
        {
            OE_WARN << LC << "Note: no surface information available\n";
        }

        // First, create a surface splatting texture array for each biome region.
        if ( _coverage.valid() && _surface.valid() )
        {
            if ( createSplattingTextures(_coverage.get(), _surface.get(), _textureDefs) == false )
            {
                OE_WARN << LC << "Failed to create any valid splatting textures\n";
                return;
            }
            
            // First install a shared noise texture.
            if ( _gpuNoise == false )
            {
                osg::StateSet* terrainStateSet = engine->getOrCreateStateSet();
                if ( terrainStateSet->getUniform("oe_splat_noiseTex") == 0L )
                {
                    // reserve a texture unit:
                    if (engine->getResources()->reserveTextureImageUnit(_noiseTexUnit, "Splat Noise"))
                    {
                        NoiseTextureFactory noise;
                        terrainStateSet->setTextureAttribute( _noiseTexUnit, noise.create(256u, 4u) );
                        terrainStateSet->addUniform( new osg::Uniform("oe_splat_noiseTex", _noiseTexUnit) );
                    }
                }
            }

            // Set up surface splatting:
            if ( engine->getResources()->reserveTextureImageUnit(_splatTexUnit, "Splat Coverage Data") )
            {
                osg::StateSet* stateset;

                if ( _surface->getBiomeRegions().size() == 1 )
                    stateset = engine->getSurfaceStateSet();
                else
                    stateset = new osg::StateSet();

                // surface splatting texture array:
                _splatTexUniform = stateset->getOrCreateUniform( SPLAT_SAMPLER, osg::Uniform::SAMPLER_2D_ARRAY );
                _splatTexUniform->set( _splatTexUnit );
                stateset->setTextureAttribute( _splatTexUnit, _textureDefs[0]._texture.get() );

                // coverage code sampler:
                osg::ref_ptr<ImageLayer> coverageLayer;
                _coverage->lockLayer( coverageLayer );

                _coverageTexUniform = stateset->getOrCreateUniform( COVERAGE_SAMPLER, osg::Uniform::SAMPLER_2D );
                _coverageTexUniform->set( coverageLayer->shareImageUnit().get() );

                // control uniforms (TODO: simplify and deprecate unneeded uniforms)
                stateset->addUniform( _scaleOffsetUniform.get() );
                stateset->addUniform( _warpUniform.get() );
                stateset->addUniform( _blurUniform.get() );
                stateset->addUniform( _noiseScaleUniform.get() );
                stateset->addUniform( _useBilinearUniform.get() );

                stateset->addUniform(new osg::Uniform("oe_splat_detailRange",  1000000.0f));


                SplattingShaders splatting;

                splatting.define( "SPLAT_EDIT",        _editMode );
                splatting.define( "SPLAT_GPU_NOISE",   _gpuNoise );
                splatting.define( "OE_USE_NORMAL_MAP", engine->normalTexturesRequired() );

                splatting.replace( "$COVERAGE_TEXTURE_MATRIX", coverageLayer->shareTexMatUniformName().get() );
            
                VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
                splatting.load( vp, splatting.VertModel );
                splatting.load( vp, splatting.VertView );
                splatting.load( vp, splatting.Frag );
                splatting.load( vp, splatting.Util );

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

                    // support shaders
                    std::string noiseShaderSource = ShaderLoader::load( splatting.Noise, splatting );
                    osg::Shader* noiseShader = new osg::Shader(osg::Shader::FRAGMENT, noiseShaderSource);
                    vp->setShader( "oe_splat_noiseshaders", noiseShader );
                }

                // TODO: disabled biome selection temporarily because the callback impl applies the splatting shader
                // to the land cover bin as well as the surface bin, which we do not want -- find another way
                if ( _surface->getBiomeRegions().size() == 1 )
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
                {
                    OE_WARN << LC << "Multi-biome setup needs re-implementing (reminder)\n";

                    // install the cull callback that will select the appropriate
                    // state based on the position of the camera.
                    _biomeRegionSelector = new BiomeRegionSelector(
                        _surface->getBiomeRegions(),
                        _textureDefs,
                        stateset,
                        _splatTexUnit );

                    engine->addCullCallback( _biomeRegionSelector.get() );
                }
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

        if ( _biomeRegionSelector.valid() )
        {
            engine->removeCullCallback( _biomeRegionSelector.get() );
            _biomeRegionSelector = 0L;
        }
    }
}

bool
SplatTerrainEffect::createSplattingTextures(const Coverage*        coverage,
                                            const Surface*         surface,
                                            SplatTextureDefVector& output) const
{
    int numValidTextures = 0;

    if ( coverage == 0L || surface == 0L )
        return false;

    const BiomeRegionVector& biomeRegions = surface->getBiomeRegions();

    OE_INFO << LC << "Creating splatting textures for " << biomeRegions.size() << " biome regions\n";

    // Create a texture def for each biome region.
    for(unsigned b = 0; b < biomeRegions.size(); ++b)
    {
        const BiomeRegion& biomeRegion = biomeRegions[b];

        // Create a texture array and lookup table for this region:
        SplatTextureDef def;

        if ( biomeRegion.getCatalog() )
        {
            if ( biomeRegion.getCatalog()->createSplatTextureDef(_dbo.get(), def) )
            {
                // install the sampling function.
                createSplattingSamplingFunction( coverage, def );
                numValidTextures++;
            }
            else
            {
                OE_WARN << LC << "Failed to create a texture for a catalog (" 
                    << biomeRegion.getCatalog()->name().get() << ")\n";
            }
        }
        else
        {
            OE_WARN << LC << "Biome Region \""
                << biomeRegion.name().get() << "\"" 
                << " has an empty catalog and will be ignored.\n";
        }

        // put it on the list either way, since the vector indicies of biomes
        // and texturedefs need to line up
        output.push_back( def );
    }

    return numValidTextures > 0;
}

#define IND "    "

bool
SplatTerrainEffect::createSplattingSamplingFunction(const Coverage*  coverage,
                                                    SplatTextureDef& textureDef) const
{
    if ( !coverage || !coverage->getLegend() )
    {
        OE_WARN << LC << "Sampling function: illegal state (no coverage or legend); \n";
        return false;
    }

    if ( !textureDef._texture.valid() )
    {
        OE_WARN << LC << "Internal: texture is not set; cannot create a sampling function\n";
        return false;
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

    const SplatCoverageLegend::Predicates& preds = coverage->getLegend()->getPredicates();
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

    SplattingShaders splatting;
    std::string code = ShaderLoader::load(
        splatting.FragGetRenderInfo,
        splatting);

    std::string codeToInject = Stringify()
        << IND
        << weightBuf.str()
        << primaryBuf.str()
        << detailBuf.str()
        << brightnessBuf.str()
        << contrastBuf.str()
        << thresholdBuf.str()
        << slopeBuf.str();

    osgEarth::replaceIn(code, "$COVERAGE_SAMPLING_FUNCTION", codeToInject);

    textureDef._samplingFunction = code;

    OE_DEBUG << LC << "Sampling function = \n" << code << "\n\n";

    return true;
}
