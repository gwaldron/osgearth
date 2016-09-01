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
#include <osg/TextureBuffer>
#include <osgDB/WriteFile>

#include "SplatShaders"

#define LC "[Splat] "

#define COVERAGE_SAMPLER "oe_splat_coverageTex"
#define SPLAT_SAMPLER    "oe_splatTex"
#define NOISE_SAMPLER    "oe_noise_tex"
#define LUT_SAMPLER      "oe_splat_coverageLUT"

using namespace osgEarth;
using namespace osgEarth::Splat;

SplatTerrainEffect::SplatTerrainEffect() :
_renderOrder ( -1.0f ),
_editMode    ( false ),
_gpuNoise    ( false ),
_splatTexUnit(-1),
_lutTexUnit(-1),
_noiseTexUnit(-1)
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

        bool splattingOK = false;

        for(Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
        {
            Zone* zone = z->get();
            Surface* surface = zone->getSurface();
            if ( surface )
            {
                if ( surface->loadTextures(_coverage.get(), _dbo.get()) )
                {
                    splattingOK = true;
                }
            }
        }

        if ( splattingOK )
        {
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

            bool coverageOK =  engine->getResources()->reserveTextureImageUnit(_splatTexUnit, "Splat Coverage Data");

            bool lutOK = engine->getResources()->reserveTextureImageUnit(_lutTexUnit, "Splat LUT");

            // Set up surface splatting:
            if ( coverageOK && lutOK )
            {
                // Set up the zone-specific elements:
                for(Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
                {
                    Zone* zone = z->get();

                    // The texture array for the zone:
                    const SplatTextureDef& texdef = zone->getSurface()->getTextureDef();
                    osg::StateSet* zoneStateset = zone->getOrCreateStateSet();

                    // apply the splatting texture catalog:
                    zoneStateset->setTextureAttribute( _splatTexUnit, texdef._texture.get() );

                    // apply the buffer containing the coverage-to-splat LUT:
                    zoneStateset->setTextureAttribute(_lutTexUnit, texdef._splatLUTBuffer.get());

                    // The zone's sampling function:
                    //VirtualProgram* vp = VirtualProgram::cloneOrCreate( zoneStateset );
                    //osg::Shader* shader = new osg::Shader(osg::Shader::FRAGMENT, texdef._samplingFunction);
                    //vp->setShader( "oe_splat_getRenderInfo", shader );

                    OE_INFO << LC << "Installed getRenderInfo for zone \"" << zone->getName() << "\" (uid=" << zone->getUID() << ")\n";
                }

                // Next set up the elements that apply to all zones:
                osg::StateSet* stateset = engine->getSurfaceStateSet();

                // Bind the texture image unit:
                _splatTexUniform = new osg::Uniform(SPLAT_SAMPLER, _splatTexUnit);
                stateset->addUniform( _splatTexUniform.get() );

                // install the uniform for the splat LUT.
                _lutTexUniform = stateset->getOrCreateUniform( LUT_SAMPLER, osg::Uniform::SAMPLER_BUFFER );
                _lutTexUniform->set(_lutTexUnit);

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

                splatting.replace( "COVERAGE_TEXTURE_MATRIX", coverageLayer->shareTexMatUniformName().get() );
            
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
    }
}
