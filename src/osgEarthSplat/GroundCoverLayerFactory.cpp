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
#include "GroundCoverLayerFactory"
#include "SplatOptions"
#include "NoiseTextureFactory"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/URI>
#include <osgEarth/ShaderLoader>
#include <osgEarth/ImageUtils>
#include <osgEarth/PatchLayer>
#include <osgEarth/Shadowing>

#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osg/Multisample>
#include <osgUtil/Optimizer>

#include "SplatShaders"

#define LC "[GroundCoverLayerFactory] "

using namespace osgEarth;
using namespace osgEarth::Splat;

namespace
{
    //TODO: fix ambiguous inheritance issue here:
    class GroundCoverPatchLayer : public PatchLayer
    {
    public:
        struct AcceptCallbackAdapter : public PatchLayer::AcceptCallback
        {
            GroundCoverPatchLayer* _owner;

            AcceptCallbackAdapter(GroundCoverPatchLayer* owner) : _owner(owner) { }

            bool acceptLayer(osg::NodeVisitor& nv, const osg::Camera* camera) const {
                return _owner->acceptLayer(nv, camera);
            }

            bool acceptKey(const TileKey& key) const { 
                return _owner->acceptKey(key);
            }
        };

    public:
        GroundCoverPatchLayer() : PatchLayer()
        {
            this->setAcceptCallback(new AcceptCallbackAdapter(this));
        }

        void setZoneAndLayer(const Zone* zone, const GroundCoverLayer* layer)
        {
            _zone = zone;
            _GroundCoverLayer = layer;
        }

    public: // AcceptCallbackAdapter

        /**
         * Whether to cull this layer. We only want to cull this layer if 
         * (a) its zone is currently active; and (b) this is either a shadowmap camera
         * or a rendering camera.
         */
        bool acceptLayer(osg::NodeVisitor& nv, const osg::Camera* camera) const
        {
            // If this layer doesn't belong to the active zone, reject it.
            const Zone* zone = VisitorData::fetch<const Zone>(nv, "oe.GroundCover.zone");
            if (zone && zone != _zone.get())
                return false;
            
            // if this is a shadow camera and the layer is configured to cast shadows, accept it.
            if (osgEarth::Shadowing::isShadowCamera(camera))
            {
                bool ok = (_GroundCoverLayer->getCastShadows() == true);
                //OE_INFO << "Layer=" << getName() << " - shadow pass (" << ok << ")\n";
                return ok;
            }

            // if this is a depth-pass camera (and not a shadow cam), reject it.
            unsigned clearMask = camera->getClearMask();
            bool isDepthCamera = ((clearMask & GL_COLOR_BUFFER_BIT) == 0u) && ((clearMask & GL_DEPTH_BUFFER_BIT) != 0u);
            if (isDepthCamera)
                return false;

            // otherwise, accept it.
            //OE_INFO << "Layer="<<getName()<< " - render pass.\n";
            return true;
        }

        bool acceptKey(const TileKey& key) const
        {
            return _GroundCoverLayer->getLOD() == key.getLOD();
        }

    private:
        osg::ref_ptr<const Zone> _zone;
        osg::ref_ptr<const GroundCoverLayer> _GroundCoverLayer;
    };
}


GroundCoverLayerFactory::GroundCoverLayerFactory() :
_noiseTexUnit    ( -1 ),
_GroundCoverTexUnit( -1 )
{
    //nop
}

void
GroundCoverLayerFactory::setDBOptions(const osgDB::Options* dbo)
{
    _dbo = dbo;
}

//namespace
//{
//    struct AcceptLOD : public PatchLayer::AcceptCallback {
//        unsigned _lod;
//        AcceptLOD(unsigned lod) { _lod = lod; }
//        bool accept(const TileKey& key) const {
//            return key.getLOD() == _lod;
//        }
//    };
//}

void
GroundCoverLayerFactory::install(MapNode* mapNode)
{    
#if 0
    if ( !mapNode )
        return;

    // first make sure there is land cover data available:
    bool GroundCoverActive = false;
    for (Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        Zone* zone = z->get();
        GroundCover* GroundCover = zone->getGroundCover();
        if (GroundCover)
        {
            GroundCoverActive = true;
        }
    }

    if (GroundCoverActive)
    {
        //if (!mapNode->getTerrainEngine()->getResources()->reserveTextureImageUnit(_noiseTexUnit, "Noise"))
        //{
        //    OE_WARN << LC << "No texture image unit available for Node." << std::endl;
        //    return;
        //}

        if (!mapNode->getTerrainEngine()->getResources()->reserveTextureImageUnit(_GroundCoverTexUnit, "GroundCover"))
        {
            OE_WARN << LC << "No texture image unit available for GroundCover." << std::endl;
            return;
        }

        //NoiseTextureFactory noise;
        //osg::ref_ptr<osg::Texture> noiseTexture = noise.create(256u, 4u);

        for (Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
        {
            Zone* zone = z->get();
            GroundCover* GroundCover = zone->getGroundCover();
            if (GroundCover)
            {
                for (GroundCoverLayers::iterator i = GroundCover->getLayers().begin();
                    i != GroundCover->getLayers().end();
                    ++i)
                {
                    GroundCoverLayer* layer = i->get();
                    if (layer)
                    {
                        if (!layer->getBiomes().empty() || layer->getTotalNumBillboards() > 0)
                        {
                            osg::StateSet* stateset = layer->getOrCreateStateSet();    
                            
                            //stateset->setTextureAttribute(_noiseTexUnit, noiseTexture.get());
                            //stateset->addUniform(new osg::Uniform("oe_splat_noiseTex", _noiseTexUnit));

                            bool useMask = (GroundCover->getMaskLayer() != 0L);

                            // Install the land cover shaders on the state set
                            VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
                            vp->setName("Land Cover (" + layer->getName() + ")");
                            GroundCoverShaders shaders;
                            if (useMask)
                            {
                                shaders.replace("MASK_SAMPLER", GroundCover->getMaskLayer()->shareTexUniformName().get());
                                shaders.replace("MASK_TEXTURE", GroundCover->getMaskLayer()->shareTexMatUniformName().get());
                            }
                            shaders.loadAll(vp, _dbo.get());

                            // Generate the coverage acceptor shader
                            osg::Shader* covTest = layer->createPredicateShader(getCoverage());
                            covTest->setName(covTest->getName() + "_GEOMETRY");
                            covTest->setType(osg::Shader::GEOMETRY);
                            vp->setShader(covTest);

                            osg::Shader* covTest2 = layer->createPredicateShader(getCoverage());
                            covTest->setName(covTest->getName() + "_TESSCONTROL");
                            covTest2->setType(osg::Shader::TESSCONTROL);
                            vp->setShader(covTest2);

                            osg::ref_ptr<osg::Shader> layerShader = layer->createShader();
                            layerShader->setType(osg::Shader::GEOMETRY);
                            vp->setShader(layerShader);

                            OE_INFO << LC << "Adding land cover layer \"" << layer->getName() << "\" to zone \"" << zone->getName() << "\" at LOD " << layer->getLOD() << "\n";

                            // Install the uniforms
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_windFactor", layer->getWind()));
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_noise", 1.0f));
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_ao", 0.5f));
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_exposure", 1.0f));

                            stateset->addUniform(new osg::Uniform("oe_GroundCover_density", layer->getDensity()));
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_fill", layer->getFill()));
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_maxDistance", layer->getMaxDistance()));

                            stateset->addUniform(new osg::Uniform("oe_GroundCover_brightness", layer->getBrightness()));
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_contrast", layer->getContrast()));

                            stateset->addUniform(new osg::Uniform("oe_GroundCover_useMask", useMask));

                            // enable alpha-to-coverage multisampling for vegetation.
                            stateset->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);

                            // uniform that communicates the availability of multisampling.
                            stateset->addUniform(new osg::Uniform(
                                "oe_terrain_hasMultiSamples",
                                osg::DisplaySettings::instance()->getMultiSamples()));

                            stateset->setAttributeAndModes(
                                new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO),
                                osg::StateAttribute::OVERRIDE);

                            //stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

                            osg::Texture* tex = layer->createTexture();

                            stateset->setTextureAttribute(_GroundCoverTexUnit, tex);
                            stateset->addUniform(new osg::Uniform("oe_GroundCover_texArray", _GroundCoverTexUnit));


                            //osgEarth::PatchLayer* patch = new osgEarth::PatchLayer();
                            GroundCoverPatchLayer* patch = new GroundCoverPatchLayer();
                            patch->setZoneAndLayer(zone, layer);
                            patch->setName("Land Cover: " + layer->getName());
                            //patch->setAcceptCallback( new AcceptLOD(layer->getLOD()) );
                            patch->setStateSet( stateset );
                            mapNode->getMap()->addLayer( patch );
                        }
                        else
                        {
                            OE_WARN << LC << "ILLEGAL: land cover layer with no biomes or no billboards defined\n";
                        }
                    }
                    else
                    {
                        OE_WARN << LC << "ILLEGAL: empty layer found in land cover layer list\n";
                    }
                }
            }
            else
            {
                // not an error.
                OE_DEBUG << LC << "zone contains no land cover information\n";
            }
        }
    }
    else
    {
        OE_DEBUG << LC << "No land cover information found\n";
    }
#endif
}

void
GroundCoverLayerFactory::uninstall(MapNode* mapNode)
{
    //TODO
    if (mapNode)
    {
        if (_GroundCoverTexUnit >= 0)
            mapNode->getTerrainEngine()->getResources()->releaseTextureImageUnit(_GroundCoverTexUnit);
    
        if (_noiseTexUnit >= 0)
            mapNode->getTerrainEngine()->getResources()->releaseTextureImageUnit(_noiseTexUnit);
    }           
}