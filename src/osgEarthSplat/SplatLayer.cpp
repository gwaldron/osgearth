/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "SplatLayer"
#include "SplatShaders"
#include "NoiseTextureFactory"
#include <osgEarth/VirtualProgram>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/FeatureSourceLayer>
#include <osgUtil/CullVisitor>
#include <osg/BlendFunc>
#include <osg/Drawable>
#include <cstdlib> // getenv

#define LC "[SplatLayer] " << getName() << ": "

#define COVERAGE_SAMPLER "oe_splat_coverageTex"
#define SPLAT_SAMPLER    "oe_splatTex"
#define NOISE_SAMPLER    "oe_splat_noiseTex"
#define LUT_SAMPLER      "oe_splat_coverageLUT"

using namespace osgEarth::Splat;

namespace osgEarth { namespace Splat {
    REGISTER_OSGEARTH_LAYER(splat_imagery, SplatLayer);
} }

//........................................................................

Config
SplatLayerOptions::getConfig() const
{
    Config conf = VisibleLayerOptions::getConfig();
    conf.set("land_cover_layer", _landCoverLayerName);

    Config zones("zones");
    for (int i = 0; i < _zones.size(); ++i) {
        Config zone = _zones[i].getConfig();
        if (!zone.empty())
            zones.add(zone);
    }
    if (!zones.empty())
        conf.set(zones);
    return conf;
}

void
SplatLayerOptions::fromConfig(const Config& conf)
{
    conf.get("land_cover_layer", _landCoverLayerName);

    const Config* zones = conf.child_ptr("zones");
    if (zones) {
        const ConfigSet& children = zones->children();
        for (ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i) {
            _zones.push_back(ZoneOptions(*i));
        }
    }
}

//........................................................................

void
SplatLayer::ZoneSelector::operator()(osg::Node* node, osg::NodeVisitor* nv) const
{
    if (nv->getVisitorType() == nv->CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);

        // If we have zones, select the current one and apply its state set.
        if (_layer->_zones.size() > 0)
        {
            int zoneIndex = 0;
            osg::Vec3d vp = cv->getViewPoint();

            for(int z=_layer->_zones.size()-1; z > 0 && zoneIndex == 0; --z)
            {
                if ( _layer->_zones[z]->contains(vp) )
                {
                    zoneIndex = z;
                }
            }

            osg::StateSet* zoneStateSet = 0L;
            Surface* surface = _layer->_zones[zoneIndex]->getSurface();
            if (surface)
            {
                zoneStateSet = surface->getStateSet();
            }

            if (zoneStateSet == 0L)
            {
                OE_FATAL << LC << "ASSERTION FAILURE - zoneStateSet is null\n";
            }
            else
            {            
                cv->pushStateSet(zoneStateSet);
                traverse(node, nv);
                cv->popStateSet();
            }
        }
    }
    else
    {
        traverse(node, nv);
    }
}

//........................................................................

SplatLayer::SplatLayer() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

SplatLayer::SplatLayer(const SplatLayerOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
SplatLayer::init()
{
    VisibleLayer::init();

    _zonesConfigured = false;

    _editMode = (::getenv("OSGEARTH_SPLAT_EDIT") != 0L); // TODO deprecate
    _gpuNoise = (::getenv("OSGEARTH_SPLAT_GPU_NOISE") != 0L); // TODO deprecate

    setRenderType(osgEarth::Layer::RENDERTYPE_TERRAIN_SURFACE);

    for (std::vector<ZoneOptions>::const_iterator i = options().zones().begin();
        i != options().zones().end();
        ++i)
    {
        osg::ref_ptr<Zone> zone = new Zone(*i);
        _zones.push_back(zone.get());
    }

    setCullCallback(new ZoneSelector(this));
}

void
SplatLayer::setLandCoverDictionary(LandCoverDictionary* layer)
{
    _landCoverDict = layer;
    if (layer)
        buildStateSets();
}

void
SplatLayer::setLandCoverLayer(LandCoverLayer* layer)
{
    _landCoverLayer = layer;
    if (layer) {
        buildStateSets();
    }
}

void
SplatLayer::addedToMap(const Map* map)
{
    if (!_landCoverDict.valid())
    {
        _landCoverDictListener.listen(map, this, &SplatLayer::setLandCoverDictionary);
    }

    if (!_landCoverLayer.valid() && options().landCoverLayer().isSet())
    {
        _landCoverListener.listen(map, options().landCoverLayer().get(), this, &SplatLayer::setLandCoverLayer);
    }

    for (Zones::iterator zone = _zones.begin(); zone != _zones.end(); ++zone)
    {
        zone->get()->configure(map, getReadOptions());
    }

    _zonesConfigured = true;
    
    buildStateSets();
}

void
SplatLayer::removedFromMap(const Map* map)
{
    //NOP
}

void
SplatLayer::setTerrainResources(TerrainResources* res)
{
    VisibleLayer::setTerrainResources(res);

    if (res)
    {
        // TODO.
        // These reservations are Layer-specific, so we should add the
        // capability to TerrainResources to support per-Layer reservations.
        if (_splatBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_splatBinding, this, "Splat texture") == false)
            {
                OE_WARN << LC << "No texture unit available for splatting texture\n";
            }
        }

        if (_lutBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_lutBinding, this, "Splatting LUT") == false)
            {
                OE_WARN << LC << "No texture unit available for splatting LUT\n";
            }
        }

        if (_noiseBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_noiseBinding, this, "Splat noise sampler") == false)
            {
                OE_WARN << LC << "No texture unit available for splatting Noise function\n";
            }
        }

        if (_splatBinding.valid() && _lutBinding.valid())
        {
            buildStateSets();
        }
    }
}

void
SplatLayer::buildStateSets()
{
    // assert we have the necessary TIUs:
    if (_splatBinding.valid() == false || _lutBinding.valid() == false) {
        OE_DEBUG << LC << "buildStateSets deferred.. bindings not reserved\n";
        return;
    }

    if (!_zonesConfigured) {
        OE_DEBUG << LC << "buildStateSets deferred.. zones not yet configured\n";
        return;
    }
    
    osg::ref_ptr<LandCoverDictionary> landCoverDict;
    if (_landCoverDict.lock(landCoverDict) == false) {
        OE_DEBUG << LC << "buildStateSets deferred.. land cover dictionary not available\n";
        return;
    }
    
    osg::ref_ptr<LandCoverLayer> landCoverLayer;
    if (_landCoverLayer.lock(landCoverLayer) == false) {
        OE_DEBUG << LC << "buildStateSets deferred.. land cover layer not available\n";
        return;
    }

    // Load all the splatting textures
    for (Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        Zone* zone = z->get();
        Surface* surface = z->get()->getSurface();
        if (surface == 0L)
        {
            OE_WARN << LC << "No surface defined for zone " << zone->getName() << std::endl;
            return;
        }
        if (surface->loadTextures(landCoverDict.get(), getReadOptions()) == false)
        {
            OE_WARN << LC << "Texture load failed for zone " << zone->getName() << "\n";
            return;
        }
    }

    // Set up the zone-specific elements:
    for (Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        Zone* zone = z->get();

        osg::StateSet* zoneStateset = zone->getSurface()->getOrCreateStateSet();
        zoneStateset->setName("Splat Zone");

        // The texture array for the zone:
        const SplatTextureDef& texdef = zone->getSurface()->getTextureDef();

        // apply the splatting texture catalog:
        zoneStateset->setTextureAttribute(_splatBinding.unit(), texdef._texture.get());

        // apply the buffer containing the coverage-to-splat LUT:
        zoneStateset->setTextureAttribute(_lutBinding.unit(), texdef._splatLUTBuffer.get());

        OE_DEBUG << LC << "Installed getRenderInfo for zone \"" << zone->getName() << "\" (uid=" << zone->getUID() << ")\n";
    }

    // Next set up the elements that apply to all zones:
    osg::StateSet* stateset = this->getOrCreateStateSet();

    // Bind the texture image unit:
    stateset->addUniform(new osg::Uniform(SPLAT_SAMPLER, _splatBinding.unit()));

    // install the uniform for the splat LUT.
    stateset->addUniform(new osg::Uniform(LUT_SAMPLER, _lutBinding.unit()));
        
    if (_noiseBinding.valid())
    {
        NoiseTextureFactory noise;
        osg::ref_ptr<osg::Texture> noiseTexture = noise.create(256u, 1u);
        stateset->setTextureAttribute(_noiseBinding.unit(), noiseTexture.get());
        stateset->addUniform(new osg::Uniform(NOISE_SAMPLER, _noiseBinding.unit()));
        stateset->setDefine("OE_SPLAT_NOISE_SAMPLER", NOISE_SAMPLER);
    }

    osg::Uniform* lcTexUniform = new osg::Uniform(COVERAGE_SAMPLER, landCoverLayer->shareImageUnit().get());
    stateset->addUniform(lcTexUniform);

    stateset->addUniform(new osg::Uniform("oe_splat_scaleOffsetInt", 0));
    stateset->addUniform(new osg::Uniform("oe_splat_noiseScale", 12.0f));
    stateset->addUniform(new osg::Uniform("oe_splat_detailRange", 100000.0f));

    if (_editMode)
        stateset->setDefine("OE_SPLAT_EDIT_MODE");

    if (_gpuNoise)
        stateset->setDefine("OE_SPLAT_GPU_NOISE");

    stateset->setDefine("OE_USE_NORMAL_MAP");

    stateset->setDefine("OE_SPLAT_COVERAGE_TEXMAT", landCoverLayer->shareTexMatUniformName().get());
    
    //stateset->setAttributeAndModes(
    //    new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ZERO),
    //    osg::StateAttribute::OVERRIDE);

    SplattingShaders splatting;
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->setName("SplatLayer");
    splatting.load(vp, splatting.VertModel);
    splatting.load(vp, splatting.VertView);
    splatting.load(vp, splatting.Frag);
    splatting.load(vp, splatting.Util);

    OE_DEBUG << LC << "Statesets built!! Ready!\n";
}


void
SplatLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    for (Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        z->get()->resizeGLObjectBuffers(maxSize);
    }

    VisibleLayer::resizeGLObjectBuffers(maxSize);
}

void
SplatLayer::releaseGLObjects(osg::State* state) const
{
    for (Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        z->get()->releaseGLObjects(state);
    }

    VisibleLayer::releaseGLObjects(state);

    // For some unknown reason, release doesn't work on the zone 
    // texture def data (SplatTextureDef). So we have to recreate
    // it here.
    const_cast<SplatLayer*>(this)->buildStateSets();
}
