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
#include "GroundCoverLayer"
#include "SplatShaders"
#include "NoiseTextureFactory"
#include <osgEarth/VirtualProgram>
#include <osgEarth/CameraUtils>
#include <osgEarth/Shaders>
#include <osgEarth/LineDrawable>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Math>
#include <osgEarth/Metrics>

#include <osg/BlendFunc>
#include <osg/Multisample>
#include <osg/Texture2D>
#include <osg/Depth>
#include <osg/Version>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/CullVisitor>
#include <osgUtil/Optimizer>

#include <cstdlib> // getenv

#define LC "[GroundCoverLayer] " << getName() << ": "

#define OE_DEVEL OE_DEBUG

//#define ATLAS_SAMPLER "oe_gc_atlas"
#define NOISE_SAMPLER "oe_gc_noiseTex"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(groundcover, GroundCoverLayer);
REGISTER_OSGEARTH_LAYER(splat_groundcover, GroundCoverLayer);

// TODO LIST
//
//  - Texture management as the catalog gets bigger. Swap in/out criteria and detection??

//  - Fix model culling. The "radius" isn't quite sufficient since the origin is not at the center,
//    AND because rotation changes the profile. Calculate it differently.

//  - Idea: include a "model range" or "max SSE" in the InstanceBuffer...?
//  - [PERF] thin out distant instances automatically in large tiles
//  - [PERF] cull by "horizon" .. e.g., the lower you are, the fewer distant trees...?
//  - Figure out exactly where some of this functionality goes -- GCL or IC? For example the
//    TileManager stuff seems like it would go in IC?
//  - Allow us to store key, slot, etc data in the actual TileContext coming from the 
//    PatchLayer. It is silly to have to do TileKey lookups and not be able to simple
//    iterate over the TileBatch.
//  - fix the random asset select with weighting...just not really working well.
//  - programmable SSE for models?
//  - deal with blending to fade in 3d models
//  - Reduce the size of the RenderBuffer structure
//  - variable spacing or clumping by landcovergroup or asset...?
//  - make the noise texture bindless as well? Stick it in the arena? Why not.

//  - (DONE) [OPT] reduce the SIZE of the instance buffer by re-using variables (instanceID, drawID, assetID)
//  - (DONE) Figure out how to pre-compile/ICO the TextureArena; it takes quite a while.
//  - (DONE) [OPT] on Generate, store the instance count per tile in an array somewhere. Also store the 
//    TOTAL instance count in the DI buffer. THen use this to dispatchComputeIndirect for the
//    MERGE. That way we don't have to dispatch to "all possible locations" during a merge,
//    hopefully making it much faster and avoiding frame breaks.
//  - (DONE) FIX the multi-GC case. A resident handle can't cross GCs, so we will need
//    to upgrade TextureArena (Texture) to store one object/handle per GC.
//    We will also need to replace the sampler IDs in teh LUT shader with a mapping
//    to a UBO (or something) that holds the actual resident handles. Blah..
//  - (DONE) Properly delete all GL memory .. use the Releaser on the GC thread?
//  - (DONE) Can we remove the normal matrix?
//  - (DONE) [OPT] combine multiple object lists into the GLObjectReleaser
//  - (DONE) FIX to work with SHADOW camera (separate culling...maybe?) (reference viewpoint?)
//  - (DONE) FIX the multi-CAMERA case.
//  - (DONE) Do NOT regenerate every tile every time the tilebatch changes!
//  - (DONE .. had to call glBindBufferRange each frame) Two GC layers at the same time doesn't work! (grass + trees)
//  - (DONE) FIX: IC's atlas is hard-coded to texture image unit 11. Allocate it dynamically.
//  - (DONE .. the lighting shader was executing in the wrong order) Lighting
//  - (DONE .. was good!!) read back the instance count to reduce the dispatch #?
//  - (DONE .. was a bad oe_gc_Assets setup in the LUT shader) Fix teh "flashing" bug :(
//  - (DONE .. merged at layer level) BUGFIX: we're merging the geometrycloud's stateset into the layer's stateset. Make sure that's kosher...?
//  - (DONE) Fix the GRASS LAYER
//  - (DONE) Rotation for models
//  - (DONE) Scaling of the 3D models to match the height/width....? or not? set from loaded model?

//........................................................................

Config
GroundCoverLayer::Options::getConfig() const
{
    Config conf = PatchLayer::Options::getConfig();
    maskLayer().set(conf, "mask_layer");
    colorLayer().set(conf, "color_layer");
    conf.set("color_min_saturation", colorMinSaturation());
    conf.set("lod", _lod);
    conf.set("cast_shadows", _castShadows);
    conf.set("max_alpha", maxAlpha());
    conf.set("alpha_to_coverage", alphaToCoverage());
    conf.set("max_sse", maxSSE());

    Config zones("zones");
    for (int i = 0; i < _biomeZones.size(); ++i) {
        Config zone = _biomeZones[i].getConfig();
        if (!zone.empty())
            zones.add(zone);
    }
    if (!zones.empty())
        conf.set(zones);
    return conf;
}

void
GroundCoverLayer::Options::fromConfig(const Config& conf)
{
    // defaults:
    lod().setDefault(13u);
    castShadows().setDefault(false);
    maxAlpha().setDefault(0.15f);
    alphaToCoverage().setDefault(true);

    maskLayer().get(conf, "mask_layer");
    colorLayer().get(conf, "color_layer");
    conf.get("color_min_saturation", colorMinSaturation());
    conf.get("lod", _lod);
    conf.get("cast_shadows", _castShadows);
    conf.get("max_alpha", maxAlpha());
    conf.get("alpha_to_coverage", alphaToCoverage());
    conf.get("max_sse", maxSSE());

    const Config* zones = conf.child_ptr("zones");
    if (zones)
    {
        const ConfigSet& children = zones->children();
        for (ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i)
        {
            _biomeZones.push_back(BiomeZone(*i));
        }
    }
    else if (conf.hasChild("groundcover"))
    {
        BiomeZone zone(conf);
        _biomeZones.push_back(zone);
    }
}

//........................................................................

bool
GroundCoverLayer::LayerAcceptor::acceptLayer(osg::NodeVisitor& nv, const osg::Camera* camera) const
{
    // if this is a shadow camera and the layer is configured to cast shadows, accept it.
    if (CameraUtils::isShadowCamera(camera))
    {
        return _layer->getCastShadows();
    }

    // if this is a depth-pass camera (and not a shadow cam), reject it.
    if (CameraUtils::isDepthCamera(camera))
    {
        return false;
    }

    // otherwise accept the layer.
    return true;
}

bool
GroundCoverLayer::LayerAcceptor::acceptKey(const TileKey& key) const
{
    return _layer->getLOD() == key.getLOD();
}

//........................................................................

namespace
{
    // Trickt o store the BiomeLayout pointer in a stateattribute so we can track it during cull/draw
    struct ZoneSA : public osg::StateAttribute
    {
        META_StateAttribute(osgEarth, ZoneSA, (osg::StateAttribute::Type)(osg::StateAttribute::CAPABILITY + 90210));
        BiomeZone* _obj;
        ZoneSA() : _obj(NULL) { }
        ZoneSA(const ZoneSA& sa, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY) : osg::StateAttribute(sa, copyop), _obj(sa._obj) { }
        ZoneSA(BiomeZone* obj) : _obj(obj) { }
        virtual int compare(const StateAttribute& sa) const { return 0; }
        static const ZoneSA* extract(const osg::State* state) {
            osg::State::AttributeMap::const_iterator i = state->getAttributeMap().find(
                std::make_pair((osg::StateAttribute::Type)(osg::StateAttribute::CAPABILITY + 90210), 0));
            if (i == state->getAttributeMap().end()) return NULL;
            if (i->second.attributeVec.empty()) return NULL;
            return dynamic_cast<const ZoneSA*>(i->second.attributeVec.front().first);
        }
    };
}

void
GroundCoverLayer::ZoneSelector::operator()(osg::Node* node, osg::NodeVisitor* nv) const
{
    if (nv->getVisitorType() == nv->CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);

        // If we have zones, select the current one and apply its state set.
        if (_layer && _layer->getZones().size() > 0)
        {
            int zoneIndex = 0;
            osg::Vec3d vp = cv->getViewPoint();

            for(int z=_layer->getZones().size()-1; z > 0 && zoneIndex == 0; --z)
            {
                if ( _layer->getZones()[z].contains(vp) )
                {
                    zoneIndex = z;
                }
            }

            osg::StateSet* zoneStateSet = _layer->getZoneStateSet(zoneIndex);

            if (zoneStateSet)
            {
                cv->pushStateSet(zoneStateSet);
                traverse(node, nv);
                cv->popStateSet();
            }
            else
            {
                OE_FATAL << LC << "ASSERTION FAILURE - zoneStateSet is null - Layer disabled" << std::endl;
                const_cast<ZoneSelector*>(this)->_layer = NULL;
            }
        }
    }
    else
    {
        traverse(node, nv);
    }
}

//........................................................................

void GroundCoverLayer::setLOD(unsigned value) {
    options().lod() = value;
}
unsigned GroundCoverLayer::getLOD() const {
    return options().lod().get();
}

void GroundCoverLayer::setCastShadows(bool value) {
    options().castShadows() = value;
}
bool GroundCoverLayer::getCastShadows() const {
    return options().castShadows().get();
}

void GroundCoverLayer::setMaxSSE(float value)
{
    options().maxSSE() = value;
    if (_sseU.valid())
        _sseU->set(value);
}
float GroundCoverLayer::getMaxSSE() const
{
    return options().maxSSE().get();
}

void
GroundCoverLayer::init()
{
    PatchLayer::init();

    setAcceptCallback(new LayerAcceptor(this));

    setCullCallback(new ZoneSelector(this));

    _debug = (::getenv("OSGEARTH_GROUNDCOVER_DEBUG") != NULL);

    // evil
    //installDefaultOpacityShader();
}

GroundCoverLayer::~GroundCoverLayer()
{
    close();
}

Status
GroundCoverLayer::openImplementation()
{
    // GL version requirement
    if (Registry::capabilities().getGLSLVersion() < 4.6f)
    {
        return Status(Status::ResourceUnavailable, "Requires GL 4.6+");
    }

    // this layer will do its own custom rendering
    _renderer = new Renderer(this);
    setDrawCallback(_renderer.get());

    // make a 4-channel noise texture to use
    NoiseTextureFactory noise;
    _renderer->_noiseTex = noise.create(256u, 4u);

    return PatchLayer::openImplementation();
}

Status
GroundCoverLayer::closeImplementation()
{
    releaseGLObjects(NULL);

    setDrawCallback(NULL);
    _renderer = NULL;

    _liveAssets.clear();
    _zoneStateSets.clear();

    return PatchLayer::closeImplementation();
}

void
GroundCoverLayer::setLandCoverDictionary(LandCoverDictionary* layer)
{
    _landCoverDict.setLayer(layer);
    if (layer)
    {
        buildStateSets();
    }
}

LandCoverDictionary*
GroundCoverLayer::getLandCoverDictionary() const
{
    return _landCoverDict.getLayer();
}

void
GroundCoverLayer::setLandCoverLayer(LandCoverLayer* layer)
{
    _landCoverLayer.setLayer(layer);
    if (layer)
    {
        OE_INFO << LC << "Land cover layer is \"" << layer->getName() << "\"\n";
        buildStateSets();
    }
}

LandCoverLayer*
GroundCoverLayer::getLandCoverLayer() const
{
    return _landCoverLayer.getLayer();
}

void
GroundCoverLayer::setMaskLayer(ImageLayer* layer)
{
    options().maskLayer().setLayer(layer);
    if (layer)
    {
        buildStateSets();
    }
}

ImageLayer*
GroundCoverLayer::getMaskLayer() const
{
    return options().maskLayer().getLayer();
}

void
GroundCoverLayer::setColorLayer(ImageLayer* value)
{
    options().colorLayer().setLayer(value);
    if (value)
    {
        buildStateSets();
    }
}

ImageLayer*
GroundCoverLayer::getColorLayer() const
{
    return options().colorLayer().getLayer();
}

void
GroundCoverLayer::setMaxAlpha(float value)
{
    options().maxAlpha() = value;
}

float
GroundCoverLayer::getMaxAlpha() const
{
    return options().maxAlpha().get();
}

void
GroundCoverLayer::setUseAlphaToCoverage(bool value)
{
    options().alphaToCoverage() = value;
}

bool
GroundCoverLayer::getUseAlphaToCoverage() const
{
    return options().alphaToCoverage().get();
}

void
GroundCoverLayer::addedToMap(const Map* map)
{
    PatchLayer::addedToMap(map);

    if (!getLandCoverLayer())
        setLandCoverLayer(map->getLayer<LandCoverLayer>());

    if (!getLandCoverDictionary())
        setLandCoverDictionary(map->getLayer<LandCoverDictionary>());

    options().maskLayer().addedToMap(map);
    options().colorLayer().addedToMap(map);

    if (getMaskLayer())
    {
        OE_INFO << LC << "Mask layer is \"" << getMaskLayer()->getName() << "\"" << std::endl;
    }

    if (getColorLayer())
    {
        OE_INFO << LC << "Color modulation layer is \"" << getColorLayer()->getName() << "\"" << std::endl;
        if (getColorLayer()->isShared() == false)
        {
            OE_WARN << LC << "Color modulation is not shared and is therefore being disabled." << std::endl;
            options().colorLayer().removedFromMap(map);
        }
    }

    _mapProfile = map->getProfile();

    if (getLandCoverLayer() == NULL)
    {
        setStatus(Status::ResourceUnavailable, "No LandCover layer available in the Map");
        return;
    }

    if (getLandCoverDictionary() == NULL)
    {
        setStatus(Status::ResourceUnavailable, "No LandCoverDictionary available in the Map");
        return;
    }

    // Now that we have access to all the layers we need...

    // Make the texture atlas from the images found in the asset list
    _renderer->_texArena = new TextureArena();

    // Add to the stateset so it gets compiled and applied
    getOrCreateStateSet()->setAttribute(_renderer->_texArena.get());

    // Load asset data from the configuration.
    loadAssets(_renderer->_texArena.get());

    // Prepare model assets and add their textures to the atlas:
    _renderer->_geomCloud = createGeometryCloud(_renderer->_texArena.get());

    // bind the cloud's stateset to this layer.
    if (_renderer->_geomCloud.valid())
    {
        osg::StateSet* cloudSS = _renderer->_geomCloud->getGeometry()->getStateSet();
        if (cloudSS)
            getOrCreateStateSet()->merge(*cloudSS);
    }

    // now that we have HANDLES, we can make the LUT shader.
    // TODO: this probably needs to be Per-Context...
    osg::ref_ptr<osg::Shader> lutShader = createLUTShader();
    lutShader->setName("GroundCover CS LUT");
    _renderer->_computeProgram->addShader(lutShader.get());
}

void
GroundCoverLayer::removedFromMap(const Map* map)
{
    PatchLayer::removedFromMap(map);

    options().maskLayer().removedFromMap(map);
    options().colorLayer().removedFromMap(map);
}

void
GroundCoverLayer::setTerrainResources(TerrainResources* res)
{
    PatchLayer::setTerrainResources(res);

    if (res)
    {
        if (_noiseBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_noiseBinding, this, "GroundCover noise sampler") == false)
            {
                OE_WARN << LC << "No texture unit available for Ground cover Noise function\n";
            }
        }

        // if there's no LOD or max range set....set the LOD to a default value
        if (options().lod().isSet() == false && options().maxVisibleRange().isSet() == false)
        {
            setLOD(options().lod().get());
        }
        
        if (options().lod().isSet() == false && options().maxVisibleRange().isSet() == true)
        {
            unsigned bestLOD = 0;
            for(unsigned lod=1; lod<=99; ++lod)
            {
                bestLOD = lod-1;
                float lodRange = res->getVisibilityRangeHint(lod);
                if (getMaxVisibleRange() > lodRange || lodRange == FLT_MAX)
                {
                    break;
                }
            }
            setLOD(bestLOD);
            OE_INFO << LC << "Setting LOD to " << getLOD() << " based on a max range of " << getMaxVisibleRange() << std::endl;

        }

        else if (options().lod().isSet() == true && options().maxVisibleRange().isSet() == false)
        {
            float maxRange = res->getVisibilityRangeHint(getLOD());
            setMaxVisibleRange(maxRange);
            OE_INFO << LC << "Setting max visibility range for LOD " << getLOD() << " to " << maxRange << "m" << std::endl;
        }

        buildStateSets();
    }
}

void
GroundCoverLayer::buildStateSets()
{
    // assert we have the necessary prereqs:

    if (!_noiseBinding.valid()) {
        OE_DEBUG << LC << "buildStateSets deferred.. noise texture not yet bound" << std::endl;
        return;
    }

    if (!getLandCoverDictionary()) {
        OE_DEBUG << LC << "buildStateSets deferred.. land cover dictionary not available" << std::endl;
        return;
    }

    if (!options().lod().isSet()) {
        OE_DEBUG << LC << "buildStateSets deferred.. LOD not available" << std::endl;
        return;
    }

    if (!_renderer.valid()) {
        OE_WARN << LC << "buildStateSets deferred.. Renderer does not exist" << std::endl;
        return;
    }

    // calculate the tile width based on the LOD:
    if (getZones().size() > 0 && _mapProfile.valid())
    {
        unsigned tx, ty;
        _mapProfile->getNumTiles(getLOD(), tx, ty);
        GeoExtent e = TileKey(getLOD(), tx/2, ty/2, _mapProfile.get()).getExtent();
        GeoCircle c = e.computeBoundingGeoCircle();
        double width_m = 2.0 * c.getRadius() / 1.4142;
        _renderer->_tileWidth = width_m;
    }

    GroundCoverShaders shaders;

    // Layer-wide stateset:
    osg::StateSet* stateset = getOrCreateStateSet();

    // bind the noise sampler.
    stateset->setTextureAttribute(_noiseBinding.unit(), _renderer->_noiseTex.get());
    stateset->addUniform(new osg::Uniform(NOISE_SAMPLER, _noiseBinding.unit()));

    if (getMaskLayer())
    {
        stateset->setDefine("OE_GROUNDCOVER_MASK_SAMPLER", getMaskLayer()->getSharedTextureUniformName());
        stateset->setDefine("OE_GROUNDCOVER_MASK_MATRIX", getMaskLayer()->getSharedTextureMatrixUniformName());
    }
    else
    {
        stateset->removeDefine("OE_GROUNDCOVER_MASK_SAMPLER");
        stateset->removeDefine("OE_GROUNDCOVER_MASK_MATRIX");
    }

    if (getColorLayer())
    {
        stateset->setDefine("OE_GROUNDCOVER_COLOR_SAMPLER", getColorLayer()->getSharedTextureUniformName());
        stateset->setDefine("OE_GROUNDCOVER_COLOR_MATRIX", getColorLayer()->getSharedTextureMatrixUniformName());
        stateset->addUniform(new osg::Uniform("oe_GroundCover_colorMinSaturation", options().colorMinSaturation().get()));
    }
    else
    {
        stateset->removeDefine("OE_GROUNDCOVER_COLOR_SAMPLER");
        stateset->removeDefine("OE_GROUNDCOVER_COLOR_MATRIX");
        stateset->removeUniform("oe_GroundCover_colorMinSaturation");
    }

    // disable backface culling to support shadow/depth cameras,
    // for which the geometry shader renders cross hatches instead of billboards.
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::PROTECTED);

    //stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA));

    stateset->addUniform(new osg::Uniform("oe_gc_maxAlpha", getMaxAlpha()));

    if (!_sseU.valid())
        _sseU = new osg::Uniform("oe_gc_sse", getMaxSSE());

    stateset->addUniform(_sseU.get());

    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
        stateset->setMode(GL_MULTISAMPLE, 1);
    else
        stateset->removeMode(GL_MULTISAMPLE);

    // Install the land cover shaders on the state set
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->setName("GroundCover");
    vp->addGLSLExtension("GL_ARB_gpu_shader_int64");

    // Load shaders particular to this class
    loadRenderingShaders(vp, getReadOptions());

    // Assemble zone-specific statesets:
    optional<float> maxVisibleRange = options().maxVisibleRange();

    _zoneStateSets.clear();
    for(unsigned z = 0; z < getZones().size(); ++z)
    {
        osg::StateSet* zoneStateSet = new osg::StateSet();
        zoneStateSet->addUniform(new osg::Uniform("oe_gc_zone", (int)z));

        // store the layout on a per-zone basis since the instancer will be different.
        const BiomeZone& zone = getZones()[z];
        zoneStateSet->setAttribute(new ZoneSA(&const_cast<BiomeZone&>(zone)));

        if (zone.options().maxDistance().isSet())
        {
            maxVisibleRange = osg::minimum(maxVisibleRange.get(), zone.options().maxDistance().get());
        }

        // keep in a vector so the ZoneSelector can pick one at cull time
        _zoneStateSets.push_back(zoneStateSet);
    }

    if (maxVisibleRange.isSet())
    {
        _options->maxVisibleRange().setDefault(maxVisibleRange.get());
    }
}

void
GroundCoverLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    for(std::vector<osg::ref_ptr<osg::StateSet> >::iterator i = _zoneStateSets.begin();
        i != _zoneStateSets.end();
        ++i)
    {
        i->get()->resizeGLObjectBuffers(maxSize);
    }

    if (_renderer.valid())
    {
        _renderer->resizeGLObjectBuffers(maxSize);
    }

    PatchLayer::resizeGLObjectBuffers(maxSize);
}

void
GroundCoverLayer::releaseGLObjects(osg::State* state) const
{
    for(std::vector<osg::ref_ptr<osg::StateSet> >::const_iterator i = _zoneStateSets.begin();
        i != _zoneStateSets.end();
        ++i)
    {
        i->get()->releaseGLObjects(state);
    }

    if (_renderer.valid())
    {
        _renderer->releaseGLObjects(state);
    }

    PatchLayer::releaseGLObjects(state);
}

namespace
{
    osg::Node* makeBBox(const osg::BoundingBox& bbox, const TileKey& key)
    {
        osg::Group* geode = new osg::Group();

        if ( bbox.valid() )
        {
            static const int index[24] = {
                0,1, 1,3, 3,2, 2,0,
                0,4, 1,5, 2,6, 3,7,
                4,5, 5,7, 7,6, 6,4
            };

            LineDrawable* lines = new LineDrawable(GL_LINES);            
            for(int i=0; i<24; i+=2)
            {
                lines->pushVertex(bbox.corner(index[i]));
                lines->pushVertex(bbox.corner(index[i+1]));
            }
            lines->setColor(osg::Vec4(1,0,0,1));
            lines->finish();

            geode->addChild(lines);
        }

        return geode;
    }

    osg::Geometry* makeShape()
    {
        osg::Geometry* geom = new osg::Geometry();
        geom->setUseVertexBufferObjects(true);
        geom->setUseDisplayList(false);

        const float s=10;

        osg::Vec3Array* v = new osg::Vec3Array();
        v->reserve(3);
        v->push_back(osg::Vec3(-s/2, 0, s)); // left
        v->push_back(osg::Vec3(+s/2, 0, s)); // right
        v->push_back(osg::Vec3( 0, 0, 0)); // bottom
        geom->setVertexArray(v);

        osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_OVERALL);
        c->push_back(osg::Vec4(1,1,1,1));
        geom->setColorArray(c);

        osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
        normals->push_back(osg::Vec3f(0, 0, 1));
        geom->setNormalArray(normals);

        osg::Vec2Array* t = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);
        t->push_back(osg::Vec2(0, 1)); // left
        t->push_back(osg::Vec2(1, 1)); // right
        t->push_back(osg::Vec2(.5, 0)); // bottom
        geom->setTexCoordArray(3, t);

        osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
        b->reserve(3);
        b->push_back(0); b->push_back(1); b->push_back(2);
        geom->addPrimitiveSet(b);

        return geom;
    }
}

osg::Node*
GroundCoverLayer::createNodeImplementation(const DrawContext& dc)
{
    osg::Node* node = NULL;
    if (_debug)
        node = makeBBox(*dc._tileBBox, *dc._key);
    return node;
}

osg::Geometry*
GroundCoverLayer::createParametricGeometry() const    
{
    // Billboard geometry
    const unsigned vertsPerInstance = 8;
    const unsigned indiciesPerInstance = 12;

    osg::Geometry* out_geom = new osg::Geometry();
    out_geom->setUseVertexBufferObjects(true);
    out_geom->setUseDisplayList(false);

    out_geom->setVertexArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 8));

    static const GLushort indices[12] = { 0,1,2,2,1,3, 4,5,6,6,5,7 };
    out_geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 12, &indices[0]));

    return out_geom;
}

//........................................................................

GroundCoverLayer::TileManager::TileManager() :
    _highestOccupiedSlot(-1)
{
    //nop
}

void
GroundCoverLayer::TileManager::reset()
{
    for(auto& i : _current)
    {
        i._revision = -1;
        i._dirty = true;
        i._expired = false;
    }

    _highestOccupiedSlot = -1;

    _new.clear();
}

int
GroundCoverLayer::TileManager::allocate(const TileKey& key, int revision)
{
    int slot = -1;
    for(unsigned i=0; i<_current.size(); ++i)
    {
        if (_current[i]._revision < 0)
        {
            slot = i;
            break;
        }
    }

    if (slot < 0)
    {
        slot = _current.size();
        _current.resize(_current.size()+1);
    }

    _highestOccupiedSlot = osg::maximum(_highestOccupiedSlot, slot);

    _current[slot]._key = key;
    _current[slot]._revision = revision;
    _current[slot]._expired = false;
    _current[slot]._dirty = true;

    return slot;
}

int
GroundCoverLayer::TileManager::release(const TileKey& key)
{
    int slot = getSlot(key);
    if (slot >= 0)
    {
        _current[slot]._revision = -1;

        if (_highestOccupiedSlot == slot)
        {
            for(int s=_current.size()-1; s >= 0; --s)
            {
                if (_current[s]._revision >= 0)
                {
                    _highestOccupiedSlot = s;
                    break;
                }
            }
        }
    }
    return slot;
}

void
GroundCoverLayer::TileManager::release(int slot)
{
    if (slot >= 0 && slot < _current.size())
    {
        _current[slot]._revision = -1;
        _current[slot]._expired = false;
    }
}

bool
GroundCoverLayer::TileManager::inUse(int slot) const
{
    return slot < _current.size() && _current[slot]._revision >= 0;
}

int
GroundCoverLayer::TileManager::getSlot(const TileKey& key) const
{
    int slot = -1;
    for(int i=0; i<_current.size(); ++i)
    {
        if (_current[i]._revision >= 0 && _current[i]._key == key)
        {
            slot = i;
            break;
        }
    }
    return slot;
}

#define PASS_COLLECT 0
#define PASS_GENERATE 1
#define PASS_CULL 2
#define PASS_DRAW 3

GroundCoverLayer::Renderer::PCPState::PCPState()
{
    // initialize all the uniform locations - we will fetch these at draw time
    // when the program is active
    _generateDataUL = -1;
    _A2CUL = -1;
}

GroundCoverLayer::Renderer::Renderer(GroundCoverLayer* layer)
{
    _layer = layer;

    // create uniform IDs for each of our uniforms
    _A2CName = osg::Uniform::getNameID("oe_gc_useAlphaToCoverage");
    _computeDataUName = osg::Uniform::getNameID("oe_tile");

    _tileWidth = 0.0;

    _a2cBlending = new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO);

    // Load our compute shader
    GroundCoverShaders shaders;

    std::string computeSource = ShaderLoader::load(shaders.GroundCover_CS, shaders, layer->getReadOptions());
    _computeSS = new osg::StateSet();
    _computeProgram = new osg::Program();
    osg::Shader* computeShader = new osg::Shader(osg::Shader::COMPUTE, computeSource);
    computeShader->setName(shaders.GroundCover_CS);
    _computeProgram->addShader(computeShader);
    _computeSS->setAttribute(_computeProgram, osg::StateAttribute::ON);
}

GroundCoverLayer::Renderer::~Renderer()
{
    releaseGLObjects(0);
}

namespace 
{
    inline bool equivalent(const osg::Matrixf& a, const osg::Matrixf& b, bool epsilon)
    {
        const float* aptr = a.ptr();
        const float* bptr = b.ptr();
        for(int i=0; i<16; ++i) {
            if (!osg::equivalent(*aptr++, *bptr++, epsilon))
                return false;
        }
        return true;
    }
}

//#define DEVEL

void
GroundCoverLayer::Renderer::visitTileBatch(osg::RenderInfo& ri, const PatchLayer::TileBatch* tiles)
{
    OE_PROFILING_ZONE_NAMED("GroundCover DrawTileBatch");
    OE_PROFILING_GPU_ZONE("GroundCover DrawTileBatch");

    osg::State* state = ri.getState();
    CameraState& ds = _cameraState.get(ri.getCurrentCamera());

    ds._renderer = this;

    const ZoneSA* sa = ZoneSA::extract(state);
    const BiomeZone* zone = sa->_obj;
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[zone];

    // First time through we will need to create and set up the instancer
    // for this camera.
    if (!instancer.valid())
    {
        OE_PROFILING_ZONE_NAMED("IC Setup");
        instancer = new InstanceCloud();

        unsigned numInstances1D = 64u;

        if (zone->options().spacing().isSet())
        {
            float spacing_m = zone->options().spacing()->as(Units::METERS);
            numInstances1D = _tileWidth / spacing_m;
            _spacing = spacing_m;
        }

        instancer->setGeometryCloud(_geomCloud.get());
        instancer->setNumInstancesPerTile(numInstances1D, numInstances1D);
    }

    // If the zone changed, we need to re-generate ALL tiles
    bool needsGenerate = false;
    bool needsReset = false;

    if (sa != ds._previousZoneSA)
    {
        needsGenerate = true;
        needsReset = true;
        ds._previousZoneSA = sa;
    }

    // If the tile batch changed, we need to re-generate SOME tiles
    if (ds._lastTileBatchID != tiles->getBatchID())
    {
        ds._lastTileBatchID = tiles->getBatchID();
        needsGenerate = true;
    }

    // not a bug. Do this as a 32-bit matrix to avoid wierd micro-precision changes.
    // NOTE: this can cause jittering in the tree positions when you zoom :(
    bool needsCull = needsGenerate;
    if (!needsCull)
    {
        osg::Matrixf mvp = state->getModelViewMatrix() * state->getProjectionMatrix();
        if (mvp != ds._lastMVP)
        {
            ds._lastMVP = mvp;
            needsCull = true;
        }
    }

    // I'm not sure why we have to push the layer's stateset here.
    // It should have been applied already in the render bin.
    // I am missing something. -gw 4/20/20
    state->pushStateSet(_layer->getStateSet());

    // Ensure we have allocated sufficient GPU memory for the tiles:
    if (needsGenerate)
    {
        OE_PROFILING_ZONE_NAMED("allocateGLObjects");

        // returns true if new memory was allocated
        if (instancer->allocateGLObjects(ri, tiles->size()) ||
            needsReset)
        {
            ds._tiles.reset();
        }
    }

    if (needsGenerate || needsCull)
    {
        state->apply(_computeSS.get()); // activate compute program
    }

    instancer->newFrame();

    if (needsGenerate)
    {
        int slot;

        // First we run a COLLECT pass. This determines which tiles are new,
        // which already exist in the instancer, and which went away and
        // need to be recycled.
        ds._pass = PASS_COLLECT;
        {
            OE_PROFILING_ZONE_NAMED("Collect");

            // Put all tiles on the expired list, only removing them when we
            // determine that they are good to keep around.
            for(auto& i : ds._tiles._current)
                if (i._revision >= 0)
                    i._expired = true;

            // traverse and build our gen list.
            tiles->visitTiles(ri);

            // Release anything still marked as expired.
            for(slot=0; slot<ds._tiles._current.size(); ++slot)
            {
                if (ds._tiles._current[slot]._expired)
                {
                    ds._tiles.release(slot);
                }
            }

            // Allocate a slot for each new tile. We do this now AFTER
            // we have released any expired tiles
            for(const auto& i : ds._tiles._new)
            {
                slot = ds._tiles.allocate(i._key, i._revision);
            }
            ds._tiles._new.clear();
        }

        ds._pass = PASS_GENERATE;
        {
            OE_PROFILING_ZONE_NAMED("Generate");
            OE_PROFILING_GPU_ZONE("IC:Generate");

            ds._numTilesGenerated = 0u;

            instancer->setHighestTileSlot(ds._tiles._highestOccupiedSlot);

            for (slot = 0; slot <= ds._tiles._current.size(); ++slot)
            {
                instancer->setTileActive(slot, ds._tiles.inUse(slot));
            }

            instancer->generate_begin(ri);
            tiles->drawTiles(ri);
            instancer->generate_end(ri);
        }

#ifdef DEVEL
        OE_INFO << "-----" << std::endl;
        OE_INFO << "Generated " << ds._numTilesGenerated << "/" << tiles->size() << " tiles" << std::endl;

        OE_INFO << "Tiles:"<<std::endl;
        for(slot=0; slot<ds._tiles._current.size(); ++slot)
        {
            const TileGenInfo& i = ds._tiles._current[slot];
            if (i._revision >= 0)
            {
                OE_INFO 
                    << "   " << slot
                    << ": " << i._key.str()
                    << ", r=" << i._revision
                    << std::endl;
            }
            else
            {
                OE_INFO 
                    << "   " << slot
                    << ": -"
                    << std::endl;
            }
        }
#endif
    }

    if (needsCull)
    {
        OE_PROFILING_ZONE_NAMED("Cull/Sort");

        // per frame cull/sort:
        ds._pass = PASS_CULL;
        
        tiles->visitTiles(ri); // collect and upload tile matrix data

        instancer->cull(ri); // cull and sort
    }

    // If we ran a compute shader, we replaced the state and 
    // now need to re-apply it before rendering.
    if (needsGenerate || needsCull)
    {
        OE_PROFILING_ZONE_NAMED("State reset");
        state->apply();
    }

    // draw pass:
    {
        OE_PROFILING_ZONE_NAMED("Draw");

        ds._pass = PASS_DRAW;
        applyLocalState(ri, ds);
        instancer->draw(ri);
    }

    // pop the layer's stateset.
    state->popStateSet();

    // Clean up and finish
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    // Need to unbind our VAO so as not to confuse OSG
    ri.getState()->unbindVertexArrayObject();

    //TODO: review this. I don't see why this should be necessary.
    ri.getState()->setLastAppliedProgramObject(NULL);
#endif
}

void
GroundCoverLayer::Renderer::applyLocalState(osg::RenderInfo& ri, CameraState& ds)
{
    if (ds._pass == PASS_DRAW)
    {
        const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
        if (!pcp)
            return;

        osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);

        GLint useA2C = 0;
        if (_layer->getUseAlphaToCoverage())
        {
            useA2C = ri.getState()->getLastAppliedMode(GL_MULTISAMPLE) ? 1 : 0;
            ri.getState()->applyMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, useA2C == 1);
            ri.getState()->applyAttribute(_a2cBlending.get());
        }

        PCPState& u = ds._pcpState[pcp];

        if (u._A2CUL < 0)
            u._A2CUL = pcp->getUniformLocation(_A2CName);

        if (u._A2CUL >= 0)
            ext->glUniform1i(u._A2CUL, useA2C);
    }
}

void
GroundCoverLayer::Renderer::visitTile(osg::RenderInfo& ri, const PatchLayer::DrawContext& tile)
{
    // make sure we don't have a shader error or NULL pcp
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp)
        return;

    CameraState& ds = _cameraState.get(ri.getCurrentCamera());

    // Find our instancer:
    const ZoneSA* sa = ZoneSA::extract(ri.getState());
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[sa->_obj];

    if (ds._pass == PASS_COLLECT)
    {
        // Decide whether this tile really needs regen:
        int slot = ds._tiles.getSlot(*tile._key);

        if (slot < 0) // new tile.
        {
            ds._tiles._new.push_back(TileGenInfo());
            TileGenInfo& newTile = ds._tiles._new.back();
            newTile._key = *tile._key;
            newTile._revision = tile._revision;
            // will allocate a slot later, after we see if anybody freed one.
            //OE_INFO << "Greetings, " << tile._key->str() << ", r=" << tile._revision << std::endl;
        }

        else
        {
            TileGenInfo& i = ds._tiles._current[slot];

            // Keep it around.
            i._expired = false;

            if (i._revision != tile._revision)
            {
                // revision changed! Queue it up for regeneration.
                i._revision = tile._revision;
                i._dirty = true;
            }
        }
    }

    else if (ds._pass == PASS_GENERATE)
    {
        int slot = ds._tiles.getSlot(*tile._key);
        if (slot >= 0)
        {
            TileGenInfo& i = ds._tiles._current[slot];

            if (i._dirty == true)
            {
                i._dirty = false;

                osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);
                PCPState& u = ds._pcpState[pcp];

                if (u._generateDataUL < 0)
                    u._generateDataUL = pcp->getUniformLocation(_computeDataUName);

                if (u._generateDataUL >= 0)
                {
                    u._generateData[0] = tile._tileBBox->xMin();
                    u._generateData[1] = tile._tileBBox->yMin();
                    u._generateData[2] = tile._tileBBox->xMax();
                    u._generateData[3] = tile._tileBBox->yMax();

                    u._generateData[4] = (float)slot;

                    // TODO: check whether this changed before calling it
                    ext->glUniform1fv(u._generateDataUL, 5, &u._generateData[0]);

                    //OE_INFO << "Gen: " << tile._key->str() << ", slot=" << slot << std::endl;

                    instancer->generate_tile(slot, ri);
                    
                    ++ds._numTilesGenerated;
                }
            }
        }
    }

    else if (ds._pass == PASS_CULL)
    {
        int slot = ds._tiles.getSlot(*tile._key);
        if (slot >= 0)
        {
            instancer->setMatrix(slot, *tile._modelViewMatrix);
        }
        else
        {
            OE_WARN << "Internal error -- CULL should not see an inactive tile" << std::endl;
        }
    }

    else // if (ds._pass == PASS_DRAW)
    {
        // NOP
        OE_INFO << "Should not be here." << std::endl;
    }
}

void
GroundCoverLayer::Renderer::resizeGLObjectBuffers(unsigned maxSize)
{
    //_drawStateBuffer.resize(osg::maximum(maxSize, _drawStateBuffer.size()));
}

void
GroundCoverLayer::Renderer::CameraStateRGLO::operator()(
    const GroundCoverLayer::Renderer::CameraState& ds) const
{
    for (InstancerPerZone::const_iterator j = ds._instancers.begin();
        j != ds._instancers.end();
        ++j)
    {
        InstanceCloud* instancer = j->second.get();
        if (instancer)
            instancer->releaseGLObjects(_state);
    }
}

void
GroundCoverLayer::Renderer::releaseGLObjects(osg::State* state) const
{
    _cameraState.forEach(CameraStateRGLO(state));

    if (_texArena.valid())
    {
        _texArena->releaseGLObjects(state);
    }
}

void
GroundCoverLayer::loadRenderingShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders shaders;
    shaders.load(vp, shaders.GroundCover_Render);
}

namespace {
    struct ModelCacheEntry {
        osg::ref_ptr<osg::Node> _node;
        int _modelID;
    };
}

void
GroundCoverLayer::loadAssets(TextureArena* arena)
{
    OE_INFO << LC << "Loading assets ..." << std::endl;

    typedef std::map<URI, osg::ref_ptr<AssetData> > TextureShareCache;
    TextureShareCache texcache;

    typedef std::map<URI, ModelCacheEntry> ModelCache;
    ModelCache modelcache;

    int landCoverGroupIndex = 0;
    int assetIDGen = 0;
    int modelIDGen = 0;

    // all the zones (these will later be called "biomes")
    for(int z=0; z<getZones().size(); ++z)
    {
        // each zone has a single layout (used to be called "groundcover")
        const BiomeZone& zone = getZones()[z];

        // each layout has one or more groupings of land cover classes
        // (this used to be called a biome)
        for(int j=0; j<zone.getLandCoverGroups().size(); ++j, ++landCoverGroupIndex)
        {
            const LandCoverGroup& group = zone.getLandCoverGroups()[j];

            // parse the land cover codes:
            const std::vector<std::string>& classNames = group.getLandCoverClassNames();
            if (classNames.empty())
            {
                OE_WARN << LC << "Skipping a land cover group because it has no classes" << std::endl;
                continue;
            }

            std::vector<int> codes;
            for(unsigned c=0; c<classNames.size(); ++c)
            {
                const LandCoverClass* lcclass = getLandCoverDictionary()->getClassByName(classNames[c]);
                if (lcclass)
                    codes.push_back(lcclass->getValue());
            }

            // Each grouping points to multiple assets (used to be "billboards")
            int uid = 0;
            for(int k=0; k<group.getAssets().size(); ++k)
            {
                const AssetUsage& asset = group.getAssets()[k];

                osg::ref_ptr<AssetData> data = new AssetData();
                data->_zoneIndex = z;
                data->_zone = &zone;
                data->_landCoverGroupIndex = landCoverGroupIndex;
                data->_landCoverGroup = &group;
                data->_asset = &asset;
                data->_numInstances = 0;
                data->_codes = codes;
                data->_modelID = -1;
                data->_assetID = -1;
                data->_sideBillboardTexIndex = -1;
                data->_topBillboardTexIndex = -1;
                data->_modelTexIndex = -1;

                if (asset.options().modelURI().isSet())
                {
                    const URI& uri = asset.options().modelURI().get();
                    ModelCache::iterator ic = modelcache.find(uri);
                    if (ic != modelcache.end())
                    {
                        data->_model = ic->second._node.get();
                        data->_modelID = ic->second._modelID;
                    }
                    else
                    {
                        data->_model = uri.getNode(getReadOptions());
                        if (data->_model.valid())
                        {
                            data->_modelID = modelIDGen++;
                            modelcache[uri]._node = data->_model.get();
                            modelcache[uri]._modelID = data->_modelID;

                            osg::ComputeBoundsVisitor cbv;
                            data->_model->accept(cbv);
                            data->_modelAABB = cbv.getBoundingBox();

                            //OE_INFO << LC << "Model " << uri.base() 
                            //    << " : X=" << (data->_modelAABB.xMin()) << " " << data->_modelAABB.xMax()
                            //    << " , Y=" << (data->_modelAABB.yMin()) << " " << data->_modelAABB.yMax()
                            //    << " , Z=" << (data->_modelAABB.zMin()) << " " << data->_modelAABB.zMax()
                            //    << std::endl;                                
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load model from \"" << uri.full() << "\"" << std::endl;
                        }
                    }
                }

                if (asset.options().sideBillboardURI().isSet())
                {
                    const URI& uri = asset.options().sideBillboardURI().get();

                    auto ic = texcache.find(uri);
                    if (ic != texcache.end())
                    {
                        data->_sideBillboardTex = ic->second->_sideBillboardTex.get();
                        data->_sideBillboardTexIndex = ic->second->_sideBillboardTexIndex;
                    }
                    else
                    {
                        data->_sideBillboardTex = new Texture();
                        data->_sideBillboardTex->_uri = uri;
                        data->_sideBillboardTex->_image = uri.getImage(getReadOptions());
                        //TODO: check for errors
                        if (true)
                        {
                            texcache[uri] = data.get();
                            data->_sideBillboardTexIndex = arena->size();
                            arena->add(data->_sideBillboardTex.get());
                        }
                    }
                }

                if (asset.options().topBillboardURI().isSet())
                {
                    const URI& uri = asset.options().topBillboardURI().get();

                    auto ic = texcache.find(uri);
                    if (ic != texcache.end())
                    {
                        data->_topBillboardTex = ic->second->_topBillboardTex;
                        data->_topBillboardTexIndex = ic->second->_topBillboardTexIndex;
                    }
                    else
                    {
                        data->_topBillboardTex = new Texture();
                        data->_topBillboardTex->_uri = uri;
                        data->_topBillboardTex->_image = uri.getImage(getReadOptions());
                        //TODO: check for errors
                        if (true)
                        {
                            texcache[uri] = data.get();
                            data->_topBillboardTexIndex = arena->size();
                            arena->add(data->_topBillboardTex.get());
                        }
                    }
                }

                if (data->_sideBillboardTex.valid() || data->_model.valid())
                {
                    data->_assetID = assetIDGen++;
                    _liveAssets.push_back(data.get());
                }
            }
        }
    }

    if (_liveAssets.empty())
    {
        OE_WARN << LC << "Failed to load any assets!" << std::endl;
        // TODO: something?
    }
    else
    {
        OE_INFO << LC << "Loaded " << _liveAssets.size() << " assets." << std::endl;
    }
}

osg::Shader*
GroundCoverLayer::createLUTShader() const
{
    std::stringstream landCoverGroupBuf;
    landCoverGroupBuf << std::fixed << std::setprecision(2);

    std::stringstream assetBuf;
    assetBuf << std::fixed << std::setprecision(1);

    int numAssetInstancesAdded = 0;
    int numLandCoverGroupsAdded = 0;
    int currentLandCoverGroupIndex = -1;
    const LandCoverGroup* currentLandCoverGroup = NULL;
    int startingAssetIndex = 0;
    int numAssetsInLandCoverGroup = 0;
    float maxWidth = -1.0f, maxHeight = -1.0f;
    AssetData* data = NULL;

    for(int a=0; a<_liveAssets.size(); ++a)
    {
        data = _liveAssets[a].get();

        // initialize pointer:
        if (currentLandCoverGroupIndex < 0)
        {
            currentLandCoverGroupIndex = data->_landCoverGroupIndex;
            currentLandCoverGroup = data->_landCoverGroup;
        }

        // close out a group:
        if (currentLandCoverGroupIndex != data->_landCoverGroupIndex)
        {
            float fill = currentLandCoverGroup->options().fill().getOrUse(
                data->_zone->options().fill().get());

            if (startingAssetIndex > 0)
                landCoverGroupBuf << ", \n";

            landCoverGroupBuf
                << "    oe_gc_LandCoverGroup("
                << startingAssetIndex << ", "
                << numAssetsInLandCoverGroup << ", " << fill << ")";

            ++numLandCoverGroupsAdded;

            startingAssetIndex = numAssetInstancesAdded;
            numAssetsInLandCoverGroup = 0;
            currentLandCoverGroupIndex = data->_landCoverGroupIndex;
            currentLandCoverGroup = data->_landCoverGroup;
        }

        // record an asset:
        float width = data->_asset->options().width().get();
        if (data->_asset->options().width().isSet() == false &&
            data->_modelAABB.valid())
        {
            width = osg::maximum(
                data->_modelAABB.xMax() - data->_modelAABB.xMin(),
                data->_modelAABB.yMax() - data->_modelAABB.yMin());
            width = data->_modelAABB.radius();
        }

        float height = data->_asset->options().height().get();
        if (data->_asset->options().height().isSet() == false &&
            data->_modelAABB.valid())
        {
            height = data->_modelAABB.zMax() - data->_modelAABB.zMin();
            height = data->_modelAABB.radius();
        }

        float sizeVariation = data->_asset->options().sizeVariation().getOrUse(
            currentLandCoverGroup->options().sizeVariation().get());

        int weight = (int)osg::maximum(data->_asset->options().selectionWeight().get(), 1.0f);

        maxWidth = osg::maximum(maxWidth, width + (width*sizeVariation));
        maxHeight = osg::maximum(maxHeight, height + (height*sizeVariation));

        // apply the selection weight by adding the object multiple times

        for(int w=0; w<weight; ++w)
        {
            if (numAssetInstancesAdded > 0)
                assetBuf << ", \n";

            assetBuf << "    oe_gc_Asset("
                << data->_assetID
                << ", " << data->_modelID
                //<< ", " << (data->_modelTex.valid() ? data->_modelTex->getHandle() : 0) << "UL"
                //<< ", " << (data->_sideBillboardTex.valid() ? data->_sideBillboardTex->getHandle() : 0) << "UL"
                //<< ", " << (data->_topBillboardTex.valid() ? data->_topBillboardTex->getHandle() : 0) << "UL"
                << ", " << (data->_modelTex.valid() ? data->_modelTexIndex : -1)
                << ", " << (data->_sideBillboardTex.valid() ? data->_sideBillboardTexIndex : -1)
                << ", " << (data->_topBillboardTex.valid() ? data->_topBillboardTexIndex : -1)
                << ", " << width
                << ", " << height
                << ", " << sizeVariation
                << ", " << data->_asset->options().fill().get()
                << ")";

            ++data->_numInstances;

            ++numAssetInstancesAdded;

            ++numAssetsInLandCoverGroup;
        }
    }

    // close out the last group:
    if (numAssetsInLandCoverGroup > 0 && data != NULL)
    {
        float fill = currentLandCoverGroup->options().fill().getOrUse(
            data->_zone->options().fill().get());

        if (startingAssetIndex > 0)
            landCoverGroupBuf << ", \n";

        landCoverGroupBuf << "    oe_gc_LandCoverGroup("
            << startingAssetIndex << ", "
            << numAssetsInLandCoverGroup
            << ", " << fill << ")";

        ++numLandCoverGroupsAdded;
    }


    std::stringstream landCoverGroupWrapperBuf;
    landCoverGroupWrapperBuf << 
        "struct oe_gc_LandCoverGroup { \n"
        "    int firstAssetIndex; \n"
        "    int numAssets; \n"
        "    float fill; \n"
        "}; \n"
        "const oe_gc_LandCoverGroup oe_gc_landCoverGroups[" << numLandCoverGroupsAdded << "] = oe_gc_LandCoverGroup[" << numLandCoverGroupsAdded << "]( \n"
        << landCoverGroupBuf.str()
        << "\n); \n";

    std::stringstream assetWrapperBuf;
    assetWrapperBuf <<
        "struct oe_gc_Asset { \n"
        "    int assetId; \n"
        "    int modelId; \n"
        "    int modelSamplerIndex; \n"
        "    int sideSamplerIndex; \n"
        "    int topSamplerIndex; \n"
        //"    uint64_t modelSampler; \n"
        //"    uint64_t sideSampler; \n"
        //"    uint64_t topSampler; \n"
        "    float width; \n"
        "    float height; \n"
        "    float sizeVariation; \n"
        "    float fill; \n"
        "}; \n"
        "const oe_gc_Asset oe_gc_assets[" << numAssetInstancesAdded << "] = oe_gc_Asset[" << numAssetInstancesAdded << "]( \n"
        << assetBuf.str()
        << "\n); \n";

    // next, create the master LUT.
    std::stringstream lutbuf;

    lutbuf << assetWrapperBuf.str() << "\n";

    lutbuf << landCoverGroupWrapperBuf.str() << "\n";

    lutbuf <<
        "bool oe_gc_getLandCoverGroup(in int zone, in int code, out oe_gc_LandCoverGroup result) { \n";

    UnorderedSet<std::string> exprs;
    std::stringstream exprBuf;

    for(int a=0; a<_liveAssets.size(); ++a)
    {
        AssetData* data = _liveAssets[a].get();

        // shouldn't happen, but check anyway
        if (data->_codes.empty())
            continue;

        exprBuf.str("");
        exprBuf << "  if ((zone==" << data->_zoneIndex << ") && (";
        for(int c = 0; c < data->_codes.size(); ++c)
        {
            if (c > 0) exprBuf << " || ";
            exprBuf << "(code==" << data->_codes[c] << ")";
        }
        exprBuf << ")) { result = oe_gc_landCoverGroups[" << data->_landCoverGroupIndex << "]; return true; } \n";

        std::string exprString = exprBuf.str();
        if (exprs.find(exprString) == exprs.end())
        {
            exprs.insert(exprString);
            lutbuf << exprString;
        }
    }

    lutbuf
        << "  return false; \n"
        << "} \n";

    lutbuf 
        << "bool oe_gc_getAsset(in int index, out oe_gc_Asset result) { result = oe_gc_assets[index]; return true; } \n";

    osg::Shader* shader = new osg::Shader(osg::Shader::COMPUTE);
    shader->setName( "GroundCover LUTs" );
    shader->setShaderSource(
        Stringify() 
        << "#version " GLSL_VERSION_STR "\n"
        << "#extension GL_ARB_gpu_shader_int64 : enable\n"
        << lutbuf.str() << "\n");

    return shader;
}

osg::StateSet*
GroundCoverLayer::getZoneStateSet(unsigned index) const
{
    return index < _zoneStateSets.size() ? _zoneStateSets[index].get() : NULL;
}

GeometryCloud*
GroundCoverLayer::createGeometryCloud(TextureArena* arena) const
{
    GeometryCloud* geomCloud = new GeometryCloud();

    // First entry is the parametric group. Any instance without a 3D model,
    // or that is out of model range, gets rendered as a parametric billboard.
    osg::Geometry* pg = createParametricGeometry();
    geomCloud->add( pg, pg->getVertexArray()->getNumElements() );

    // Add each 3D model to the geometry cloud and update the texture
    // atlas along the way.
    UnorderedMap<int, Texture*> visited;
    UnorderedMap<Texture*, int> arenaIndex;
    for(AssetDataVector::const_iterator a = _liveAssets.begin();
        a != _liveAssets.end();
        ++a)
    {
        AssetData* asset = a->get();
        if (asset->_modelID >= 0)
        {
            int atlasIndex = -1;
            auto i = visited.find(asset->_modelID);
            if (i == visited.end())
            {
                // adds the model, and returns the texture associated with the
                // FIRST discovered texture from the model.
                Texture* tex = geomCloud->add(asset->_model.get());
                visited[asset->_modelID] = tex;
                if (tex)
                {
                    asset->_modelTexIndex = arena->size();
                    arena->add(tex);
                    arenaIndex[tex] = asset->_modelTexIndex;
                }

                asset->_modelTex = tex;
            }
            else
            {
                asset->_modelTex = i->second;
                asset->_modelTexIndex = arenaIndex[i->second];
            }
        }
    }

    return geomCloud;
}
