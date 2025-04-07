/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "GroundCoverLayer"
#include "ProceduralShaders"
#include "NoiseTextureFactory"

#include <osgEarth/VirtualProgram>
#include <osgEarth/CameraUtils>
#include <osgEarth/Shaders>
#include <osgEarth/LineDrawable>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Math>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Metrics>

#include <osg/BlendFunc>
#include <osg/Multisample>
#include <osg/Texture2D>
#include <osg/Depth>
#include <osg/Version>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
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

using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(groundcover, GroundCoverLayer);
REGISTER_OSGEARTH_LAYER(splat_groundcover, GroundCoverLayer);

// TODO LIST
//

//  - Move normal map conversion code out of here, and move towards a "MaterialTextureSet"
//    kind of setup that will support N material textures at once.

//  - FEATURE: automatically generate billboards? Imposters? Other?

//  - Texture management as the catalog gets bigger. Swap in/out criteria and detection??
//    (This will probably have to wait until we change the Biome meta.)
//    OR, the GPU can periodically "report" on usage in a readback buffer and the CPU
//    can respond accordingly; as long as the readback is fairly asynchronous

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
//  - variable spacing or clumping by landcovergroup or asset...?
//  - make the noise texture bindless as well? Stick it in the arena? Why not.

//  - (DONE) Lighting: do something about billboard lighting. We might have to generate normal 
//    maps when we make the imposter billboards (if we make them).
//  - (DONE) FEATURE: FADE in 3D models from billboards
//  - (DONE) BUG: multiple biomes, same asset in each biome; billboard sizes are messed up.
//  - (DONE - using indexes instead of copies) Reduce the size of the RenderBuffer structure
//  - (DONE) Fix model culling. The "radius" isn't quite sufficient since the origin is not at the center,
//    AND because rotation changes the profile. Calculate it differently.
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
    biomeLayer().set(conf, "biomes_layer");
    conf.set("group", group());
    conf.set("color_min_saturation", colorMinSaturation());
    conf.set("lod", _lod);
    conf.set("cast_shadows", _castShadows);
    conf.set("max_alpha", maxAlpha());
    conf.set("alpha_to_coverage", alphaToCoverage());
    conf.set("max_sse", maxSSE());
    conf.set("spacing", spacing());
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
    spacing().setDefault(Distance(20.0, Units::METERS));

    maskLayer().get(conf, "mask_layer");
    colorLayer().get(conf, "color_layer");
    biomeLayer().get(conf, "biomes_layer");

    conf.get("group", group());
    conf.get("color_min_saturation", colorMinSaturation());
    conf.get("lod", _lod);
    conf.get("cast_shadows", _castShadows);
    conf.get("max_alpha", maxAlpha());
    conf.get("alpha_to_coverage", alphaToCoverage());
    conf.get("max_sse", maxSSE());
    conf.get("spacing", spacing());
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

void GroundCoverLayer::setLOD(unsigned value) {
    options().lod() = value;
}
unsigned GroundCoverLayer::getLOD() const {
    return options().lod().get();
}

void GroundCoverLayer::setSpacing(const Distance& value) {
    options().spacing() = value;
}
const Distance& GroundCoverLayer::getSpacing() const {
    return options().spacing().get();
}

void GroundCoverLayer::setCastShadows(bool value) {
    options().castShadows() = value;
}
bool GroundCoverLayer::getCastShadows() const {
    return options().castShadows().get();
}

void GroundCoverLayer::setMaxSSE(float value) {
    options().maxSSE() = value;
    if (_sseU.valid())
        _sseU->set(value);
}
float GroundCoverLayer::getMaxSSE() const {
    return options().maxSSE().get();
}

void GroundCoverLayer::setAssetGroupName(const std::string& value) {
    options().group() = value;
}
const std::string& GroundCoverLayer::getAssetGroupName() const {
    return options().group().get();
}

void
GroundCoverLayer::init()
{
    PatchLayer::init();

    setAcceptCallback(new LayerAcceptor(this));

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

    return PatchLayer::openImplementation();
}

Status
GroundCoverLayer::closeImplementation()
{
    releaseGLObjects(nullptr);

    setDrawCallback(nullptr);
    _renderer = nullptr;
    _rendererSetup.abandon();

    return PatchLayer::closeImplementation();
}

void
GroundCoverLayer::update(osg::NodeVisitor& nv)
{
    // check to see whether the asset loading job has completed
    if (getStatus().isOK() && 
        !_renderer.valid() &&
        _rendererSetup.isAvailable())
    {
        _renderer = _rendererSetup.release();

        if (_renderer.valid())
        {
            if (_renderer->_status.isError())
            {
                setStatus(_renderer->_status);
            }
            else
            {
                OE_INFO << LC << "Renderer is ready." << std::endl;

                // install it as a patch layer draw callback
                setDrawCallback(_renderer.get());

                // rebuild the state
                buildStateSets();
            }
        }
    }
}

void
GroundCoverLayer::setBiomeLayer(BiomeLayer* layer)
{
    _biomeLayer.setLayer(layer);

    if (layer && _renderer.valid())
    {
        buildStateSets();
    }
}

BiomeLayer*
GroundCoverLayer::getBiomeLayer() const
{
    return _biomeLayer.getLayer();
}

void
GroundCoverLayer::setLifeMapLayer(LifeMapLayer* layer)
{
    _lifeMapLayer.setLayer(layer);
    if (layer)
    {
        buildStateSets();
    }
}

LifeMapLayer*
GroundCoverLayer::getLifeMapLayer() const
{
    return _lifeMapLayer.getLayer();
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

    if (!getLifeMapLayer())
        setLifeMapLayer(map->getLayer<LifeMapLayer>());

    if (!getBiomeLayer())
        setBiomeLayer(map->getLayer<BiomeLayer>());

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

    if (getBiomeLayer() == nullptr)
    {
        setStatus(Status::ResourceUnavailable, "No Biomes layer available in the Map");
        return;
    }

    if (getLifeMapLayer() == nullptr)
    {
        setStatus(Status::ResourceUnavailable, "No LifeMap available in the Map");
        return;
    }
}

void
GroundCoverLayer::removedFromMap(const Map* map)
{
    PatchLayer::removedFromMap(map);

    options().maskLayer().removedFromMap(map);
    options().colorLayer().removedFromMap(map);
}

void
GroundCoverLayer::prepareForRendering(TerrainEngine* engine)
{
    PatchLayer::prepareForRendering(engine);

    TerrainResources* res = engine->getResources();
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

        // Initialize the renderer in the background.
        osg::observer_ptr<GroundCoverLayer> layer_obs(this);

        _rendererSetup = Job().dispatch<osg::ref_ptr<Renderer>>(
            [layer_obs](Cancelable* progress)
            {
                osg::ref_ptr<Renderer> result;

                osg::ref_ptr<GroundCoverLayer> layer;
                if (layer_obs.lock(layer))
                    result = new Renderer(layer.get());

                return result;
            }
        );
    }
}

void
GroundCoverLayer::buildStateSets()
{
    if (!_renderer.valid()) {
        OE_DEBUG << LC << "buildStateSets deferred.. renderer not yet created" << std::endl;
        return;
    }

    if (!_noiseBinding.valid()) {
        OE_DEBUG << LC << "buildStateSets deferred.. noise texture not yet bound" << std::endl;
        return;
    }

    if (!getBiomeLayer() && !getLifeMapLayer()) {
        OE_DEBUG << LC << "buildStateSets deferred.. land cover dictionary not available" << std::endl;
        return;
    }

    if (!options().lod().isSet()) {
        OE_DEBUG << LC << "buildStateSets deferred.. LOD not available" << std::endl;
        return;
    }

    // calculate the tile width based on the LOD:
    if (_mapProfile.valid())
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

    if (getLifeMapLayer())
    {
        stateset->setDefine("OE_LIFEMAP_SAMPLER", getLifeMapLayer()->getSharedTextureUniformName());
        stateset->setDefine("OE_LIFEMAP_MATRIX", getLifeMapLayer()->getSharedTextureMatrixUniformName());
    }
    else
    {
        stateset->removeDefine("OE_LIFEMAP_SAMPLER");
        stateset->removeDefine("OE_LIFEMAP_MATRIX");
    }

    if (getBiomeLayer())
    {
        stateset->setDefine("OE_BIOME_SAMPLER", getBiomeLayer()->getSharedTextureUniformName());
        stateset->setDefine("OE_BIOME_MATRIX", getBiomeLayer()->getSharedTextureMatrixUniformName());
    }
    else
    {
        stateset->removeDefine("OE_BIOME_SAMPLER");
        stateset->removeDefine("OE_BIOME_MATRIX");
    }

    // disable backface culling to support shadow/depth cameras,
    // for which the geometry shader renders cross hatches instead of billboards.
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::PROTECTED);

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

    // Binds the vertex attribute containing the texture handle LUT index
    // that we assigned in GeometryCloud::add
    vp->addBindAttribLocation("oe_gc_texArenaIndex", 6);
}

void
GroundCoverLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    if (_renderer.valid())
    {
        _renderer->resizeGLObjectBuffers(maxSize);
    }

    PatchLayer::resizeGLObjectBuffers(maxSize);
}

void
GroundCoverLayer::releaseGLObjects(osg::State* state) const
{
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
}

osg::Node*
GroundCoverLayer::createParametricGeometry(
    std::vector<osg::Texture*>& textures) const
{
    osg::Group* group = new osg::Group();
    osg::Geometry* geom[2];

    // one part if we only have side textures;
    // two parts if we also have top textures
    int parts = textures.size() > 2 ? 2 : 1;

    for (int i = 0; i < parts; ++i)
    {
        geom[i] = new osg::Geometry();
        geom[i]->setUseVertexBufferObjects(true);
        geom[i]->setUseDisplayList(false);

        static const GLushort indices[6] = { 0,1,2, 2,1,3 };
        geom[i]->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 6, &indices[0]));
        geom[i]->setVertexArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 4));

        osg::StateSet* ss = geom[i]->getOrCreateStateSet();
        if (i == 0)
        {
            if (textures.size() > 0)
                ss->setTextureAttribute(0, textures[0], 1); // side albedo
            if (textures.size() > 1)
                ss->setTextureAttribute(1, textures[1], 1); // side normal
        }
        else
        {
            if (textures.size() > 2)
                ss->setTextureAttribute(0, textures[2], 1); // top albedo
            if (textures.size() > 3)
                ss->setTextureAttribute(1, textures[3], 1); // top normal
        }
        group->addChild(geom[i]);
    }
    return group;
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
    _isMSUL = -1;
}

GroundCoverLayer::Renderer::Renderer(GroundCoverLayer* layer)
{
    _layer = layer;
    _biomeRevision = -1;

    // create uniform IDs for each of our uniforms
    _isMSUName = osg::Uniform::getNameID("oe_gc_isMultisampled");
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

    // make a 4-channel noise texture to use
    NoiseTextureFactory noise;
    _noiseTex = noise.create(256u, 4u);
}

GroundCoverLayer::Renderer::~Renderer()
{
    releaseGLObjects(nullptr);
}

bool
GroundCoverLayer::Renderer::checkForUpdates()
{
    BiomeManager& biomeMan = _layer->getBiomeLayer()->getBiomeManager();
    
    if (_biomeRevision.exchange(biomeMan.getRevision()) != biomeMan.getRevision())
    {
        // revision changed; start a new asset load.
        osg::ref_ptr<GroundCoverLayer> layer(_layer.get());
        std::string assetGroup(_layer->options().group().get());

        _geomCloudInProgress = Job().dispatch<osg::ref_ptr<GeometryCloud>>(
            [layer, assetGroup](Cancelable* c)
            {
                osg::ref_ptr<GeometryCloud> result;

                BiomeManager& biomeMan = layer->getBiomeLayer()->getBiomeManager();

                biomeMan.loadGroup(
                    assetGroup,
                    [&](std::vector<osg::Texture*>& textures) { 
                        return layer->createParametricGeometry(textures); },
                    layer->getReadOptions());

                result = biomeMan.createGeometryCloud(
                    ASSET_GROUP_TREES, //assetGroup,
                    nullptr);
                    
                return result;
            }
        );
    }

    else if (_geomCloudInProgress.isAvailable())
    {
        _geomCloud = _geomCloudInProgress.release();
        return true;
    }

    return false;
}

void
GroundCoverLayer::Renderer::visitTileBatch(osg::RenderInfo& ri, const PatchLayer::TileBatch* tiles)
{
    if (_status.isError())
        return;

    bool newDataAvailable = checkForUpdates();

    if (!_geomCloud.valid() || _geomCloud->empty())
        return;

    OE_PROFILING_ZONE_NAMED("GroundCover DrawTileBatch");
    OE_PROFILING_GPU_ZONE("GroundCover DrawTileBatch");

    osg::State* state = ri.getState();
    CameraState& ds = _cameraState.get(ri.getCurrentCamera());

    ds._renderer = this;

    // First time through we will need to create and set up the instancer
    // for this camera.
    if (!ds._instancer.valid())
    {
        OE_PROFILING_ZONE_NAMED("IC Setup");
        ds._instancer = new InstanceCloud();
        ds._lastTileBatchID = -1;
        ds._pcpState.clear();

        unsigned numInstances1D = 64u;

        float spacing_m = _layer->getSpacing().as(Units::METERS);
        numInstances1D = _tileWidth / spacing_m;
        _spacing = spacing_m;

        ds._instancer->setGeometryCloud(_geomCloud.get());
        ds._instancer->setNumInstancesPerTile(numInstances1D, numInstances1D);
    }

    else if (newDataAvailable)
    {
        ds._instancer->setGeometryCloud(_geomCloud.get());
    }

    // If the zone changed, we need to re-generate ALL tiles
    bool needsGenerate = newDataAvailable;
    bool needsReset = newDataAvailable;

    // Debug mode: regenerate every time.
    if (ds._renderer->_layer->_debug)
    {
        needsGenerate = true;
        needsReset = true;
    }

    // If the tile batch changed, we need to re-generate SOME tiles
    if (ds._lastTileBatchID != tiles->getBatchID())
    {
        ds._lastTileBatchID = tiles->getBatchID();
        needsGenerate = true;
    }

#if 1
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
#else
    bool needsCull = true;
#endif

    // I'm not sure why we have to push the layer's stateset here.
    // It should have been applied already in the render bin.
    // I am missing something. -gw 4/20/20
    state->pushStateSet(_layer->getStateSet());

    // Apply the geometry cloud's stateset, which contains the texture arena
    // and the GPU lookup tables.
    state->apply(_geomCloud->getStateSet());

    // Ensure we have allocated sufficient GPU memory for the tiles:
    if (needsGenerate)
    {
        OE_PROFILING_ZONE_NAMED("allocateGLObjects");

        // returns true if new memory was allocated
        if (ds._instancer->allocateGLObjects(ri, tiles->size()) ||
            needsReset)
        {
            ds._tiles.reset();
        }
    }

    if (needsGenerate || needsCull)
    {
        state->apply(_computeSS.get()); // activate compute program
    }

    ds._instancer->newFrame();

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

            ds._instancer->setHighestTileSlot(ds._tiles._highestOccupiedSlot);

            for (slot = 0; slot <= ds._tiles._current.size(); ++slot)
            {
                ds._instancer->setTileActive(slot, ds._tiles.inUse(slot));
            }

            ds._instancer->generate_begin(ri);
            tiles->drawTiles(ri);
            ds._instancer->generate_end(ri);
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

        ds._instancer->cull(ri); // cull and sort
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
        ds._instancer->draw(ri);

        ds._instancer->endFrame(ri);
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

        GLint isMultisampled = 0;
        isMultisampled = ri.getState()->getLastAppliedMode(GL_MULTISAMPLE) ? 1 : 0;
        if (_layer->getUseAlphaToCoverage())
        {
            ri.getState()->applyMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, isMultisampled == 1);
            ri.getState()->applyAttribute(_a2cBlending.get());
        }

        PCPState& u = ds._pcpState[pcp];

        if (u._isMSUL < 0)
            u._isMSUL = pcp->getUniformLocation(_isMSUName);

        if (u._isMSUL >= 0)
            ext->glUniform1i(u._isMSUL, isMultisampled);
    }
}

void
GroundCoverLayer::Renderer::visitTile(osg::RenderInfo& ri, const PatchLayer::DrawContext& tile)
{
    if (!_geomCloud.valid())
        return;

    // make sure we don't have a shader error or NULL pcp
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp)
        return;

    CameraState& ds = _cameraState.get(ri.getCurrentCamera());

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

                    ds._instancer->generate_tile(ri);
                    
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
            ds._instancer->setMatrix(slot, *tile._modelViewMatrix);
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
}

void
GroundCoverLayer::Renderer::CameraStateRGLO::operator()(
    const GroundCoverLayer::Renderer::CameraState& ds) const
{
    if (ds._instancer.valid())
    {
        ds._instancer->releaseGLObjects(_state);
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

    if (_geomCloud.valid())
    {
        _geomCloud->releaseGLObjects(state);
    }
}

const std::string&
GroundCoverLayer::Renderer::getName() const
{
    static std::string empty;
    return _layer.valid() ? _layer->getName() : empty;
}

void
GroundCoverLayer::loadRenderingShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders shaders;
    shaders.load(vp, shaders.GroundCover_Render);
}
