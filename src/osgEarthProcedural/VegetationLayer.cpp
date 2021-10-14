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
#include "VegetationLayer"
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

#define LC "[VegetationLayer] " << getName() << ": "

#define OE_DEVEL OE_DEBUG

//#define ATLAS_SAMPLER "oe_veg_atlas"
#define NOISE_SAMPLER "oe_veg_noiseTex"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(vegetation, VegetationLayer);

// TODO LIST
//

//  - BUG: Close/Open VegLayer doesn't work

//  - TODO: separate the cloud's LUTs and TextureArena into different statesets, since they
//    are never used together. This will skip binding all the LUTS when only rendering, and
//    will skip binding the TA when only computing.
//  - Move normal map conversion code out of here, and move towards a "MaterialTextureSet"
//    kind of setup that will support N material textures at once.
//  - FEATURE: automatically generate billboards? Imposters? Other?
//  - [IDEA] programmable SSE for models?
//  - Idea: include a "model range" or "max SSE" in the InstanceBuffer...?
//  - [PERF] thin out distant instances automatically in large tiles
//  - [PERF] cull by "horizon" .. e.g., the lower you are, the fewer distant trees...?
//  - Figure out exactly where some of this functionality goes -- VL or IC? For example the
//    TileManager stuff seems like it would go in IC?
//  - variable spacing or clumping by asset...?
//  - make the noise texture bindless as well? Stick it in the arena? Why not.

//  - (DONE) [BUG] biomelayer from cache will NOT generate biome update notifications 6/25/21
//  - (DONE) fix the random asset select with weighting...just not really working well.
//  - (DONE) Allow us to store key, slot, etc data in the actual TileContext coming from the 
//    PatchLayer. It is silly to have to do TileKey lookups and not be able to simple
//    iterate over the TileBatch.
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
//  - (DONE .. was a bad oe_veg_Assets setup in the LUT shader) Fix teh "flashing" bug :(
//  - (DONE .. merged at layer level) BUGFIX: we're merging the geometrycloud's stateset into the layer's stateset. Make sure that's kosher...?
//  - (DONE) Fix the GRASS LAYER
//  - (DONE) Rotation for models
//  - (DONE) Scaling of the 3D models to match the height/width....? or not? set from loaded model?

//........................................................................

namespace
{
    static const std::string s_assetGroupName[2] = {
        "trees",
        "undergrowth"
    };
}

//........................................................................

Config
VegetationLayer::Options::getConfig() const
{
    Config conf = PatchLayer::Options::getConfig();
    colorLayer().set(conf, "color_layer");
    biomeLayer().set(conf, "biomes_layer");

    conf.set("color_min_saturation", colorMinSaturation());
    conf.set("alpha_to_coverage", alphaToCoverage());
    conf.set("max_sse", maxSSE());

    //TODO: groups

    return conf;
}

void
VegetationLayer::Options::fromConfig(const Config& conf)
{
    // defaults:
    alphaToCoverage().setDefault(true);
    maxSSE().setDefault(150.0f);

    colorLayer().get(conf, "color_layer");
    biomeLayer().get(conf, "biomes_layer");

    conf.get("color_min_saturation", colorMinSaturation());
    conf.get("alpha_to_coverage", alphaToCoverage());
    conf.get("max_sse", maxSSE());

    // defaults for groups:
    groups().resize(NUM_ASSET_GROUPS);

    if (AssetGroup::TREES < NUM_ASSET_GROUPS)
    {
        groups()[AssetGroup::TREES].castShadows() = true;
        groups()[AssetGroup::TREES].maxRange() = 2500.0f;
        groups()[AssetGroup::TREES].lod() = 14;
        groups()[AssetGroup::TREES].spacing() = Distance(15.0f, Units::METERS);
        groups()[AssetGroup::TREES].maxAlpha() = 0.15f;
    }

    if (AssetGroup::UNDERGROWTH < NUM_ASSET_GROUPS)
    {
        groups()[AssetGroup::UNDERGROWTH].castShadows() = false;
        groups()[AssetGroup::UNDERGROWTH].maxRange() = 75.0f;
        groups()[AssetGroup::UNDERGROWTH].lod() = 19;
        groups()[AssetGroup::UNDERGROWTH].spacing() = Distance(1.0f, Units::METERS);
        groups()[AssetGroup::UNDERGROWTH].maxAlpha() = 0.75f;
    }

    ConfigSet groups_c = conf.child("groups").children();
    for (auto& group_c : groups_c)
    {
        int g = -1;
        std::string name;
        group_c.get("name", name);
        if (name == AssetGroup::name(AssetGroup::TREES))
            g = AssetGroup::TREES;
        else if (name == AssetGroup::name(AssetGroup::UNDERGROWTH)) 
            g = AssetGroup::UNDERGROWTH;

        if (g >= 0 && g < NUM_ASSET_GROUPS)
        {
            Group& group = groups()[g];
            group_c.get("spacing", group.spacing());
            group_c.get("max_range", group.maxRange());
            group_c.get("lod", group.lod());
            group_c.get("cast_shadows", group.castShadows());
            group_c.get("max_alpha", group.maxAlpha());
        }
    }
}

//........................................................................

bool
VegetationLayer::LayerAcceptor::acceptLayer(osg::NodeVisitor& nv, const osg::Camera* camera) const
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
VegetationLayer::LayerAcceptor::acceptKey(const TileKey& key) const
{
    return _layer->hasGroupAtLOD(key.getLOD());
}

//........................................................................

void VegetationLayer::setMaxSSE(float value)
{
    if (value != options().maxSSE().get())
    {
        options().maxSSE() = value;
        if (_sseU.valid())
            _sseU->set(value);
    }
}

float VegetationLayer::getMaxSSE() const
{
    return options().maxSSE().get();
}

//........................................................................

void
VegetationLayer::init()
{
    PatchLayer::init();

    setAcceptCallback(new LayerAcceptor(this));

    _debug = (::getenv("OSGEARTH_VEGETATION_DEBUG") != NULL);

    _forceGenerate = false;

    // evil
    //installDefaultOpacityShader();
}

VegetationLayer::~VegetationLayer()
{
    close();
}

Status
VegetationLayer::openImplementation()
{
    // GL version requirement
    if (Registry::capabilities().getGLSLVersion() < 4.6f)
    {
        return Status(Status::ResourceUnavailable, "Requires GL 4.6+");
    }

    // Clamp the layer's max visible range the maximum range of the farthest
    // asset group. This will minimize the number of tiles sent to the
    // renderer and improve performance. Doing this here (in open) so the
    // user can close a layer, adjust parameters, and re-open if desired
    float max_range = 0.0f;
    for (auto& group : options().groups())
    {
        max_range = std::max(max_range, group.maxRange().get());
    }
    max_range = std::min(max_range, getMaxVisibleRange());
    setMaxVisibleRange(max_range);

    return PatchLayer::openImplementation();
}

Status
VegetationLayer::closeImplementation()
{
    releaseGLObjects(nullptr);

    _renderer = nullptr;

    return PatchLayer::closeImplementation();
}

void
VegetationLayer::update(osg::NodeVisitor& nv)
{
    // this code will release memory after the layer's not been
    // used for a while.
    if (isOpen() && _renderer != nullptr)
    {
        int df = nv.getFrameStamp()->getFrameNumber() - _renderer->_lastVisit.getFrameNumber();
        double dt = nv.getFrameStamp()->getReferenceTime() - _renderer->_lastVisit.getReferenceTime();

        if (dt > 5.0 && df > 60)
        {
            releaseGLObjects(nullptr);

            _renderer->reset();

            if (getBiomeLayer())
                getBiomeLayer()->getBiomeManager().reset();

            OE_INFO << LC << "timed out for inactivity." << std::endl;
        }
    }
}

void
VegetationLayer::setBiomeLayer(BiomeLayer* layer)
{
    _biomeLayer.setLayer(layer);

    if (layer && _renderer)
    {
        buildStateSets();
    }
}

BiomeLayer*
VegetationLayer::getBiomeLayer() const
{
    return _biomeLayer.getLayer();
}

void
VegetationLayer::setLifeMapLayer(LifeMapLayer* layer)
{
    _lifeMapLayer.setLayer(layer);
    if (layer && _renderer)
    {
        buildStateSets();
    }
}

LifeMapLayer*
VegetationLayer::getLifeMapLayer() const
{
    return _lifeMapLayer.getLayer();
}

void
VegetationLayer::setColorLayer(ImageLayer* value)
{
    options().colorLayer().setLayer(value);
    if (value)
    {
        buildStateSets();
    }
}

ImageLayer*
VegetationLayer::getColorLayer() const
{
    return options().colorLayer().getLayer();
}

void
VegetationLayer::setUseAlphaToCoverage(bool value)
{
    options().alphaToCoverage() = value;
}

bool
VegetationLayer::getUseAlphaToCoverage() const
{
    return options().alphaToCoverage().get();
}

bool
VegetationLayer::getCastShadows() const
{
    for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
    {
        if (options().groups()[i].castShadows() == true)
            return true;
    }
    return false;
}

bool
VegetationLayer::hasGroupAtLOD(unsigned lod) const
{
    for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
    {
        if (options().groups()[i].lod() == lod)
            return true;
    }
    return false;
}

unsigned
VegetationLayer::getGroupLOD(AssetGroup::Type group) const
{
    if (group > 0 && group < NUM_ASSET_GROUPS)
        return options().group(group).lod().get();
    else
        return 0;
}
void
VegetationLayer::setMaxRange(AssetGroup::Type type, float value)
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);

    auto& group = options().group(type);
    group.maxRange() = value;
}

float
VegetationLayer::getMaxRange(AssetGroup::Type type) const
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);
    return options().group(type).maxRange().get();
}

void
VegetationLayer::addedToMap(const Map* map)
{
    PatchLayer::addedToMap(map);

    if (!getLifeMapLayer())
        setLifeMapLayer(map->getLayer<LifeMapLayer>());

    if (!getBiomeLayer())
        setBiomeLayer(map->getLayer<BiomeLayer>());

    options().colorLayer().addedToMap(map);

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
VegetationLayer::removedFromMap(const Map* map)
{
    PatchLayer::removedFromMap(map);

    options().colorLayer().removedFromMap(map);
}

void
VegetationLayer::prepareForRendering(TerrainEngine* engine)
{
    PatchLayer::prepareForRendering(engine);

    TerrainResources* res = engine->getResources();
    if (res)
    {
        if (_noiseBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_noiseBinding, this, "GroundCover noise sampler") == false)
            {
                setStatus(Status::ResourceUnavailable, "No texture unit available for noise sampler");
                return;
            }
        }    
        
        // Compute LOD for each asset group if necessary.
        for (int g = 0; g < options().groups().size(); ++g)
        {
            Options::Group& group = options().group((AssetGroup::Type)g);
            if (!group.lod().isSet())
            {
                unsigned bestLOD = 0;
                for (unsigned lod = 1; lod <= 99; ++lod)
                {
                    bestLOD = lod - 1;
                    float lodRange = res->getVisibilityRangeHint(lod);
                    if (group.maxRange() > lodRange || lodRange == FLT_MAX)
                    {
                        break;
                    }
                }
                group.lod() = bestLOD;

                OE_INFO << LC 
                    << "Rendering asset group" << s_assetGroupName[g] 
                    << " at terrain level " << bestLOD <<  std::endl;
            }
        }

        _renderer = std::make_shared<Renderer>(this);
    }

    buildStateSets();
}

namespace
{
    // adds the defines for a shared image layer
    void bind(
        ImageLayer* layer,
        const std::string& sampler,
        const std::string& matrix,
        osg::StateSet* stateset )
    {
        if (layer) {
            stateset->setDefine(sampler, layer->getSharedTextureUniformName());
            stateset->setDefine(matrix, layer->getSharedTextureMatrixUniformName());
        }
        else {
            stateset->removeDefine(sampler);
            stateset->removeDefine(matrix);
        }
    }
}

void
VegetationLayer::buildStateSets()
{
    if (_renderer == nullptr) {
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

    // calculate the tile widths based on the LOD:
    if (_mapProfile.valid())
    {
        for (unsigned lod = 0; lod < 20; ++lod)
        {
            unsigned tx, ty;
            _mapProfile->getNumTiles(lod, tx, ty);
            GeoExtent e = TileKey(lod, tx / 2, ty / 2, _mapProfile.get()).getExtent();
            GeoCircle c = e.computeBoundingGeoCircle();
            double width_m = 2.0 * c.getRadius() / 1.4142;
            _renderer->_tileWidths[lod] = width_m;
        }
    }

    GroundCoverShaders shaders;

    // Layer-wide stateset:
    osg::StateSet* stateset = getOrCreateStateSet();

    // bind the noise sampler.
    stateset->setTextureAttribute(_noiseBinding.unit(), _renderer->_noiseTex.get(), osg::StateAttribute::OVERRIDE);
    stateset->addUniform(new osg::Uniform(NOISE_SAMPLER, _noiseBinding.unit()));

    bind(getColorLayer(),   "OE_GROUNDCOVER_COLOR_SAMPLER", "OE_GROUNDCOVER_COLOR_MATRIX", stateset);
    bind(getLifeMapLayer(), "OE_LIFEMAP_SAMPLER", "OE_LIFEMAP_MATRIX", stateset);
    bind(getBiomeLayer(),   "OE_BIOME_SAMPLER", "OE_BIOME_MATRIX", stateset);

    // disable backface culling to support shadow/depth cameras,
    // for which the geometry shader renders cross hatches instead of billboards.
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::PROTECTED);

    if (!_sseU.valid())
    {
        _sseU = new osg::Uniform("oe_veg_sse", getMaxSSE());
    }

    _sseU->set(getMaxSSE());
    stateset->addUniform(_sseU.get());

    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
    {
        stateset->setDefine("OE_IS_MULTISAMPLE", "1");
        stateset->setMode(GL_MULTISAMPLE, 1);
        stateset->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);
    }
    else
    {
        stateset->removeMode(GL_MULTISAMPLE);
        stateset->removeMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB);
        stateset->removeDefine("OE_IS_MULTISAMPLE");
    }

    if (getUseAlphaToCoverage())
        stateset->setAttributeAndModes(_renderer->_a2cBlending.get(), 1);

    // NEXT assemble the asset group statesets.
    if (AssetGroup::TREES < NUM_ASSET_GROUPS)
        configureTrees();

    if (AssetGroup::UNDERGROWTH < NUM_ASSET_GROUPS)
        configureUndergrowth();
}

void
VegetationLayer::configureTrees()
{
    auto& trees = options().group(AssetGroup::TREES);
    trees._renderStateSet = new osg::StateSet();

    trees._renderStateSet->addUniform(new osg::Uniform("oe_veg_maxAlpha", trees.maxAlpha().get()));
    trees._renderStateSet->addUniform(new osg::Uniform("oe_veg_maxRange", trees.maxRange().get()));

    VirtualProgram* trees_vp = VirtualProgram::getOrCreate(trees._renderStateSet.get());
    trees_vp->setName("Vegetation:Trees");
    trees_vp->addGLSLExtension("GL_ARB_gpu_shader_int64");
    trees_vp->addBindAttribLocation("oe_veg_texArenaIndex", 6);
    trees_vp->addBindAttribLocation("oe_veg_nmlArenaIndex", 7);

    GroundCoverShaders shaders;
    shaders.load(trees_vp, shaders.Trees);

    // functor for generating billboard geometry for trees:
    trees._createImposter = [](std::vector<osg::Texture*>& textures)
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
    };
}

void
VegetationLayer::configureUndergrowth()
{
    auto& undergrowth = options().group(AssetGroup::UNDERGROWTH);
    undergrowth._renderStateSet = new osg::StateSet();

    undergrowth._renderStateSet->addUniform(new osg::Uniform("oe_veg_maxAlpha", undergrowth.maxAlpha().get()));
    undergrowth._renderStateSet->addUniform(new osg::Uniform("oe_veg_maxRange", undergrowth.maxRange().get()));

    VirtualProgram* undergrowth_vp = VirtualProgram::getOrCreate(undergrowth._renderStateSet.get());
    undergrowth_vp->setName("Vegetation:Undergrowth");
    undergrowth_vp->addGLSLExtension("GL_ARB_gpu_shader_int64");
    undergrowth_vp->addBindAttribLocation("oe_veg_texArenaIndex", 6);
    // don't need to bind oe_veg_nmlArenaIndex since undergrowth does not use it

    GroundCoverShaders shaders;
    shaders.load(undergrowth_vp, shaders.Grass);

    // functor for generating billboard geometry for grass:
    undergrowth._createImposter = [](std::vector<osg::Texture*>& textures)
    {
        constexpr unsigned vertsPerInstance = 16;
        constexpr unsigned indiciesPerInstance = 54;

        osg::Geometry* out_geom = new osg::Geometry();
        out_geom->setUseVertexBufferObjects(true);
        out_geom->setUseDisplayList(false);

        static const GLushort indices[indiciesPerInstance] = {
            0,1,4, 4,1,5, 1,2,5, 5,2,6, 2,3,6, 6,3,7,
            4,5,8, 8,5,9, 5,6,9, 9,6,10, 6,7,10, 10,7,11,
            8,9,12, 12,9,13, 9,10,13, 13,10,14, 10,11,14, 14,11,15
        };

        out_geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, indiciesPerInstance, &indices[0]));

        out_geom->setVertexArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, vertsPerInstance));

        osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, vertsPerInstance);
        normals->assign(vertsPerInstance, osg::Vec3(0, 1, 0));
        out_geom->setNormalArray(normals);

        osg::StateSet* ss = out_geom->getOrCreateStateSet();
        if (textures.size() > 0)
            ss->setTextureAttribute(0, textures[0], 1);
        if (textures.size() > 1)
            ss->setTextureAttribute(1, textures[1], 1);

        //osg::ShortArray* handles = new osg::ShortArray(vertsPerInstance);
        //handles->setBinding(osg::Array::BIND_PER_VERTEX);
        //handles->assign(vertsPerInstance, tex_index_0);
        //out_geom->setVertexAttribArray(6, handles);

        return out_geom;
    };
}


void
VegetationLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    if (_renderer != nullptr)
    {
        _renderer->resizeGLObjectBuffers(maxSize);
    }

    PatchLayer::resizeGLObjectBuffers(maxSize);
}

void
VegetationLayer::releaseGLObjects(osg::State* state) const
{
    if (_renderer != nullptr)
    {
        _renderer->releaseGLObjects(state);
    }

    PatchLayer::releaseGLObjects(state);
}

unsigned
VegetationLayer::getNumTilesRendered() const
{
    return _renderer ? _renderer->_lastTileBatchSize : 0u;
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
VegetationLayer::createParametricGeometry(
    AssetGroup::Type group,
    std::vector<osg::Texture*>& textures) const
{
    return options().group(group)._createImposter(textures);
}

//........................................................................

VegetationLayer::TileManager::TileManager() :
    _highestOccupiedSlot(-1)
{
    //nop
}

void
VegetationLayer::TileManager::reset()
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
VegetationLayer::TileManager::allocate(const TileKey& key, int revision)
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

    _highestOccupiedSlot = std::max(_highestOccupiedSlot, slot);

    _current[slot]._key = key;
    _current[slot]._revision = revision;
    _current[slot]._expired = false;
    _current[slot]._dirty = true;

    return slot;
}

int
VegetationLayer::TileManager::release(const TileKey& key)
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
VegetationLayer::TileManager::release(int slot)
{
    if (slot >= 0 && slot < _current.size())
    {
        _current[slot]._revision = -1;
        _current[slot]._expired = false;
        _current[slot]._dirty = true;
    }
}

bool
VegetationLayer::TileManager::inUse(int slot) const
{
    return slot < _current.size() && _current[slot]._revision >= 0;
}

int
VegetationLayer::TileManager::getSlot(const TileKey& key) const
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

VegetationLayer::Renderer::PCPUniforms::PCPUniforms()
{
    // initialize all the uniform locations - we will fetch these at draw time
    // when the program is active
    _generateDataUL = -1;
    _maxRangeUL = -1;
}

VegetationLayer::Renderer::Renderer(VegetationLayer* layer)
{
    _layer = layer;

    reset();

    // create uniform IDs for each of our uniforms
    //_isMSUName = osg::Uniform::getNameID("oe_veg_isMultisampled");
    _computeDataUName = osg::Uniform::getNameID("oe_tile");
    _maxRangeUName = osg::Uniform::getNameID("oe_veg_maxRange");

    _a2cBlending = new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO);

    // Load our compute shader
    GroundCoverShaders shaders;

    std::string computeSource = ShaderLoader::load(shaders.Compute, shaders, layer->getReadOptions());
    _computeSS = new osg::StateSet();
    osg::Program* compute = new osg::Program();
    compute->setName("VegetationLayer:COMPUTE");
    osg::Shader* computeShader = new osg::Shader(osg::Shader::COMPUTE, computeSource);
    computeShader->setName(shaders.Compute);
    compute->addShader(computeShader);
    _computeSS->setAttribute(compute, osg::StateAttribute::OVERRIDE);

    // make a 4-channel noise texture to use
    NoiseTextureFactory noise;
    _noiseTex = noise.create(256u, 4u);
}

void
VegetationLayer::Renderer::reset()
{
    _lastVisit.setReferenceTime(DBL_MAX);
    _lastVisit.setFrameNumber(~0U);
    _lastTileBatchSize = 0u;
    _uniforms.clear();
    _cameraState.clear();
    _geomClouds.clear();
    _geomCloudsInProgress.abandon();

    OE_SOFT_ASSERT_AND_RETURN(_layer.valid() && _layer->getBiomeLayer(), void());

    BiomeManager& biomeMan = _layer->getBiomeLayer()->getBiomeManager();
    _biomeRevision = biomeMan.getRevision();
}

VegetationLayer::Renderer::~Renderer()
{
    releaseGLObjects(nullptr);
}

void
VegetationLayer::Renderer::compileClouds(
    osg::RenderInfo& ri)
{
    for (auto iter : _geomClouds)
    {
        GeometryCloud* cloud = iter.second.get();
        cloud->getGeometry()->compileGLObjects(ri);
        cloud->getGeometry()->getStateSet()->compileGLObjects(*ri.getState());
    }
}

bool
VegetationLayer::Renderer::isNewGeometryCloudAvailable(
    osg::RenderInfo& ri)
{
    BiomeManager& biomeMan = _layer->getBiomeLayer()->getBiomeManager();

    if (_biomeRevision.exchange(biomeMan.getRevision()) != biomeMan.getRevision())
    {
        // revision changed; start a new asset load.

        OE_INFO << LC 
            << "Biomes changed (rev="<< _biomeRevision
            <<" fr=" << ri.getState()->getFrameStamp()->getFrameNumber() 
            <<")...building new veg clouds" << std::endl;

        osg::observer_ptr<VegetationLayer> layer_weakptr(_layer.get());

        auto buildGeometry = [layer_weakptr](Cancelable* c) -> GeometryCloudCollection
        {
            GeometryCloudCollection result;
            osg::ref_ptr<VegetationLayer> layer;
            if (layer_weakptr.lock(layer))
            {
                BiomeManager& biomeMan = layer->getBiomeLayer()->getBiomeManager();

                std::set<AssetGroup::Type> groups;

                result = biomeMan.updateResidency(
                    [&](AssetGroup::Type group, std::vector<osg::Texture*>& textures)
                    {
                        return layer->createParametricGeometry(group, textures);
                    },
                    layer->getReadOptions());
            }
            return result;
        };

#if 1 // async
        _geomCloudsInProgress = Job().dispatch<GeometryCloudCollection>(buildGeometry);
#else // sync
        _geomClouds = buildGeometry(nullptr);
        compileClouds();
#endif
    }

    else if (_geomCloudsInProgress.isAvailable())
    {
        _geomClouds = _geomCloudsInProgress.release();

        if (!_geomClouds.empty())
        {
            OE_INFO << LC << "New geom clouds are ready (fr="
                << ri.getState()->getFrameStamp()->getFrameNumber() << ")"
                << std::endl;

            // Very important that we compile this now so there aren't any
            // texture conflicts later on.
            // Strange that osg::Geometry doesn't also compile its StateSet
            // so we have to do it manually...
            compileClouds(ri);

            //OE_INFO << LC << "New geom clouds finished compiling (fr="
            //    << ri.getState()->getFrameStamp()->getFrameNumber() << ")"
            //    << std::endl;
        }

        return true;
    }

    return false;
}

VegetationLayer::Renderer::PCPUniforms*
VegetationLayer::Renderer::getUniforms(
    osg::RenderInfo& ri)
{
    auto pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp) return nullptr;
    PCPUniforms& u = _uniforms[pcp];
    if (u._generateDataUL < 0)
        u._generateDataUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_tile"));
    if (u._maxRangeUL < 0)
        u._maxRangeUL = pcp->getUniformLocation(osg::Uniform::getNameID("oe_veg_maxRange"));
    return &u;
}

//...................................................................

VegetationLayer::Renderer::CameraState::CameraState() :
    _geomDirty(false),
    _groups(NUM_ASSET_GROUPS)
{
}

struct StateEx : public osg::State {
    UniformMap& getMutableUniformMap() { return _uniformMap; }
    inline void updateDefines() { _defineMap.updateCurrentDefines(); }
};

void
VegetationLayer::Renderer::CameraState::setGeometry(
    osg::RenderInfo& ri,
    GeometryCloudCollection& data)
{
    //todo - loop over groups, assign data, and mark dirty
    for (auto& iter : data)
    {
        AssetGroup::Type group = iter.first;
        GeometryCloud* cloud = iter.second.get();

        GroupState& gs = _groups[group];
        if (!gs.active())
        {
            gs._group = group;

            auto& groupOptions = _renderer->_layer->options().group(gs._group);
            gs._renderer = _renderer;
            gs._layer = _renderer->_layer.get();            
            gs._lod = groupOptions.lod().get();
            gs._castShadows = groupOptions.castShadows().get();
            gs._previousHash = -1;
            gs._renderSS = groupOptions._renderStateSet;
            gs._maxRange_u = gs._renderSS->getUniform("oe_veg_maxRange");

            gs._instancer = new InstanceCloud();

            float spacing_m = groupOptions.spacing()->as(Units::METERS);
            unsigned numInstances1D = _renderer->_tileWidths[gs._lod] / spacing_m;
            gs._instancer->setNumInstancesPerTile(numInstances1D, numInstances1D);
        }

        gs.setGeometry(cloud);
    }

    _geomDirty = true;
}

void
VegetationLayer::Renderer::CameraState::draw(
    osg::RenderInfo& ri,
    const TileBatch& batch)
{
    osg::State* state = ri.getState();

    // do not shadow groups marked as not casting shadows
    bool isShadowCam = CameraUtils::isShadowCamera(ri.getCurrentCamera());

    // Why doesn't OSG push this in the LayerDrawable RenderLeaf? No one knows.
    // Anyway, push it before checking the oe_veg_sse uniform.
    state->pushStateSet(_renderer->_layer->getStateSet());

    // do we need to re-generate some (or all) of the tiles?
    bool needsGenerate =
        _geomDirty ||
        _renderer->_layer->_debug ||
        _renderer->_layer->_forceGenerate;

    // split up the tiles to render by asset group:
    for (auto& group : _groups)
    {
        if (group.active() && group.accepts(ri))
        {
            // returns true if the new batch differs from the previous batch,
            // in which case we need to regenerate tiles
            if (group.setTileBatch(batch))
            {
                needsGenerate = true;
            }

            // figure out if we need to reallocate the GPU memory, which will
            // happen if the number of tiles in the new batch exceeds the number
            // we're already allocated space for
            if (needsGenerate)
            {
                bool reallocated = group.reallocate(ri);

                // If we had to allocate more GPU memory, or if we have totally new geometry,
                // clear out the tile manager and start fresh.
                if (reallocated)
                {
                    group._tilemgr.reset();
                }
            }
        }
    }

    // if we need to generate, we need to cull too
    bool needsCull = needsGenerate;

    // if the SSE value changed, we need to cull.
    // This feels a little janky but it works.
    if (!needsCull)
    {
        if (_last_sse != _renderer->_layer->getMaxSSE())
        {
            _last_sse = _renderer->_layer->getMaxSSE();
            needsCull = true;
        }
    }

    // If the camera moved, we need to cull:
    if (!needsCull)
    {
        osg::Matrix mvp = state->getModelViewMatrix() * state->getProjectionMatrix();
        if (mvp != _lastMVP)
        {
            _lastMVP = mvp;
            needsCull = true;
        }
    }

    // compute pass:
    if (needsGenerate || needsCull)
    {
        state->pushStateSet(_renderer->_computeSS.get());

        for (auto& group : _groups)
        {
            if (group.active() && !group.empty())
            {
                group.compute(ri, needsGenerate, needsCull);
            }
        }

        state->popStateSet();
    }

    // draw pass:
    for (auto& group : _groups)
    {
        if (group.active() && !group.empty() && group.accepts(ri))
        {
            group.render(ri);
        }
    }

    // pop the layer's stateset.
    state->popStateSet();

    // clear the dirty flag for the next frame.
    _geomDirty = false;
}

//...................................................................

VegetationLayer::Renderer::GroupState::GroupState() :
    _renderer(nullptr),
    _previousHash(-1),
    _batch(nullptr)
{
    //nop
}

bool
VegetationLayer::Renderer::GroupState::accepts(
    osg::RenderInfo& ri) const
{
    if (CameraUtils::isShadowCamera(ri.getCurrentCamera()) &&
        _castShadows == false)
    {
        return false;
    }
    return true;
}

void
VegetationLayer::Renderer::GroupState::setGeometry(
    GeometryCloud* cloud)
{
    OE_HARD_ASSERT(cloud != nullptr);

    _instancer->setGeometryCloud(cloud);
    _tilemgr.reset();
    _previousHash = -1;
}

bool
VegetationLayer::Renderer::GroupState::setTileBatch(
    const TileBatch& input)
{
    _batch._env = input.env();
    _batch._tiles.clear();

    std::size_t hash(0);

    for (auto tile_ptr : input._tiles)
    {
        //if (tile_ptr->getKey().str() == "14/17205/4349")
        if (tile_ptr->getKey().getLOD() == _lod)
        {
            _batch._tiles.push_back(tile_ptr);

            hash = hash_value_unsigned(
                hash,
                tile_ptr->getKey().hash(),
                (std::size_t)tile_ptr->getRevision());
        }
    }
    bool batchChanged = hash != _previousHash;
    _previousHash = hash;
    return batchChanged;
}

bool
VegetationLayer::Renderer::GroupState::reallocate(
    osg::RenderInfo& ri)
{
    return _instancer->allocateGLObjects(ri, _batch.tiles().size());
}

void
VegetationLayer::Renderer::GroupState::compute(
    osg::RenderInfo& ri,
    bool needs_generate,
    bool needs_cull)
{
    osg::State* state = ri.getState();

    state->apply(_instancer->getGeometryCloud()->getStateSet());
    
    OE_HARD_ASSERT(state->getLastAppliedProgramObject() != nullptr); // catch shader error

    if (state->getUseModelViewAndProjectionUniforms())
        state->applyModelViewAndProjectionUniformsIfRequired();

    _instancer->bind();

    if (needs_generate)
    {
        // the generator will change this, so save it and restore it later.
        osg::Matrix mvm = state->getModelViewMatrix();

        collect(ri);
        generate(ri);

        state->applyModelViewMatrix(mvm);
    }

    if (needs_cull)
    {
        cull(ri);
    }

    _instancer->unbind(ri);
}

void
VegetationLayer::Renderer::GroupState::cull(
    osg::RenderInfo& ri)
{
    OE_PROFILING_ZONE_NAMED("Cull/Sort");

    PCPUniforms& uniforms = *_renderer->getUniforms(ri);

    // collect and upload tile matrix data
    for (auto tile : _batch.tiles())
    {
        int slot = _tilemgr.getSlot(tile->getKey());
        if (slot >= 0)
        {
            _instancer->setMatrix(slot, tile->getModelViewMatrix());
        }
        else
        {
            OE_WARN << "Internal error -- CULL should not see an inactive tile" << std::endl;
            OE_SOFT_ASSERT(slot >= 0);
        }
    }

    // Set the max_range uniform
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();
    ext->glUniform1f(
        uniforms._maxRangeUL,
        _renderer->_layer->options().group(_group).maxRange().get());

    _instancer->cull(ri); // cull and sort
}

void
VegetationLayer::Renderer::GroupState::render(
    osg::RenderInfo& ri)
{
    osg::State* state = ri.getState();

    _maxRange_u->set(std::min(
        _layer->getMaxVisibleRange(), 
        _layer->options().group(_group).maxRange().get()));

    state->pushStateSet(_instancer->getGeometryCloud()->getStateSet());

    // activate the rendering program for this group:
    state->apply(_renderSS.get());

    // since we changed programs, we need to re-apply the matrix uniforms
    if (state->getUseModelViewAndProjectionUniforms())
        state->applyModelViewAndProjectionUniformsIfRequired();

    // binds the instancer's buffer objects
    _instancer->bind();

    // draws the instances
    _instancer->draw(ri);

    // releases the command buffer object
    _instancer->unbind(ri);


    state->popStateSet();

    // Unbind the current VAO so OSG doesn't crash the driver
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    ri.getState()->unbindVertexArrayObject();
#endif

    // Just to be safe.
    ri.getState()->setLastAppliedProgramObject(nullptr);
}

void
VegetationLayer::Renderer::GroupState::collect(
    osg::RenderInfo& ri)
{
    OE_PROFILING_ZONE_NAMED("Collect");

    // Put all tiles on the expired list, only removing them when we
    // determine that they are good to keep around.
    for (auto& i : _tilemgr._current)
        if (i._revision >= 0)
            i._expired = true;

    // traverse each tile
    for (auto tile : _batch.tiles())
    {
        // Decide whether this tile really needs regen:
        int slot = _tilemgr.getSlot(tile->getKey());

        if (slot < 0) // new tile.
        {
            TileManager::Tile newTile;
            newTile._key = tile->getKey();
            newTile._revision = tile->getRevision();
            _tilemgr._new.emplace_back(newTile);
            // will allocate a slot later, after we see if anybody freed one.
            //OE_INFO << "Greetings, " << tile._key->str() << ", r=" << tile._revision << std::endl;
        }

        else
        {
            TileManager::Tile& i = _tilemgr._current[slot];

            // Keep it around.
            i._expired = false;

            if (i._revision != tile->getRevision() || 
                _layer->_forceGenerate)
            {
                // revision changed! Mark it dirty for regeneration.
                i._revision = tile->getRevision();
                i._dirty = true;
            }
        }
    }

    // Release anything still marked as expired.
    for (int slot = 0; slot < _tilemgr._current.size(); ++slot)
    {
        if (_tilemgr._current[slot]._expired)
        {
            _tilemgr.release(slot);
        }
    }

    // Allocate a slot for each new tile. We do this now AFTER
    // we have released any expired tiles
    for (const auto& i : _tilemgr._new)
    {
        _tilemgr.allocate(i._key, i._revision);
    }
    _tilemgr._new.clear();
}

void
VegetationLayer::Renderer::GroupState::generate(
    osg::RenderInfo& ri)
{
    OE_PROFILING_ZONE_NAMED("Generate");

    // Tell the instancer the highest tile slot with data in it.
    _instancer->setHighestTileSlot(_tilemgr._highestOccupiedSlot);

    // Tell the instancer which tiles are active.
    for (int slot = 0; slot <= _tilemgr._current.size(); ++slot)
    {
        _instancer->setTileActive(slot, _tilemgr.inUse(slot));
    }

    _instancer->generate_begin(ri);

    PCPUniforms& uniforms = *_renderer->getUniforms(ri);
    OE_HARD_ASSERT(uniforms._generateDataUL >= 0);
    
    osg::GLExtensions* ext = nullptr;

    for (auto tile : _batch.tiles())
    {
        int slot = _tilemgr.getSlot(tile->getKey());
        if (slot >= 0)
        {
            TileManager::Tile& i = _tilemgr._current[slot];

            if (i._dirty == true)
            {
                if (ext == nullptr)
                    ext = ri.getState()->get<osg::GLExtensions>();

                const osg::BoundingBox& box = tile->getBBox();
                uniforms._generateData[0] = box.xMin();
                uniforms._generateData[1] = box.yMin();
                uniforms._generateData[2] = box.xMax();
                uniforms._generateData[3] = box.yMax();
                uniforms._generateData[4] = (float)slot;

                ext->glUniform1fv(uniforms._generateDataUL, 5, &uniforms._generateData[0]);

                //OE_INFO << "oe_tile " <<
                //    box.xMin() << " " << box.yMin() << " " <<
                //    box.xMax() << " " << box.yMax() << " " <<
                //    slot << std::endl;

                tile->apply(ri, _batch.env());

                _instancer->generate_tile(ri);

                i._dirty = false;
            }
        }
    }

    _instancer->generate_end(ri);


#if 0
    OE_INFO << "-----" << std::endl;
    OE_INFO << "Tiles:" << std::endl;
    for (int slot = 0; slot < _tilemgr._current.size(); ++slot)
    {
        const TileGenInfo& i = _tilemgr._current[slot];
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

void
VegetationLayer::Renderer::draw(
    osg::RenderInfo& ri,
    const TileBatch& batch)
{
    if (_status.isError())
        return;


    OE_PROFILING_ZONE_NAMED("VegetationLayer::draw");

    _lastVisit = *ri.getState()->getFrameStamp();
    _lastTileBatchSize = batch.tiles().size();

    // state associated with the current camera
    CameraState& this_cs = _cameraState[ri.getCurrentCamera()];

    // one at a time please when checking for and installing new data
    {
        ScopedMutexLock lock(_newGeometryMutex);

        // detects whether the biome has changed and we need to 
        // reload the geometry clouds once they become available.
        bool newGeometryAvailable = isNewGeometryCloudAvailable(ri);

        if (_geomClouds.empty())
        {
            if (newGeometryAvailable)
            {
                OE_INFO << LC << "New geometry available, but collection is empty" << std::endl;
            }
            return;
        }

        // when new data becomes available we must assign it to ALL camera
        // states regardless of which one detected the new data.
        if (newGeometryAvailable)
        {
            for (auto& iter : _cameraState)
            {
                CameraState& cs = iter.second;
                if (!cs.active())
                {
                    cs._renderer = _layer->_renderer;
                }
                cs.setGeometry(ri, _geomClouds);
            }
        }
        else if (!this_cs.active())
        {
            this_cs._renderer = _layer->_renderer;
            this_cs.setGeometry(ri, _geomClouds);
        }
    }

    // compute + render
    this_cs.draw(ri, batch);

    // Just to be safe.
    ri.getState()->setLastAppliedProgramObject(nullptr);
}

void
VegetationLayer::Renderer::resizeGLObjectBuffers(unsigned maxSize)
{
    for (auto cs_iter : _cameraState)
    {
        for (auto cloud_iter : _geomClouds)
        {
            cloud_iter.second->resizeGLObjectBuffers(maxSize);
        }

        //for (auto& gs : cs_iter.second._groups)
        //{
        //    if (gs._instancer.valid())
        //    {
        //        gs._instancer->resizeGLObjectBuffers(maxSize);
        //    }
        //}
    }
}

void
VegetationLayer::Renderer::releaseGLObjects(osg::State* state) const
{
    for (auto cs_iter : _cameraState)
    {
        for (auto cloud_iter : _geomClouds)
        {
            cloud_iter.second->releaseGLObjects(state);
        }

        for (auto& gs : cs_iter.second._groups)
        {
            if (gs._instancer.valid())
            {
                gs._instancer->releaseGLObjects(state);
            }
        }
    }
}

const std::string&
VegetationLayer::Renderer::getName() const
{
    static std::string empty;
    return _layer.valid() ? _layer->getName() : empty;
}
