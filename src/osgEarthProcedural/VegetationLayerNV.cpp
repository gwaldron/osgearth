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
#include "VegetationLayerNV"
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
#include <osgEarth/GLUtils>
#include <osgEarth/Random>
#include <osgEarth/rtree.h>

#include <osg/BlendFunc>
#include <osg/Multisample>
#include <osg/Texture2D>
#include <osg/Version>
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgUtil/CullVisitor>
#include <osgUtil/Optimizer>

#include <cstdlib> // getenv

#define LC "[VegetationLayerNV] " << getName() << ": "

#define OE_DEVEL OE_DEBUG

//#define ATLAS_SAMPLER "oe_veg_atlas"
#define NOISE_SAMPLER "oe_veg_noiseTex"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

#ifndef GL_SAMPLE_ALPHA_TO_COVERAGE_ARB
#define GL_SAMPLE_ALPHA_TO_COVERAGE_ARB   0x809E
#endif

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(vegetationnv, VegetationLayerNV);

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


    const char* cs = R"(

#version 430
#extension GL_ARB_gpu_shader_int64 : enable
#pragma oe_use_shared_layer(OE_LIFEMAP_TEX, OE_LIFEMAP_MAT)

bool oe_custom_cull(in vec2 uv)
{
#ifdef OE_LIFEMAP_TEX
    vec2 lifemap_uv = (OE_LIFEMAP_MAT * vec4(uv,0,1)).st;
    vec3 lifemap = texture(OE_LIFEMAP_TEX, lifemap_uv).xyz;
    return (lifemap[1] > 0.01); // density
#else
    return true;
#endif
}

        )";

    const char* vs = R"(

#version 460
#extension GL_ARB_gpu_shader_int64 : enable

#pragma import_defines(OE_USE_ALPHA_TO_COVERAGE)

layout(binding=1, std430) buffer TextureArena {
    uint64_t textures[];
};
struct Instance {
    mat4 xform;
    int cmd_index;
    float _padding[3];
};
layout(binding=0, std430) buffer Instances {
    Instance instances[];
};

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec4 color;
layout(location=3) in vec2 uv;
layout(location=4) in int albedo; // todo: material LUT index

out vec3 vp_Normal;
out vec4 vp_Color;
out vec2 tex_coord;
out vec3 pos_view3;
flat out uint64_t tex_handle;

void vegetation_vs(inout vec4 vertex)
{
    int i = gl_BaseInstance + gl_InstanceID;
    vertex = instances[i].xform * vec4(position, 1);
    vp_Color = color;
    vp_Normal = normal;
    tex_coord = uv;
    tex_handle = albedo >= 0 ? textures[albedo] : 0;

    pos_view3 = (gl_ModelViewMatrix * vertex).xyz;
};

    )";

    const char* fs = R"(

#version 430
#extension GL_ARB_gpu_shader_int64 : enable
#pragma import_defines(OE_USE_ALPHA_DISCARD)

in vec2 tex_coord;
in vec3 pos_view3;
flat in uint64_t tex_handle;

void vegetation_fs(inout vec4 color)
{
    if (tex_handle > 0) {
        vec4 texel = texture(sampler2D(tex_handle), tex_coord);
        color *= texel;
    }

    vec3 pos = gl_FragCoord.xyz;
    vec3 face_normal = normalize(cross(dFdx(pos_view3), dFdy(pos_view3)));
    float d = clamp(2.0*abs(dot(face_normal, vec3(0,0,1))),0.0,1.0);
    color.a *= d;

#ifdef OE_USE_ALPHA_TO_COVERAGE
    // mitigate the screen-door effect of A2C in the distance
    // https://tinyurl.com/y7bbbpl9
    //float a = (color.a - oe_veg_maxAlpha) / max(fwidth(color.a), 0.0001) + 0.5;
    //color.a = mix(color.a, a, oe_veg_distance);

    // adjust the alpha based on the calculated mipmap level:
    // better, but a bit more expensive than the above method
    // https://tinyurl.com/fhu4zdxz
    if (oe_veg_texHandle > 0UL)
    {
        ivec2 tsize = textureSize(sampler2D(tex_handle), 0);
        vec2 cf = vec2(float(tsize.x)*tex_coord.s, float(tsize.y)*tex_coord.t);
        vec2 dx_vtc = dFdx(cf);
        vec2 dy_vtc = dFdy(cf);
        float delta_max_sqr = max(dot(dx_vtc, dx_vtc), dot(dy_vtc, dy_vtc));
        float mml = max(0, 0.5 * log2(delta_max_sqr));
        color.a *= (1.0 + mml * 0.25);
    }

#else
    if (color.a < 0.15)
        discard;
#endif
}

    )";
}

//........................................................................

Config
VegetationLayerNV::Options::getConfig() const
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
VegetationLayerNV::Options::fromConfig(const Config& conf)
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
        groups()[AssetGroup::TREES].enabled() = true;
        groups()[AssetGroup::TREES].castShadows() = true;
        groups()[AssetGroup::TREES].maxRange() = 2500.0f;
        groups()[AssetGroup::TREES].lod() = 14;
        groups()[AssetGroup::TREES].spacing() = Distance(15.0f, Units::METERS);
        groups()[AssetGroup::TREES].maxAlpha() = 0.15f;
    }

    if (AssetGroup::UNDERGROWTH < NUM_ASSET_GROUPS)
    {
        groups()[AssetGroup::UNDERGROWTH].enabled() = true;
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
            group_c.get("enabled", group.enabled());
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
VegetationLayerNV::LayerAcceptor::acceptLayer(osg::NodeVisitor& nv, const osg::Camera* camera) const
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
VegetationLayerNV::LayerAcceptor::acceptKey(const TileKey& key) const
{
     return _layer->hasGroupAtLOD(key.getLOD());
}

//........................................................................

void VegetationLayerNV::setMaxSSE(float value)
{
    if (value != options().maxSSE().get())
    {
        options().maxSSE() = value;
        if (_sseU.valid())
            _sseU->set(value);
    }
}

float VegetationLayerNV::getMaxSSE() const
{
    return options().maxSSE().get();
}

//........................................................................

void
VegetationLayerNV::init()
{
    PatchLayer::init();

    setAcceptCallback(new LayerAcceptor(this));
}

VegetationLayerNV::~VegetationLayerNV()
{
    close();
}

Status
VegetationLayerNV::openImplementation()
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
VegetationLayerNV::closeImplementation()
{
    releaseGLObjects(nullptr);
    return PatchLayer::closeImplementation();
}

void
VegetationLayerNV::update(osg::NodeVisitor& nv)
{
    // this code will release memory after the layer's not been
    // used for a while.
    if (isOpen())
    {
        int df = nv.getFrameStamp()->getFrameNumber() - _lastVisit.getFrameNumber();
        double dt = nv.getFrameStamp()->getReferenceTime() - _lastVisit.getReferenceTime();

        if (dt > 5.0 && df > 60)
        {
            releaseGLObjects(nullptr);

            reset();

            if (getBiomeLayer())
                getBiomeLayer()->getBiomeManager().reset();

            OE_INFO << LC << "timed out for inactivity." << std::endl;
        }

        checkForNewAssets();

        BiomeManager::Drawables newDrawables = _newDrawables.release();
        if (!newDrawables.objects.empty())
        {
            _drawables = std::move(newDrawables);
        }
    }
}

void
VegetationLayerNV::setBiomeLayer(BiomeLayer* layer)
{
    _biomeLayer.setLayer(layer);

    if (layer)
    {
        buildStateSets();
    }
}

BiomeLayer*
VegetationLayerNV::getBiomeLayer() const
{
    return _biomeLayer.getLayer();
}

void
VegetationLayerNV::setLifeMapLayer(LifeMapLayer* layer)
{
    _lifeMapLayer.setLayer(layer);
    if (layer)
    {
        buildStateSets();
    }
}

LifeMapLayer*
VegetationLayerNV::getLifeMapLayer() const
{
    return _lifeMapLayer.getLayer();
}

void
VegetationLayerNV::setColorLayer(ImageLayer* value)
{
    options().colorLayer().setLayer(value);
    if (value)
    {
        buildStateSets();
    }
}

ImageLayer*
VegetationLayerNV::getColorLayer() const
{
    return options().colorLayer().getLayer();
}

void
VegetationLayerNV::setUseAlphaToCoverage(bool value)
{
    options().alphaToCoverage() = value;
}

bool
VegetationLayerNV::getUseAlphaToCoverage() const
{
    return options().alphaToCoverage().get();
}

bool
VegetationLayerNV::getCastShadows() const
{
    for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
    {
        if (options().groups()[i].castShadows() == true)
            return true;
    }
    return false;
}

bool
VegetationLayerNV::hasGroupAtLOD(unsigned lod) const
{
    for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
    {
        if (options().groups()[i].lod() == lod)
            return true;
    }
    return false;
}

unsigned
VegetationLayerNV::getGroupLOD(AssetGroup::Type group) const
{
    if (group > 0 && group < NUM_ASSET_GROUPS)
        return options().group(group).lod().get();
    else
        return 0;
}

void
VegetationLayerNV::setMaxRange(AssetGroup::Type type, float value)
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);

    auto& group = options().group(type);
    group.maxRange() = value;
}

float
VegetationLayerNV::getMaxRange(AssetGroup::Type type) const
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);
    return options().group(type).maxRange().get();
}

void
VegetationLayerNV::setEnabled(AssetGroup::Type type, bool value)
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);

    auto& group = options().group(type);
    group.enabled() = value;
}

bool
VegetationLayerNV::getEnabled(AssetGroup::Type type) const
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);
    return options().group(type).enabled().get();
}

void
VegetationLayerNV::addedToMap(const Map* map)
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

    _map = map;

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
VegetationLayerNV::removedFromMap(const Map* map)
{
    PatchLayer::removedFromMap(map);

    options().colorLayer().removedFromMap(map);
}

void
VegetationLayerNV::prepareForRendering(TerrainEngine* engine)
{
    PatchLayer::prepareForRendering(engine);

    _textures = new TextureArena();

    _defaultTree = Chonk::create();

    ChonkFactory factory(_textures.get());

    _defaultTree->add(
        osgDB::readRefNodeFile("D:/data/splat/assets/trees/RedOak/redoak.osgb"),
        400,
        FLT_MAX,
        factory);

    _defaultTree->add(
        osgDB::readRefNodeFile("H:/devel/osgearth/master/repo/data/tree.osg"),
        0,
        400,
        factory);

    TerrainResources* res = engine->getResources();
    if (res)
    {
        //TODO: put this in the texture arena instead
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
VegetationLayerNV::buildStateSets()
{
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
            _tileWidths[lod] = width_m;
        }
    }

    Shaders shaders;

    // Layer-wide stateset:
    osg::StateSet* stateset = getOrCreateStateSet();

    _textures->setBindingPoint(1);
    stateset->setAttribute(_textures, 1);

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->addGLSLExtension("GL_ARB_gpu_shader_int64");
    vp->setFunction("vegetation_vs", vs, ShaderComp::LOCATION_VERTEX_MODEL);
    vp->setFunction("vegetation_fs", fs, ShaderComp::LOCATION_FRAGMENT_COLORING);

    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
    {
        stateset->setDefine("OE_USE_ALPHA_TO_COVERAGE");
        stateset->setMode(GL_MULTISAMPLE, 1);
        stateset->setMode(GL_BLEND, 0);
        stateset->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);
    }

    // NEXT assemble the asset group statesets.
    if (AssetGroup::TREES < NUM_ASSET_GROUPS)
        configureTrees();

    if (AssetGroup::UNDERGROWTH < NUM_ASSET_GROUPS)
        configureUndergrowth();

    //_cullShader = new osg::Shader(
    //    osg::Shader::COMPUTE,
    //    cs);
    
    ShaderPackage pkg;
    pkg.add("", cs);

    if (getLifeMapLayer())
    {
        pkg.replace(
            "OE_LIFEMAP_TEX",
            getLifeMapLayer()->getSharedTextureUniformName());
        pkg.replace(
            "OE_LIFEMAP_MAT",
            getLifeMapLayer()->getSharedTextureMatrixUniformName());
    }

    std::string src = ShaderLoader::load("", pkg, getReadOptions());
    _cullShader = new osg::Shader(osg::Shader::COMPUTE, src);

#if 0
    // bind the noise sampler.
    stateset->setTextureAttribute(_noiseBinding.unit(), _noiseTex.get(), osg::StateAttribute::OVERRIDE);
    stateset->addUniform(new osg::Uniform(NOISE_SAMPLER, _noiseBinding.unit()));

    //TODO: do we still need these??
    bind(getColorLayer(),   "OE_GROUNDCOVER_COLOR_SAMPLER", "OE_GROUNDCOVER_COLOR_MATRIX", stateset);
    bind(getLifeMapLayer(), "OE_LIFEMAP_SAMPLER", "OE_LIFEMAP_MATRIX", stateset);
    bind(getBiomeLayer(),   "OE_BIOME_SAMPLER", "OE_BIOME_MATRIX", stateset);

    // disable backface culling to support shadow/depth cameras,
    // for which the geometry shader renders cross hatches instead of billboards.
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::PROTECTED);

    // Screen-space error uniform
    if (!_sseU.valid())
        _sseU = new osg::Uniform("oe_veg_sse", getMaxSSE());
    _sseU->set(getMaxSSE());
    stateset->addUniform(_sseU.get());

    // Load our compute shader
    GroundCoverShaders gc_shaders;

    // apply the shared layer bindings
    if (getColorLayer())
    {
        gc_shaders.replace(
            "OE_COLOR_SAMPLER",
            getColorLayer()->getSharedTextureUniformName());
        gc_shaders.replace(
            "OE_COLOR_MATRIX",
            getColorLayer()->getSharedTextureMatrixUniformName());
    }

    if (getLifeMapLayer())
    {
        gc_shaders.replace(
            "OE_LIFEMAP_SAMPLER",
            getLifeMapLayer()->getSharedTextureUniformName());
        gc_shaders.replace(
            "OE_LIFEMAP_MATRIX",
            getLifeMapLayer()->getSharedTextureMatrixUniformName());
    }

    if (getBiomeLayer())
    {
        gc_shaders.replace(
            "OE_BIOME_SAMPLER",
            getBiomeLayer()->getSharedTextureUniformName());
        gc_shaders.replace(
            "OE_BIOME_MATRIX",
            getBiomeLayer()->getSharedTextureMatrixUniformName());
    }

    //std::string computeSource = ShaderLoader::load(gc_shaders.Compute, shaders, getReadOptions());
    //osg::Program* compute = new osg::Program();
    //compute->setName("VegetationLayerNV:COMPUTE");
    //osg::Shader* computeShader = new osg::Shader(osg::Shader::COMPUTE, computeSource);
    //computeShader->setName(gc_shaders.Compute);
    //compute->addShader(computeShader);
    //_computeSS->setAttribute(compute, osg::StateAttribute::OVERRIDE);

    // make a 4-channel noise texture to use
    NoiseTextureFactory noise;
    _noiseTex = noise.create(256u, 4u);
#endif
}


void
VegetationLayerNV::configureTrees()
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

    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
    {
        trees._renderStateSet->setDefine("OE_USE_ALPHA_TO_COVERAGE");
        trees._renderStateSet->setMode(GL_MULTISAMPLE, 1);
        trees._renderStateSet->setMode(GL_BLEND, 0);
        trees._renderStateSet->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);
    }

    GroundCoverShaders shaders;
    shaders.load(trees_vp, shaders.Trees);

#if 1
    // functor for generating cross hatch geometry for trees:
    trees._createImposter = [](
        const osg::BoundingBox& b,
        std::vector<osg::Texture*>& textures)
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

            osg::StateSet* ss = geom[i]->getOrCreateStateSet();

            if (i == 0)
            {
                static const GLushort indices[12] = {
                    0,1,2,  2,3,0,
                    4,5,6,  6,7,4 };

                geom[i]->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 12, &indices[0]));

                osg::Vec3Array* verts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 8);

                // X plane
                (*verts)[0].set(b.xMin(), 0, b.zMin());
                (*verts)[1].set(b.xMax(), 0, b.zMin());
                (*verts)[2].set(b.xMax(), 0, b.zMax());
                (*verts)[3].set(b.xMin(), 0, b.zMax());

                // Y plane
                (*verts)[4].set(0, b.yMin(), b.zMin());
                (*verts)[5].set(0, b.yMax(), b.zMin());
                (*verts)[6].set(0, b.yMax(), b.zMax());
                (*verts)[7].set(0, b.yMin(), b.zMax());

                geom[i]->setVertexArray(verts);

                osg::Vec2Array* uv = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX, 8);
                (*uv)[0].set(0, 0);
                (*uv)[1].set(1, 0);
                (*uv)[2].set(1, 1);
                (*uv)[3].set(0, 1);

                (*uv)[4].set(0, 0);
                (*uv)[5].set(1, 0);
                (*uv)[6].set(1, 1);
                (*uv)[7].set(0, 1);
                geom[i]->setTexCoordArray(0, uv);

                if (textures.size() > 0)
                    ss->setTextureAttribute(0, textures[0], 1); // side albedo
                if (textures.size() > 1)
                    ss->setTextureAttribute(1, textures[1], 1); // side normal
            }
            else if (i == 1)
            {
                static const GLushort indices[6] = {
                    0,1,2,  2,3,0 };

                geom[i]->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 6, &indices[0]));

                double zmid = 0.5*(b.zMax() - b.zMin());
                osg::Vec3Array* verts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 4);

                // overhead:
                (*verts)[0].set(b.xMin(), b.yMin(), zmid);
                (*verts)[1].set(b.xMax(), b.yMin(), zmid);
                (*verts)[2].set(b.xMax(), b.yMax(), zmid);
                (*verts)[3].set(b.xMin(), b.yMax(), zmid);

                geom[i]->setVertexArray(verts);

                osg::Vec2Array* uv = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX, 4);
                (*uv)[0].set(0, 0);
                (*uv)[1].set(1, 0);
                (*uv)[2].set(1, 1);
                (*uv)[3].set(0, 1);
                geom[i]->setTexCoordArray(0, uv);

                if (textures.size() > 2)
                    ss->setTextureAttribute(0, textures[2], 1); // top albedo
                if (textures.size() > 3)
                    ss->setTextureAttribute(1, textures[3], 1); // top normal
            }
            group->addChild(geom[i]);
        }
        return group;
    };
#else
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
#endif
}

void
VegetationLayerNV::configureUndergrowth()
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
    undergrowth._createImposter = [](
        const osg::BoundingBox& bbox,
        std::vector<osg::Texture*>& textures)
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

unsigned
VegetationLayerNV::getNumTilesRendered() const
{
    return _lastTileBatchSize;
}

void
VegetationLayerNV::checkForNewAssets()
{
    OE_SOFT_ASSERT_AND_RETURN(getBiomeLayer() != nullptr, void());

    BiomeManager& biomeMan = getBiomeLayer()->getBiomeManager();

    // if the revision has not changed, bail out.
    if (_biomeRevision.exchange(biomeMan.getRevision()) == biomeMan.getRevision())
        return;

    // revision changed -- start loading new assets.

    osg::observer_ptr<VegetationLayerNV> layer_weakptr(this);

    auto load = [layer_weakptr](Cancelable* c) -> BiomeManager::Drawables
    {
        BiomeManager::Drawables result;
        osg::ref_ptr< VegetationLayerNV> layer;
        if (layer_weakptr.lock(layer))
        {
            BiomeManager::CreateImposterFunction func = [layer](AssetGroup::Type group, const osg::BoundingBox& bbox, std::vector<osg::Texture*>& textures) {
                return layer->options().group(group)._createImposter(bbox, textures);
                //return layer->createParametricGeometry(group, bbox, textures);
            };

            result = layer->getBiomeLayer()->getBiomeManager().getDrawables(
                layer->_textures.get(),
                func,
                layer->getReadOptions());
        }
        return result;
    };
    _newDrawables = Job().dispatch<BiomeManager::Drawables>(load);
}

//........................................................................

void
VegetationLayerNV::reset()
{
    _lastVisit.setReferenceTime(DBL_MAX);
    _lastVisit.setFrameNumber(~0U);
    _lastTileBatchSize = 0u;
    _cameraState.clear();

    OE_SOFT_ASSERT_AND_RETURN(getBiomeLayer(), void());

    BiomeManager& biomeMan = getBiomeLayer()->getBiomeManager();
    _biomeRevision = biomeMan.getRevision();
}

ChonkDrawable*
VegetationLayerNV::createDrawable(
    const TileKey& key,
    const osg::BoundingBox& bbox) const
{
    OE_PROFILING_ZONE;

    auto d = new ChonkDrawable();

    GeoImage lifemap;
    if (getLifeMapLayer())
    {
        //TODO:
        // This causes a frame stall. Options...
        // - Bakground job?
        // - only use if already in memory?
        // - Wait until if completes to return a drawable? 
        // - Return a placeholder drawable and regen later?
        lifemap = getLifeMapLayer()->createImage(key);
    }
    
    double bbox_width = (bbox.xMax() - bbox.xMin());
    double bbox_height = (bbox.yMax() - bbox.yMin());
    osg::Matrixf xform;

    constexpr int tries = 16384;
    constexpr int max_instances = 4096;

    using Index = RTree<double, double, 2>;
    Index index;
    Random prng(key.hash());

    std::vector<osg::Vec3d> map_points;
    std::vector<osg::Vec3f> local_points;
    std::vector<osg::Vec2f> uvs;
    std::vector<Chonk::Ptr> chonks;

    map_points.reserve(max_instances);
    local_points.reserve(max_instances);
    uvs.reserve(max_instances);
    chonks.reserve(max_instances);

    const GeoExtent& e = key.getExtent();

    osg::Vec4f lifemap_value;
    double local[2];
    std::vector<double> hits;
    const double s2inv = 1.0 / sqrt(2.0);

    for (int i = 0; i < tries && chonks.size() < max_instances; ++i)
    {
        Chonk::Ptr chonk = _drawables.objects[prng.next(_drawables.objects.size())];
        const osg::BoundingBox& box = chonk->getBound();
        float u = prng.next();
        float v = prng.next();

        float density = 1.0f;
        if (lifemap.valid())
        {
            lifemap.getReader()(lifemap_value, u, v);
            density = lifemap_value[LIFEMAP_DENSE];
        }
        if (density < 0.01f)
            continue;

        float radius = 0.5f*(box.xMax() - box.xMin());

        // adjust collision radius based on density:
        float search_radius = mix(radius*3.0f, radius*0.25f, density);

        local[0] = bbox.xMin() + u * bbox_width;
        local[1] = bbox.yMin() + v * bbox_height;
        if (index.KNNSearch(local, &hits, nullptr, 1, search_radius) == 0)
        {
            double r = radius * s2inv;
            double a_min[2] { local[0] - r, local[1] - r };
            double a_max[2] { local[0] + r, local[1] + r };
            index.Insert(a_min, a_max, radius);

            float rotation = prng.next() * 3.1415927 * 2.0;

            chonks.emplace_back(chonk);
            local_points.emplace_back(local[0], local[1], rotation);
            map_points.emplace_back(e.xMin() + u * e.width(), e.yMin() + v * e.height(), 0);
            uvs.emplace_back(u, v);
        }
    }

    // clamp them to the terrain
    _map->getElevationPool()->sampleMapCoords(
        map_points,
        Distance(),
        nullptr,
        nullptr);

    const osg::Vec3f ZAXIS(0, 0, 1);

    for (unsigned i = 0; i < map_points.size(); ++i)
    {
        xform.makeTranslate(local_points[i].x(), local_points[i].y(), map_points[i].z());
        xform.preMultRotate(osg::Quat(local_points[i].z(), ZAXIS));

        d->add(chonks[i], xform, uvs[i]);
    }

    return d;
}

void
VegetationLayerNV::cull(
    const TileBatch& batch,
    osg::NodeVisitor& nv) const
{
    if (_drawables.objects.empty())
        return;

    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
    
    // todo: mutex
    _lastVisit = *cv->getFrameStamp();

    CameraState& cs = _cameraState[cv->getCurrentCamera()];

    CameraState::TileCache new_tiles;
    ChonkDrawable::Vector tile_drawables;

    for (auto& batch_entry : batch.tiles())
    {
        const TileKey& key = batch_entry->getKey();
        Tile& tile = cs._tiles[key];

        // create if necessary:
        if (!tile._drawable.valid())
        {
            tile._drawable = createDrawable(key, batch_entry->getBBox());
        }

        if (tile._drawable.valid())
        {
            // update the MVM for this frame:
            tile._drawable->setModelViewMatrix(batch_entry->getModelViewMatrix());

            new_tiles[key] = tile;

            tile_drawables.push_back(tile._drawable.get());
        }
    }

    if (!tile_drawables.empty())
    {
        // intiailize the super-drawable:
        if (cs._superDrawable == nullptr)
        {
            cs._superDrawable = new ChonkDrawable();
            cs._superDrawable->setName("VegetationNV");
            cs._superDrawable->setDrawStateSet(getStateSet());
        }

        // Experiment:
        // inject a custom cull function:
        //cs._superDrawable->setCustomCullingShader(_cullShader.get());

        // assign the new set of tiles:
        cs._superDrawable->setChildren(tile_drawables);

        // finally, traverse it so OSG will draw it.
        cs._superDrawable->accept(nv);
    }

    // Purge old tiles.
    cs._tiles.swap(new_tiles);
}

void
VegetationLayerNV::resizeGLObjectBuffers(unsigned maxSize)
{
    //todo
    PatchLayer::resizeGLObjectBuffers(maxSize);
}

void
VegetationLayerNV::releaseGLObjects(osg::State* state) const
{
    //todo
    PatchLayer::releaseGLObjects(state);
}
