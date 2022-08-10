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
#include <osgEarth/LineDrawable>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Math>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Metrics>
#include <osgEarth/GLUtils>
#include <osgEarth/Chonk>
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
#include <osgUtil/SmoothingVisitor>

#include <cstdlib> // getenv
#include <random>

#define LC "[VegetationLayer] " << getName() << ": "

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

REGISTER_OSGEARTH_LAYER(vegetation, VegetationLayer);

// TODO LIST
//
//  - BUG: Close/Open VegLayer doesn't work
//  - FEATURE: automatically generate billboards? Imposters? Other?
//  - [IDEA] programmable SSE for models?
//  - [PERF] cull by "horizon" .. e.g., the lower you are, the fewer distant trees...?
//  - variable spacing or clumping by asset...?
//  - make the noise texture bindless as well? Stick it in the arena? Why not.

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

    conf.set("alpha_to_coverage", alphaToCoverage());

    //TODO: groups

    return conf;
}

void
VegetationLayer::Options::fromConfig(const Config& conf)
{
    // defaults:
    alphaToCoverage().setDefault(true);
    gravity().setDefault(0.125f);

    colorLayer().get(conf, "color_layer");
    biomeLayer().get(conf, "biomes_layer");    

    conf.get("alpha_to_coverage", alphaToCoverage());
    conf.get("gravity", gravity());

    // defaults for groups:
    groups().resize(NUM_ASSET_GROUPS);

    if (AssetGroup::TREES < NUM_ASSET_GROUPS)
    {
        groups()[AssetGroup::TREES].lod().setDefault(14);
        groups()[AssetGroup::TREES].enabled().setDefault(true);
        groups()[AssetGroup::TREES].castShadows().setDefault(true);
        groups()[AssetGroup::TREES].maxRange().setDefault(4000.0f);
        groups()[AssetGroup::TREES].instancesPerSqKm().setDefault(16384);
    }

    if (AssetGroup::UNDERGROWTH < NUM_ASSET_GROUPS)
    {
        groups()[AssetGroup::UNDERGROWTH].lod().setDefault(19);
        groups()[AssetGroup::UNDERGROWTH].enabled().setDefault(true);
        groups()[AssetGroup::UNDERGROWTH].castShadows().setDefault(false);
        groups()[AssetGroup::UNDERGROWTH].maxRange().setDefault(75.0f);
        groups()[AssetGroup::UNDERGROWTH].instancesPerSqKm().setDefault(524288);
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
            group_c.get("max_range", group.maxRange());
            group_c.get("instances_per_sqkm", group.instancesPerSqKm());
            group_c.get("lod", group.lod());
            group_c.get("cast_shadows", group.castShadows());

            if (group_c.value("lod") == "auto") {
                group.lod() = 0;
            }
        }
    }
}

//........................................................................

bool
VegetationLayer::LayerAcceptor::acceptLayer(
    osg::NodeVisitor& nv,
    const osg::Camera* camera) const
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
    // TODO:  this is what keeps the patch layer from drawing at all lods, I'm assuming it'll only do it at lod 14 and 19...
    return _layer->hasEnabledGroupAtLOD(key.getLOD());
}

//........................................................................

void
VegetationLayer::init()
{
    PatchLayer::init();

    _biomeRevision = 0;
    setAcceptCallback(new LayerAcceptor(this));

    _requestMultisampling = false;
    _multisamplingActivated = false;
}

VegetationLayer::~VegetationLayer()
{
    close();
}

Status
VegetationLayer::openImplementation()
{
    // GL version requirement
    if (Capabilities::get().getGLSLVersion() < 4.6f ||
        Capabilities::get().supportsNVGL() == false)
    {
        return Status(Status::ResourceUnavailable, "Requires NVIDIA GL 4.6");
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

    _lastVisit.setFrameNumber(~0);

    // make a 4-channel noise texture to use
    NoiseTextureFactory noise;
    _noiseTex = noise.create(256u, 4u);

    return PatchLayer::openImplementation();
}

Status
VegetationLayer::closeImplementation()
{
    releaseGLObjects(nullptr);
    reset();

    return PatchLayer::closeImplementation();
}

void
VegetationLayer::update(osg::NodeVisitor& nv)
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

            OE_INFO << LC << "timed out for inactivity." << std::endl;
        }

        checkForNewAssets();

        if (_newAssets.isAvailable())
        {
            ScopedMutexLock lock(_assets);
            _assets = std::move(_newAssets.release());
        }

        //Assets newAssets = _newAssets.release();
        //if (!newAssets.empty())
        //{
        //    ScopedMutexLock lock(_assets);
        //    _assets = std::move(newAssets);
        //}

        if (_requestMultisampling)
        {
            activateMultisampling();
            _requestMultisampling = false;
        }
    }
}

void
VegetationLayer::addVegetationInstance(const VegetationInstance& i)
{
    std::lock_guard<std::mutex> lk(_vegetationInstancesMutex);
    const Profile* profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    TileKey key = profile->createTileKey(i.location.x(), i.location.y(), 14);
    _vegetationInstances[key].push_back(i);
}

void
VegetationLayer::dirty()
{
    _tiles.scoped_lock([this]()
        {
            _tiles.clear(); 
            _placeholders.clear();
            _vegetationInstances.clear();
        });

    _cameraState.scoped_lock([this]()
        {
            _cameraState.clear();
        });
}

void
VegetationLayer::setSSEScales(const osg::Vec4f& value)
{
    if (_pixelScalesU.valid())
        _pixelScalesU->set(value);
}

osg::Vec4f
VegetationLayer::getSSEScales() const
{
    osg::Vec4f value;
    if (_pixelScalesU.valid())
        _pixelScalesU->get(value);
    return value;
}

void
VegetationLayer::setBiomeLayer(BiomeLayer* layer)
{
    _biomeLayer.setLayer(layer);
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
VegetationLayer::hasEnabledGroupAtLOD(unsigned lod) const
{
    for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
    {
        if (options().groups()[i].lod() == lod &&
            options().groups()[i].enabled() == true)
        {
            return true;
        }
    }
    return false;
}

AssetGroup::Type
VegetationLayer::getGroupAtLOD(unsigned lod) const
{
    for (int i = 0; i < NUM_ASSET_GROUPS; ++i)
    {
        if (options().groups()[i].lod() == lod)
            return (AssetGroup::Type)i;
    }
    return AssetGroup::UNDEFINED;
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
VegetationLayer::setEnabled(AssetGroup::Type type, bool value)
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);

    auto& group = options().group(type);
    group.enabled() = value;
}

bool
VegetationLayer::getEnabled(AssetGroup::Type type) const
{
    OE_HARD_ASSERT(type < NUM_ASSET_GROUPS);
    return options().group(type).enabled().get();
}

void
VegetationLayer::setGravity(float value)
{
    options().gravity() = value;
}

float
VegetationLayer::getGravity() const
{
    return options().gravity().get();
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

    _map = map;

    if (getBiomeLayer() == nullptr)
    {
        setStatus(Status::ResourceUnavailable, "No Biomes layer available in the Map");
        return;
    }

    /*
    if (getLifeMapLayer() == nullptr)
    {
        setStatus(Status::ResourceUnavailable, "No LifeMap available in the Map");
        return;
    }
    */
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

    _requestMultisampling = false;
    _multisamplingActivated = false;

    TerrainResources* res = engine->getResources();
    if (res)
    {
        // Get a binding for the noise texture. This is only used for wind, btw.
        // Perhaps make this mapnode-level global or stick it in the texture arena.
        if (_noiseBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_noiseBinding, this, "VegLayerNV noise sampler") == false)
            {
                setStatus(Status::ResourceUnavailable, "No texture unit available for noise sampler");
                return;
            }
        }

        // Compute LOD for each asset group if necessary.
        for (int g = 0; g < options().groups().size(); ++g)
        {
            Options::Group& group = options().group((AssetGroup::Type)g);
            if (group.lod() == 0)
            //if (!group.lod().isSet())
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
VegetationLayer::buildStateSets()
{
    if (!getBiomeLayer()) {
        OE_DEBUG << LC << "buildStateSets deferred.. biome layer not available" << std::endl;
        return;
    }
    
/*
    // TODO:  This is the reason the statesets weren't built...
    if (!getLifeMapLayer()) {
        OE_DEBUG << LC << "buildStateSets deferred.. lifemap layer not available" << std::endl;
        return;
    }
    */

    // NEXT assemble the asset group statesets.
    if (AssetGroup::TREES < NUM_ASSET_GROUPS)
        configureTrees();

    if (AssetGroup::UNDERGROWTH < NUM_ASSET_GROUPS)
        configureGrass();

    osg::StateSet* ss = getOrCreateStateSet();
    ss->setName(typeid(*this).name());

    // Backface culling should be off for impostors.
    ss->setMode(GL_CULL_FACE, 0x0 | osg::StateAttribute::PROTECTED);

    // Install the texture arena:
    TextureArena* textures = getBiomeLayer()->getBiomeManager().getTextures();
    textures->setBindingPoint(1);
    ss->setAttribute(textures);

    // Custom shaders:
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    ProceduralShaders shaders;
    shaders.load(vp, shaders.Vegetation);

    // bind the noise sampler.
    ss->setTextureAttribute(_noiseBinding.unit(), _noiseTex.get(), 1);
    ss->addUniform(new osg::Uniform("oe_veg_noise", _noiseBinding.unit()));

    // If multisampling is on, use alpha to coverage.
    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
    {
        activateMultisampling();
    }

    // Far pixel scale overrides.
    _pixelScalesU = new osg::Uniform("oe_lod_scale", osg::Vec4f(1, 1, 1, 1));
    ss->addUniform(_pixelScalesU.get(), osg::StateAttribute::OVERRIDE | 0x01);
}

void
VegetationLayer::activateMultisampling()
{
    osg::StateSet* ss = getOrCreateStateSet();
    ss->setDefine("OE_USE_ALPHA_TO_COVERAGE");
    ss->setMode(GL_MULTISAMPLE, 1);
    ss->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);
    ss->setAttributeAndModes(new osg::BlendFunc(), 0 | osg::StateAttribute::OVERRIDE);

    _multisamplingActivated = true;
}

void
VegetationLayer::configureTrees()
{
    auto& trees = options().group(AssetGroup::TREES);

    // functor for generating cross hatch geometry for trees:
    trees._createImpostor = [](
        const osg::BoundingBox& b,
        std::vector<osg::Texture*>& textures)
    {
        osg::Group* group = new osg::Group();
        osg::Geometry* geom[2];

        // one part if we only have side textures;
        // two parts if we also have top textures
        int parts = textures.size() > 2 ? 2 : 1;

        float xmin = std::min(b.xMin(), b.yMin());
        float ymin = xmin;
        float xmax = std::max(b.xMax(), b.yMax());
        float ymax = xmax;

        for (int i = 0; i < parts; ++i)
        {
            geom[i] = new osg::Geometry();
            geom[i]->setName("Tree impostor");
            geom[i]->setUseVertexBufferObjects(true);
            geom[i]->setUseDisplayList(false);

            osg::StateSet* ss = geom[i]->getOrCreateStateSet();

            const osg::Vec4f colors[1] = {
                {1,1,1,1}
            };

            if (i == 0)
            {
                static const GLushort indices[12] = {
                    0,1,2,  2,3,0,
                    4,5,6,  6,7,4 };

                const osg::Vec3f verts[8] = {
                    { xmin, 0, b.zMin() },
                    { xmax, 0, b.zMin() },
                    { xmax, 0, b.zMax() },
                    { xmin, 0, b.zMax() },
                    { 0, ymin, b.zMin() },
                    { 0, ymax, b.zMin() },
                    { 0, ymax, b.zMax() },
                    { 0, ymin, b.zMax() }
                };

                osg::Vec3f normals[8] = {
                    {-1,0,1}, {1,0,1}, {1,0,2}, {-1,0,2},
                    {0,-1,1}, {0,1,1}, {0,1,2}, {0,-1,2}
                };
                for (int i = 0; i < 8; ++i) normals[i].normalize();

                const osg::Vec2f uvs[8] = {
                    {0,0},{1,0},{1,1},{0,1},
                    {0,0},{1,0},{1,1},{0,1}
                };

                const osg::Vec3f flexors[8] = {
                    {0,0,0}, {0,0,0}, {1,0,1}, {-1,0,1},
                    {0,0,0}, {0,0,0}, {0,1,1}, {0,-1,1}
                };

                geom[i]->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 12, &indices[0]));
                geom[i]->setVertexArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 8, verts));
                geom[i]->setNormalArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 8, normals));
                geom[i]->setColorArray(new osg::Vec4Array(osg::Array::BIND_OVERALL, 1, colors));
                geom[i]->setTexCoordArray(0, new osg::Vec2Array(osg::Array::BIND_PER_VERTEX, 8, uvs));
                geom[i]->setTexCoordArray(3, new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 8, flexors));

                if (textures.size() > 0)
                    ss->setTextureAttribute(0, textures[0], 1); // side albedo

                // No normal texture support - it looks much better without it.
                //if (textures.size() > 1)
                //    ss->setTextureAttribute(1, textures[1], 1); // side normal
            }
            else if (i == 1)
            {
                float zmid = 0.33f*(b.zMax() - b.zMin());

                static const GLushort indices[6] = {
                    0,1,2,  2,3,0
                };
                const osg::Vec3f verts[4] = {
                    {xmin, ymin, zmid},
                    {xmax, ymin, zmid},
                    {xmax, ymax, zmid},
                    {xmin, ymax, zmid}
                };
                osg::Vec3f normals[4] = {
                    {-1,-1,2}, {1,-1,2}, {1,1,2}, {-1,1,2}
                };
                for (int i = 0; i < 4; ++i) normals[i].normalize();

                const osg::Vec2f uvs[4] = {
                    {0,0}, {1,0}, {1,1}, {0,1}
                };

                geom[i]->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 6, &indices[0]));
                geom[i]->setVertexArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 4, verts));
                geom[i]->setNormalArray(new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 4, normals));
                geom[i]->setColorArray(new osg::Vec4Array(osg::Array::BIND_OVERALL, 1, colors));
                geom[i]->setTexCoordArray(0, new osg::Vec2Array(osg::Array::BIND_PER_VERTEX, 4, uvs));
                geom[i]->setTexCoordArray(3, new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 4)); // flexors

                if (textures.size() > 2)
                    ss->setTextureAttribute(0, textures[2], 1); // top albedo

                // No normal texture support - it looks much better without it.
                //if (textures.size() > 3)
                //    ss->setTextureAttribute(1, textures[3], 1); // top normal
            }
            group->addChild(geom[i]);
        }
        return group;
    };

    getBiomeLayer()->getBiomeManager().setCreateFunction(
        AssetGroup::TREES,
        trees._createImpostor);
}

void
VegetationLayer::configureGrass()
{
    auto& grass = options().group(AssetGroup::UNDERGROWTH);

    float gravity = options().gravity().get();

    // functor for generating billboard geometry for grass:
    grass._createImpostor = [gravity](
        const osg::BoundingBox& bbox,
        std::vector<osg::Texture*>& textures)
    {
        constexpr unsigned vertsPerInstance = 16;
        constexpr unsigned indiciesPerInstance = 54;

        osg::Geometry* out_geom = new osg::Geometry();
        out_geom->setName("Grass impostor");
        out_geom->setUseVertexBufferObjects(true);
        out_geom->setUseDisplayList(false);

        static const GLushort indices[indiciesPerInstance] = {
            0,1,4, 4,1,5, 1,2,5, 5,2,6, 2,3,6, 6,3,7,
            4,5,8, 8,5,9, 5,6,9, 9,6,10, 6,7,10, 10,7,11,
            8,9,12, 12,9,13, 9,10,13, 13,10,14, 10,11,14, 14,11,15
        };
        out_geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, indiciesPerInstance, &indices[0]));

        osg::Vec3Array* verts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        verts->reserve(vertsPerInstance);
        out_geom->setVertexArray(verts);

        const float th = 1.0f / 3.0f;
        const float x0 = -0.5f;
        for (int z = 0; z < 4; ++z)
        {
            verts->push_back({ x0+0.0f,    th, float(z)*th });
            verts->push_back({ x0+th,    0.0f, float(z)*th });
            verts->push_back({ x0+th+th, 0.0f, float(z)*th });
            verts->push_back({ x0+1.0f,    th, float(z)*th });
        }

        osg::Vec2Array* uvs = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);
        uvs->reserve(vertsPerInstance);
        out_geom->setTexCoordArray(0, uvs);
        for (int z = 0; z < 4; ++z)
        {
            uvs->push_back({ 0.0f,    float(z)*th });
            uvs->push_back({ th,      float(z)*th });
            uvs->push_back({ th + th, float(z)*th });
            uvs->push_back({ 1.0f,    float(z)*th });
        }

        const osg::Vec4f colors[1] = {
            {1,1,1,1}
        };
        out_geom->setColorArray(new osg::Vec4Array(1, colors), osg::Array::BIND_OVERALL);

        osg::Vec3f up(0, 0, 1);

        osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 16);
        out_geom->setNormalArray(normals);

        osg::Vec3Array* flex = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX, 16);
        out_geom->setTexCoordArray(3, flex);

        const osg::Vec3f face_vec(0, -1, 0);

        for (int i = 0; i < 16; ++i)
        {
            if (i < 4) {
                (*normals)[i] = up;
                (*flex)[i].set(0, 0, 0); // no flex
            }
            else {
                (*normals)[i] = up + ((*verts)[i] - (*verts)[i % 4]);
                (*normals)[i].normalize();
                (*flex)[i] = ((*verts)[i] - (*verts)[i - 4]);
                (*flex)[i].normalize();
                (*flex)[i] *= accel((*uvs)[i].y());
            }
        }

        osg::StateSet* ss = out_geom->getOrCreateStateSet();
        if (textures.size() > 0)
            ss->setTextureAttribute(0, textures[0], 1); // side albedo
        if (textures.size() > 1)
            ss->setTextureAttribute(1, textures[1], 1); // side normal

        return out_geom;
    };

    getBiomeLayer()->getBiomeManager().setCreateFunction(
        AssetGroup::UNDERGROWTH,
        grass._createImpostor);
}

bool
VegetationLayer::checkForNewAssets() const
{
    OE_SOFT_ASSERT_AND_RETURN(getBiomeLayer() != nullptr, false);

    BiomeManager& biomeMan = getBiomeLayer()->getBiomeManager();

    // if the revision has not changed, bail out.
    if (_biomeRevision.exchange(biomeMan.getRevision()) == biomeMan.getRevision())
        return false;

    // revision changed -- start loading new assets.

    osg::observer_ptr<const VegetationLayer> layer_weakptr(this);

    auto loadNewAssets = [layer_weakptr](Cancelable* c) -> Assets
    {
        OE_PROFILING_ZONE_NAMED("VegetationLayer::loadNewAssets(job)");

        Assets result;
        result.resize(NUM_ASSET_GROUPS);

        osg::ref_ptr<const VegetationLayer> layer;
        if (layer_weakptr.lock(layer))
        {
            BiomeManager::ResidentBiomes biomes = layer->getBiomeLayer()->getBiomeManager().getResidentBiomes(
                layer->getReadOptions());

            // re-organize the data into a form we can readily use.
            for (auto iter : biomes)
            {
                if (c && c->isCanceled())
                    break;

                const Biome* biome = iter.first;
                auto& instances = iter.second;

                for (int group = 0; group < NUM_ASSET_GROUPS; ++group)
                {
                    float total_weight = 0.0f;
                    float smallest_weight = FLT_MAX;

                    for (auto& instance : instances[group])
                    {
                        total_weight += instance.weight();
                        smallest_weight = std::min(smallest_weight, instance.weight());
                    }

                    // calculate the weight multiplier
                    float weight_scale = 1.0f / smallest_weight;
                    total_weight *= weight_scale;

                    for (auto& instance : instances[group])
                    {
                        unsigned num = std::max(1u, (unsigned)(instance.weight() * weight_scale));
                        for(unsigned i=0; i<num; ++i)
                            result[group][biome].push_back(instance);
                    }
                }
            }
        }
        return result;
    };

    auto arena = JobArena::get("oe.veg");
    _newAssets = Job(arena).dispatch<Assets>(loadNewAssets);

    return true;
}

//........................................................................

void
VegetationLayer::reset()
{
    _lastVisit.setReferenceTime(DBL_MAX);
    _lastVisit.setFrameNumber(~0U);

    _biomeRevision = 0;

    _assets.scoped_lock([this]()
        {
            _assets.clear();
        });

    _tiles.scoped_lock([this]()
        {
            _tiles.clear();
            _placeholders.clear();
        });

    _cameraState.scoped_lock([this]()
        {
            _cameraState.clear();
        });
}

// random-texture channels
#define N_SMOOTH   0
#define N_RANDOM   1
#define N_RANDOM_2 2
#define N_CLUMPY   3

Future<osg::ref_ptr<ChonkDrawable>>
VegetationLayer::createDrawableAsync(
    const TileKey& key_,
    const AssetGroup::Type& group_,
    const osg::BoundingBox& tile_bbox_) const
{
    osg::ref_ptr<const VegetationLayer> layer = this;
    TileKey key = key_;
    AssetGroup::Type group = group_;
    osg::BoundingBox tile_bbox = tile_bbox_;

    auto function =
        [layer, key, group, tile_bbox](Cancelable* c) -> osg::ref_ptr<ChonkDrawable>
    {
        osg::ref_ptr<ProgressCallback> p = new ProgressCallback(c);
        return layer->createDrawable(key, group, tile_bbox, p.get());
    };

    auto arena = JobArena::get("oe.veg");
    return Job(arena).dispatch<osg::ref_ptr<ChonkDrawable>>(function);
}


bool
VegetationLayer::getAssetPlacements(
    const TileKey& key,
    const AssetGroup::Type& group,
    bool loadBiomesOnDemand,
    std::vector<VegetationLayer::Placement>& output,
    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    // Just draw the tree layer for now b/c that is where we are storing the vegetation
    if (group != AssetGroup::TREES)
    {
        return false;
    }

    std::vector<Placement> result;

    // Safely copy the instance list. The object is immutable
    // once we get to this point, since all assets are materialized
    // by the biome manager.
    AssetsByBiome groupAssets;

    if (loadBiomesOnDemand == false)
    {
        ScopedMutexLock lock(_assets);

        if (_assets.size() <= group)
        {
            // data is unavailable.
            return false;
        }
        else
        {
            groupAssets = _assets[group]; // shallow copy
        }

        // if it's empty, bail out (and probably return later)
        if (groupAssets.empty())
        {
            OE_DEBUG << LC << "key=" << key.str() << "; asset list is empty for group " << group << std::endl;
            //return std::move(result);
            return false;
        }
    }

    // Make sure the biome we want is loaded.
    auto biome = getBiomeLayer()->getBiomeCatalog()->getBiome("Full");
    getBiomeLayer()->getBiomeManager().ref(biome);

    // If the biome residency is not up to date, do that now
    // after loading the biome map.
    if (loadBiomesOnDemand)
    {
        if (checkForNewAssets() == true)
        {
            _newAssets.join(progress);
            Assets newAssets = _newAssets.release();
            if (!newAssets.empty())
            {
                ScopedMutexLock lock(_assets);
                _assets = std::move(newAssets);
            }
        }

        // make a shallow copy of assets list safely
        {
            ScopedMutexLock lock(_assets);
            if (_assets.size() <= group)
                return false;
            else
                groupAssets = _assets[group]; // shallow copy
        }

        // if it's empty, bail out (and probably return later)
        if (groupAssets.empty())
        {
            output = std::move(result);
            return true;
        }
    }
    
    const GeoExtent& e = key.getExtent();

    osg::Matrixf xform;
    osg::Vec4f lifemap_value;
    osg::Vec4f biomemap_value;
    std::vector<double> hits;
    const double s2inv = 1.0 / sqrt(2.0);

    auto catalog = getBiomeLayer()->getBiomeCatalog();

    // determine a local tile bbox size for collisions and uv generation
    // note. This doesn't take elevation data into account. Does that matter?
    auto& ex = key.getExtent();
    GeoPoint centroid = ex.getCentroid();
    osg::ref_ptr<const SpatialReference> ltp = ex.getSRS()->createTangentPlaneSRS(centroid.vec3d());
    double x0, y0, x1, y1;
    ex.getSRS()->transform2D(ex.xMin(), ex.yMin(), ltp.get(), x0, y0);
    ex.getSRS()->transform2D(ex.xMax(), ex.yMax(), ltp.get(), x1, y1);
    osg::BoundingBox local_bbox(x0, y0, 0, x1, y1, 0);
    double local_width = x1 - x0;
    double local_height = y1 - y0;
    osg::Vec2d local;

    std::set<const Biome*> empty_biomes;

#if 0
    double width = ex.width();
    double height = ex.height();
    unsigned int count = 25;
    double dx = width / (double)count;
    double dy = height / (double)count;

    // store these separately so we can clamp them all in one go
    std::vector<osg::Vec3d> map_points;
    map_points.reserve(count * count);
    for (unsigned int c = 0; c < count; ++c)
    {
        for (unsigned int r = 0; r < count; ++r)
        {
            double x = ex.xMin() + (double)c * dx;
            double y = ex.yMin() + (double)r * dy;

            float u = (float)c / (float)count;
            float v = (float)r / (float)count;


            // fetch the collection of assets belonging to the selected biome:
            auto iter = groupAssets.find(biome);
            if (iter == groupAssets.end())
            {
                empty_biomes.insert(biome);
                continue;
            }

            auto& assetInstances = iter->second;

            // Get the first instance
            auto& instance = assetInstances[0];

            auto& asset = instance.residentAsset();

            // if there's no geometry... bye
            if (asset->chonk() == nullptr)
            {
                continue;
            }

            osg::Vec3d scale(1, 1, 1);

            // hack. assume an explicit width/height means this is a grass billboard..
            // TODO: find another way.
            bool isGrass = asset->assetDef()->width().isSet();
            if (isGrass)
            {
                // Grass imposter geometrh is 1x1, so scale it:
                scale.set(
                    asset->assetDef()->width().get(),
                    asset->assetDef()->width().get(),
                    asset->assetDef()->height().get());
            }

            // tile-local coordinates of the position:
            local.set(
                local_bbox.xMin() + u * local_width,
                local_bbox.yMin() + v * local_height);

            float rotation = 0.0f;

            map_points.emplace_back(
                x, y, 0
            );

            Placement p;
            p.localPoint() = local;
            p.uv().set(u, v);
            p.scale() = scale;
            p.rotation() = rotation;
            p.asset() = asset;

            result.emplace_back(std::move(p));
        }
    }
#else

    // Get the instances for this tile
    std::vector< VegetationInstance > tileInstances;
    {
        std::lock_guard<std::mutex> lk(const_cast<VegetationLayer*>(this)->_vegetationInstancesMutex);
        auto itr = _vegetationInstances.find(key);
        if (itr != _vegetationInstances.end())
        {
            tileInstances = itr->second;
        }
    }

    if (tileInstances.empty())
    {
        return false;
    }

    double width = ex.width();
    double height = ex.height();

    // store these separately so we can clamp them all in one go
    std::vector<osg::Vec3d> map_points;
    map_points.reserve(tileInstances.size());
    for (unsigned int i = 0; i < tileInstances.size(); ++i)
    {        
        auto &tileInstance = tileInstances[i];

        float u = (tileInstance.location.x() - ex.xMin()) / ex.width();
        float v = (tileInstance.location.y() - ex.yMin()) / ex.height();

        // fetch the collection of assets belonging to the selected biome:
        auto biomeIter = groupAssets.find(biome);
        auto& assetInstances = biomeIter->second;

        // Get the first instance
        //auto& instance = assetInstances[0];
        ResidentModelAssetInstance* instance = nullptr;
        for (auto& i : assetInstances)
        {
            if (i.residentAsset()->assetDef()->name() == tileInstance.assetName)
            {
                instance = &i;
                break;
            }
        }
        if (instance == nullptr)
        {
            continue;
        }

        // Find the instance 
        auto& asset = instance->residentAsset();

        // if there's no geometry... bye
        if (asset->chonk() == nullptr)
        {
            continue;
        }

        osg::Vec3d scale(1, 1, 1);

        // hack. assume an explicit width/height means this is a grass billboard..
        // TODO: find another way.
        bool isGrass = asset->assetDef()->width().isSet();
        if (isGrass)
        {
            // Grass imposter geometrh is 1x1, so scale it:
            scale.set(
                asset->assetDef()->width().get(),
                asset->assetDef()->width().get(),
                asset->assetDef()->height().get());
        }

        // tile-local coordinates of the position:
        local.set(
            local_bbox.xMin() + u * local_width,
            local_bbox.yMin() + v * local_height);

        float rotation = 0.0f;

        map_points.emplace_back(
            tileInstance.location.x(), tileInstance.location.y(), 0
        );

        Placement p;
        p.localPoint() = local;
        p.uv().set(u, v);
        p.scale() = scale;
        p.rotation() = rotation;
        p.asset() = asset;

        result.emplace_back(std::move(p));
    }
#endif
    
    // clamp everything to the terrain
    _map->getElevationPool()->sampleMapCoords(
        map_points,
        Distance(),
        nullptr,
        nullptr,
        0.0f); // store zero upon failure?     

    // copy the clamped map points back over
    for (std::size_t i = 0; i < map_points.size(); ++i)
    {
        result[i].mapPoint() = std::move(map_points[i]);
    }

    /*
    // print warnings about empty biomes
    if (!empty_biomes.empty())
    {
        for (auto biome : empty_biomes)
        {
            OE_WARN << "No assets defined in group " << AssetGroup::name(group) << " for biome " << biome->id().get() << std::endl;
        }
    }
    */

    output = std::move(result);
    return true;
}

osg::ref_ptr<ChonkDrawable>
VegetationLayer::createDrawable(
    const TileKey& key,
    const AssetGroup::Type& group,
    const osg::BoundingBox& tile_bbox,
    ProgressCallback* progress) const
{
    std::vector<VegetationLayer::Placement> placements;

    bool placementsOK = getAssetPlacements(
        key,
        group,
        false,
        placements,
        progress);

    if (!placementsOK)
    {
        // data not available...bail out and return later.
        progress->cancel();
        return nullptr;
    }

    const osg::Vec3f ZAXIS(0, 0, 1);

    // finally, assemble the drawable.
    osg::ref_ptr<ChonkDrawable> result = new ChonkDrawable();
    result->setName(key.str() + " Vegetation");

    for (auto& p : placements)
    {
        osg::Matrixd xform;

        if (p.mapPoint().z() < FLT_MAX)
        {
            xform.makeTranslate(p.localPoint().x(), p.localPoint().y(), p.mapPoint().z());
            xform.preMultRotate(osg::Quat(p.rotation(), ZAXIS));
            xform.preMultScale(p.scale());

            result->add(p.asset()->chonk(), xform, p.uv());
        }
    }

    return result;
}

void
VegetationLayer::cull(
    const TileBatch& batch,
    osg::NodeVisitor& nv) const
{
    // Make sure the biome is loaded
    auto biome = getBiomeLayer()->getBiomeCatalog()->getBiome("Full");
    getBiomeLayer()->getBiomeManager().ref(biome);

    if (_assets.empty())
        return;

    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

    // update the framestamp for auto-timeout support
    _lastVisit = *cv->getFrameStamp();

    // exclusive lock on the camera state (created a new one as necessary)
    _cameraState.lock();
    CameraState::Ptr& cs = _cameraState[cv->getCurrentCamera()];
    _cameraState.unlock();

    // check for multisampling
    if (!_multisamplingActivated)
    {
        if (cv->getState()->getLastAppliedModeValue(GL_MULTISAMPLE) ||
            osg::DisplaySettings::instance()->getMultiSamples() == true ||
            osg::DisplaySettings::instance()->getNumMultiSamples() > 1 ||
            options().alphaToCoverage() == true)
        {
            _requestMultisampling = true;
        }
    }

    if (cs == nullptr)
        cs = std::make_shared<CameraState>();

    CameraState::TileViews active_views;

    for (auto& entry : batch.tiles())
    {
        AssetGroup::Type group = getGroupAtLOD(entry->getKey().getLOD());
        OE_HARD_ASSERT(group != AssetGroup::UNDEFINED);

        if (options().group(group).enabled() == false)
        {
            continue;
        }

        if (CameraUtils::isShadowCamera(cv->getCurrentCamera()) &&
            options().group(group).castShadows() == false)
        {
            continue;
        }

        // combine the key and revision to make a Unique ID.
        TileKeyAndRevision rev_tile_key(
            { entry->getKey(), entry->getRevision() }
        );

        // find this camera's view on the tile, creating a slot if necessary:
        TileView& view = cs->_views[rev_tile_key];

        // If this camera doesn't have a view on the tile, establish one:
        if (view._tile == nullptr)
        {
            // We don't want more than one camera creating the
            // same drawable, so this tileJobs table tracks 
            // createDrawable jobs globally.
            ScopedMutexLock lock(_tiles);

            Tile::Ptr& tile = _tiles[rev_tile_key];
            if (tile == nullptr)
            {
                // new tile; create and fire off the loading job.
                tile = std::make_shared<Tile>();

                tile->_drawable = createDrawableAsync(
                    entry->getKey(),
                    group,
                    entry->getBBox());
            }

            view._tile = tile;
            view._matrix = new osg::RefMatrix();

            // in the meantime, get (or create) a placeholder
            // based on the same tile key, ignoring the revision;
            // then swap the new tile in as the next placeholder.
            view._placeholder = _placeholders[entry->getKey()];
        }

        // if the data is ready, cull it:
        if (view._tile->_drawable.isAvailable())
        {
            auto drawable = view._tile->_drawable.get();

            if (drawable.valid())
            {
                view._matrix->set(entry->getModelViewMatrix());
                cv->pushModelViewMatrix(view._matrix.get(), osg::Transform::ABSOLUTE_RF);
                view._tile->_drawable.get()->accept(nv);
                cv->popModelViewMatrix();

                // unref the old placeholder.
                if (!view._loaded)
                {
                    view._loaded = true;
                    view._placeholder = nullptr;

                    // update the placeholder for this tilekey.
                    _tiles.scoped_lock([&]()
                        {
                            _placeholders[entry->getKey()] = view._tile;
                        });
                }
            }
            else
            {
                // creation failed; reset for another try.
                view._tile = nullptr;
            }
        }

        // If the job exists but was canceled for some reason,
        // Reset this view so it will try again later.
        else if (view._tile->_drawable.isAbandoned())
        {
            view._tile = nullptr;
        }

        // Still loading, so draw a placeholder if we have one.
        // The placeholder is just an older version of the tile.
        else if (view._placeholder != nullptr)
        {
            view._matrix->set(entry->getModelViewMatrix());
            cv->pushModelViewMatrix(view._matrix.get(), osg::Transform::ABSOLUTE_RF);
            view._placeholder->_drawable.get()->accept(nv);
            cv->popModelViewMatrix();
        }

        if (view._tile)
        {
            active_views[rev_tile_key] = view;
        }
    }

    // Purge old tiles.
    cs->_views.swap(active_views);


    // purge unused tiles
    _tiles.scoped_lock([this]()
        {
            for (auto it = _tiles.begin(); it != _tiles.end(); )
            {
                if (it->second.use_count() == 1)
                    it = _tiles.erase(it);
                else
                    ++it;
            }
        });
}

void
VegetationLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    PatchLayer::resizeGLObjectBuffers(maxSize);

    ScopedMutexLock lock(_tiles);

    for (auto& tile : _tiles)
    {
        auto drawable = tile.second->_drawable.get();
        if (drawable.valid())
            drawable->resizeGLObjectBuffers(maxSize);
    }
}

void
VegetationLayer::releaseGLObjects(osg::State* state) const
{
    PatchLayer::releaseGLObjects(state);

    ScopedMutexLock lock(_tiles);

    for (auto& tile : _tiles)
    {
        auto drawable = tile.second->_drawable.get();
        if (drawable.valid())
            drawable->releaseGLObjects(state);
    }
}
