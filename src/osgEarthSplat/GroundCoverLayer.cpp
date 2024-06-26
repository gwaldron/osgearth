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
#include <osgEarth/TerrainEngineNode>
#include <osg/BlendFunc>
#include <osg/Multisample>
#include <osg/Texture2D>
#include <osg/Depth>
#include <osg/Version>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/CullVisitor>
#include <osgUtil/Optimizer>
#include <cstdlib> // getenv

#define LC "[GroundCoverLayer] " << getName() << ": "

#define GCTEX_SAMPLER "oe_GroundCover_billboardTex"
#define NOISE_SAMPLER "oe_GroundCover_noiseTex"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(groundcover, GroundCoverLayer);
REGISTER_OSGEARTH_LAYER(splat_groundcover, GroundCoverLayer);

// Test instanced model substitution.
// This works, but we need a compute or geometry shader or something to
// do culling, because everything makes its way tot the fragment shader
// and we're relying on discards to skip drawing
//#define TEST_MODEL_INSTANCING

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
    conf.set("wind_scale", windScale());

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
    windScale().setDefault(1.0f);

    maskLayer().get(conf, "mask_layer");
    colorLayer().get(conf, "color_layer");
    conf.get("color_min_saturation", colorMinSaturation());
    conf.get("lod", _lod);
    conf.get("cast_shadows", _castShadows);
    conf.get("max_alpha", maxAlpha());
    conf.get("alpha_to_coverage", alphaToCoverage());
    conf.get("wind_scale", windScale());

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
    // Trick to store the BiomeLayout pointer in a stateattribute so we can track it during cull/draw
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

            const SpatialReference* wgs84 = SpatialReference::get("wgs84");
            GeoPoint p;
            p.fromWorld(wgs84, vp);
                
            for(int z=_layer->getZones().size()-1; z > 0 && zoneIndex == 0; --z)
            {
                if ( _layer->getZones()[z].contains(p) )
                {
                    zoneIndex = z;
                    //OE_INFO << p.toString() << ", zoneIndex=" << zoneIndex << std::endl;
                }
            }

            osg::StateSet* zoneStateSet = _layer->getZoneStateSet(zoneIndex);
            //_layer->_perCamera[cv->getCurrentCamera]._currentZoneStateSet = zoneStateSet;

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

void
GroundCoverLayer::init()
{
    PatchLayer::init();

    _isModel = false;

    _debug = (::getenv("OSGEARTH_GROUNDCOVER_DEBUG") != NULL);

    _frameLastUpdate = 0U;

    // set a static bounding box buffer that can account for geometry in this layer.
    // 25 meters on the sides and 50m on the top will be a rough guess for now.
    _buffer.set(-25, -25, 0, 25, 25, 50);
}

GroundCoverLayer::~GroundCoverLayer()
{
    close();
}

Status
GroundCoverLayer::openImplementation()
{
    // GL version requirement
    if (Capabilities::get().getGLSLVersion() < 4.3f)
    {
        return Status(Status::ResourceUnavailable, "Requires GL 4.3+");
    }

    if (GLUtils::useNVGL())
    {
        return Status(Status::ResourceUnavailable, "Layer is not compatible with NVGL");
    }

    setAcceptCallback(new LayerAcceptor(this));
    setCullCallback(new ZoneSelector(this));

    // this layer will do its own custom rendering
    _renderer = std::make_shared<Renderer>(this);
    //setRenderer(_renderer.get());
    //setRenderer(_renderer.get());
    //setDrawCallback(_renderer.get());

    return PatchLayer::openImplementation();
}

Status
GroundCoverLayer::closeImplementation()
{
    releaseGLObjects(NULL);

    //setDrawCallback(NULL);
    //setRenderer(nullptr);
    _renderer = NULL;

    setAcceptCallback(NULL);
    setCullCallback(NULL);

    _zoneStateSets.clear();

    _noiseBinding.release();
    _groundCoverTexBinding.release();
    
    _liveAssets.clear();
    _atlasImages.clear();
    _atlas = nullptr;

    return PatchLayer::closeImplementation();
}

void
GroundCoverLayer::setLandCoverDictionary(LandCoverDictionary* layer)
{
    _landCoverDict.setLayer(layer);
    if (layer)
        buildStateSets();
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
    if (layer) {
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
GroundCoverLayer::update(osg::NodeVisitor& nv)
{
    int frame = nv.getFrameStamp()->getFrameNumber();

    if (frame > _frameLastUpdate &&
        _renderer && 
        _renderer->_frameLastActive > 0u &&
        (frame - _renderer->_frameLastActive) > 2)
    {
        releaseGLObjects(nullptr);
        _renderer->_frameLastActive = 0u;
    }
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

    // calculate the tile width based on the LOD:
    if (_renderer && getZones().size() > 0)
    {
        unsigned lod = getLOD();
        unsigned tx, ty;
        map->getProfile()->getNumTiles(lod, tx, ty);
        GeoExtent e = TileKey(lod, tx/2, ty/2, map->getProfile()).getExtent();
        GeoCircle c = e.computeBoundingGeoCircle();
        double width_m = 2.0 * c.getRadius() / 1.4142;
        _renderer->_tileWidth = width_m;

        //OE_INFO << LC << "Instances across = " << _renderer->_settings._vboTileSize << std::endl;
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
        if (_groundCoverTexBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_groundCoverTexBinding, this, "Ground cover texture catalog") == false)
            {
                OE_WARN << LC << "No texture unit available for ground cover texture catalog\n";
            }
        }

        if (_noiseBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_noiseBinding, this, "Ground cover noise sampler") == false)
            {
                OE_WARN << LC << "No texture unit available for Ground cover Noise function\n";
            }
        }

        if (_groundCoverTexBinding.valid())
        {
            buildStateSets();
        }
    }
}

//Returns true if any billboard in the data model uses a "top-down" image.
bool 
GroundCoverLayer::shouldEnableTopDownBillboards() const
{
    for(AssetDataVector::const_iterator i = _liveAssets.begin();
        i != _liveAssets.end();
        ++i)
    {
        if (i->get()->_topImage.valid())
            return true;
    }

    return false;
}


void
GroundCoverLayer::buildStateSets()
{
    // assert we have the necessary TIUs:
    if (_groundCoverTexBinding.valid() == false) {
        OE_DEBUG << LC << "buildStateSets deferred.. bindings not reserved\n";
        return;
    }

    if (!getLandCoverDictionary()) {
        OE_DEBUG << LC << "buildStateSets deferred.. land cover dictionary not available\n";
        return;
    }

    if (_liveAssets.empty())
    {
        loadAssets();
        // TODO: check for errors.
    }


    NoiseTextureFactory noise;
    osg::ref_ptr<osg::Texture> noiseTexture = noise.create(256u, 4u);

    GroundCoverShaders shaders;

    // Layer-wide stateset:
    osg::StateSet* stateset = getOrCreateStateSet();

    // bind the noise sampler.
    stateset->setTextureAttribute(_noiseBinding.unit(), noiseTexture.get());
    stateset->addUniform(new osg::Uniform(NOISE_SAMPLER, _noiseBinding.unit()));

    if (getMaskLayer())
    {
        stateset->setDefine("OE_GROUNDCOVER_MASK_SAMPLER", getMaskLayer()->getSharedTextureUniformName());
        stateset->setDefine("OE_GROUNDCOVER_MASK_MATRIX", getMaskLayer()->getSharedTextureMatrixUniformName());
    }

    if (getColorLayer())
    {
        stateset->setDefine("OE_GROUNDCOVER_COLOR_SAMPLER", getColorLayer()->getSharedTextureUniformName());
        stateset->setDefine("OE_GROUNDCOVER_COLOR_MATRIX", getColorLayer()->getSharedTextureMatrixUniformName());
        stateset->addUniform(new osg::Uniform("oe_GroundCover_colorMinSaturation", options().colorMinSaturation().get()));
    }

    // disable backface culling to support shadow/depth cameras,
    // for which the geometry shader renders cross hatches instead of billboards.
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::PROTECTED);

    stateset->addUniform(new osg::Uniform("oe_GroundCover_maxAlpha", getMaxAlpha()));

    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
    {
        stateset->setMode(GL_MULTISAMPLE, 1);
    }

    // Install LUTs on the compute shader:
    osg::ref_ptr<osg::Shader> lutShader = createLUTShader();
    lutShader->setName("GroundCover CS LUT");
    _renderer->_computeProgram->addShader(lutShader.get());

    // Install the land cover shaders on the state set
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->setName(typeid(*this).name());

    // Load shaders particular to this class
    loadShaders(vp, getReadOptions());

    // whether to support top-down image billboards. We disable it when not in use
    // for performance reasons.
    if (shouldEnableTopDownBillboards())
    {
        stateset->setDefine("OE_GROUNDCOVER_USE_TOP_BILLBOARDS");
    }

    // adjust the effect of wind with this factor
    stateset->setDefine("OE_GROUNDCOVER_WIND_SCALE", Stringify() << options().windScale().get());

    osg::Texture* tex = createTextureAtlas();
    stateset->setTextureAttribute(_groundCoverTexBinding.unit(), tex);
    stateset->addUniform(new osg::Uniform(GCTEX_SAMPLER, _groundCoverTexBinding.unit()));
    _atlas = tex;

    // Assemble zone-specific statesets:
    float maxVisibleRange = getMaxVisibleRange();
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
            maxVisibleRange = osg::minimum(maxVisibleRange, zone.options().maxDistance().get());
        }

        // keep in a vector so the ZoneSelector can pick one at cull time
        _zoneStateSets.push_back(zoneStateSet);
    }

    setMaxVisibleRange(maxVisibleRange);
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

    if (_renderer)
        _renderer->resizeGLObjectBuffers(maxSize);

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

    if (_renderer)
    {
        _renderer->releaseGLObjects(state);
    }

    PatchLayer::releaseGLObjects(state);

    if (isOpen() &&
        _atlas.valid() &&
        _atlas->getUnRefImageDataAfterApply() == false)
    {
        // Workaround for
        // https://github.com/openscenegraph/OpenSceneGraph/issues/1013
        for (unsigned i = 0; i < _atlas->getNumImages(); ++i)
            if (_atlas->getImage(i))
                _atlas->getImage(i)->dirty();
    }
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

#if 0
    osg::Geometry* loadShape()
    {
        //osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile("D:/data/models/rockinsoil/RockSoil.3DS.osg");
        osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile("D:/data/models/OakTree/redoak.osgb");
        
        InstanceCloud::ModelCruncher cruncher;
        cruncher.add(node.get());
        cruncher.finalize();
        
        return cruncher._geom.release();
    }
#endif
}

osg::Geometry*
GroundCoverLayer::createGeometry() const    
{
    osg::Geometry* out_geom = NULL;

    if (_isModel)
    {
        //out_geom = loadShape();
        //out_geom->setUseVertexBufferObjects(true);
        //out_geom->setUseDisplayList(false);
        return out_geom;
    }
    else
    {
        const unsigned vertsPerInstance = 8;
        const unsigned indiciesPerInstance = 12;

        osg::Geometry* out_geom = new osg::Geometry();
        out_geom->setUseVertexBufferObjects(true);

        static const GLushort indices[12] = { 0,1,2,2,1,3, 4,5,6,6,5,7 };
        out_geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 12, &indices[0]));

        return out_geom;
    }
}

//........................................................................

// only used with non-GS implementation
GroundCoverLayer::Renderer::UniformState::UniformState()
{
    // initialize all the uniform locations - we will fetch these at draw time
    // when the program is active
    _computeDataUL = -1;
    _A2CUL = -1;
    _tileCounter = 0;
    _numInstances1D = 0;
}

GroundCoverLayer::Renderer::Renderer(GroundCoverLayer* layer)
{
    _layer = layer;

    // create uniform IDs for each of our uniforms
    _A2CName = osg::Uniform::getNameID("oe_GroundCover_A2C");
    _computeDataUName = osg::Uniform::getNameID("oe_tile");

    _drawStateBuffer.resize(256u);

    _tileWidth = 0.0;

    _a2cBlending = new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO);

    // Load our compute shader
    GroundCoverShaders shaders;
    std::string source = ShaderLoader::load(shaders.GroundCover_CS, shaders, layer->getReadOptions());
    _computeStateSet = new osg::StateSet();
    osg::Shader* s = new osg::Shader(osg::Shader::COMPUTE, source);
    _computeProgram = new osg::Program();
    _computeProgram->addShader(s);
    _computeStateSet->setAttribute(_computeProgram, osg::StateAttribute::ON);

    _counter = 0;

    _frameLastActive = ~0U;
}

void
GroundCoverLayer::Renderer::draw(osg::RenderInfo& ri, const TileBatch& input)
{
    _frameLastActive = ri.getState()->getFrameStamp()->getFrameNumber();

    DrawState& ds = _drawStateBuffer[GLUtils::getUniqueStateID(*ri.getState())]; // ri.getContextID()];
    ds._renderer = this;
    osg::State* state = ri.getState();

    // Push the pre-gen culling shader and run it:
    const ZoneSA* sa = ZoneSA::extract(ri.getState());
    osg::ref_ptr<LegacyInstanceCloud>& instancer = ds._instancers[sa->_obj];
    if (!instancer.valid())
    {
        instancer = new LegacyInstanceCloud();
        ds._lastTileBatchID = -1;
        ds._uniforms.clear();
    }

    // Pull the per-camera data
    PerCameraData& pcd = _layer->_perCamera.get(ri.getCurrentCamera());

    // Only run the compute shader when the tile batch has changed:
    bool needsCompute = sa != pcd._previousZoneSA;
    pcd._previousZoneSA = sa;

    std::size_t batch_id(0);
    for (auto tile_ptr : input._tiles)
    {
        batch_id = hash_value_unsigned(
            batch_id,
            tile_ptr->getKey().hash(),
            (std::size_t)tile_ptr->getRevision());
    }

    if (ds._lastTileBatchID != batch_id)
    {
        ds._lastTileBatchID = batch_id;
        needsCompute = true;
    }

    // this can happen for a new instancer or after releaseGLobjects is called:
    if (instancer->_data.commands == nullptr)
    {
        needsCompute = true;
    }

    if (needsCompute)
    {
        // I'm not sure why we have to push the layer's stateset here.
        // It should have bee applied already in the render bin.
        // I am missing something. -gw 4/20/20
        
        state->pushStateSet(_layer->getStateSet());

        // First pass: render with compute shader
        state->apply(_computeStateSet.get());
        applyLocalState(ri, ds);

        instancer->allocateGLObjects(ri, input.tiles().size());
        instancer->preCull(ri);
        _pass = 0;
        for (auto tile_ptr : input.tiles())
        {
            tile_ptr->apply(ri, input.env());
            visitTile(ri, tile_ptr);
        }
        //tiles->drawTiles(ri);
        instancer->postCull(ri);

        // restore previous program
        state->apply();

        // rendering pass:
        applyLocalState(ri, ds);
        _pass = 1;
        for (auto tile_ptr : input.tiles())
        {
            tile_ptr->apply(ri, input.env());
            visitTile(ri, tile_ptr);
        }
        //tiles->drawTiles(ri);

        state->popStateSet();
    }

    else
    {
        applyLocalState(ri, ds);
        _pass = 1;
        for (auto tile_ptr : input.tiles())
        {
            tile_ptr->apply(ri, input.env());
            visitTile(ri, tile_ptr);
        }
        //tiles->drawTiles(ri);

        instancer->endFrame(ri);
    }

    // Clean up and finish - need to unbind everything so
    // OSG doesn't get confused.
    ri.getState()->unbindVertexArrayObject();
    ri.getState()->setLastAppliedProgramObject(nullptr);
    ri.getState()->unbindElementBufferObject();
    ri.getState()->unbindVertexArrayObject();
}

void
GroundCoverLayer::Renderer::applyLocalState(osg::RenderInfo& ri, DrawState& ds)
{
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp)
        return;

    auto cid = GLUtils::getSharedContextID(*ri.getState());
    osg::GLExtensions* ext = osg::GLExtensions::Get(cid, true);

    UniformState& u = ds._uniforms[pcp];

    if (u._computeDataUL < 0)
    {
        u._computeDataUL = pcp->getUniformLocation(_computeDataUName);
        u._A2CUL = pcp->getUniformLocation(_A2CName);
    }

    u._tileCounter = 0;

    // Check for initialization in this zone:
    const BiomeZone* bz = ZoneSA::extract(ri.getState())->_obj;
    osg::ref_ptr<LegacyInstanceCloud>& instancer = ds._instancers[bz];

    if (!instancer->getGeometry() || u._numInstances1D == 0)
    {
        if (bz->options().spacing().isSet())
        {
            float spacing_m = bz->options().spacing()->as(Units::METERS);
            u._numInstances1D = _tileWidth / spacing_m;
            _spacing = spacing_m;
        }
        else
        {
            u._numInstances1D = 64;
        }

        //OE_WARN << "Num Instances = " << u._numInstances1D*u._numInstances1D << std::endl;

        if (instancer->getGeometry() == NULL)
        {
            instancer->setGeometry(_layer->createGeometry());
            instancer->setNumInstances(u._numInstances1D, u._numInstances1D);

            // TODO: review this. I don't like it but have no good reason. -gw
            // This is here to integrate the model's texture atlas into the stateset
            if (instancer->_geom->getStateSet())
                _layer->getOrCreateStateSet()->merge(*instancer->_geom->getStateSet());
        }
    }

    GLint useA2C = 0;
    if (_layer->getUseAlphaToCoverage())
    {
        useA2C = ri.getState()->getLastAppliedMode(GL_MULTISAMPLE) ? 1 : 0;
        ri.getState()->applyMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, useA2C == 1);
        ri.getState()->applyAttribute(_a2cBlending.get());
    }

    if (u._A2CUL >= 0)
    {
        ext->glUniform1i(u._A2CUL, useA2C);
    }
}

void
GroundCoverLayer::Renderer::visitTile(osg::RenderInfo& ri, const TileState* tile)
{
    const ZoneSA* sa = ZoneSA::extract(ri.getState());
    DrawState& ds = _drawStateBuffer[GLUtils::getUniqueStateID(*ri.getState())];
    osg::ref_ptr<LegacyInstanceCloud>& instancer = ds._instancers[sa->_obj];
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();

    OE_SOFT_ASSERT_AND_RETURN(pcp != nullptr, void());

    UniformState& u = ds._uniforms[pcp];

    if (_pass == 0) // COMPUTE shader
    {
        auto cid = GLUtils::getSharedContextID(*ri.getState());
        osg::GLExtensions* ext = osg::GLExtensions::Get(cid, true);

        if (u._computeDataUL >= 0)
        {
            u._computeData[0] = tile->getBBox().xMin();
            u._computeData[1] = tile->getBBox().yMin();
            u._computeData[2] = tile->getBBox().xMax();
            u._computeData[3] = tile->getBBox().yMax();

            u._computeData[4] = (float)u._tileCounter;

            // TODO: check whether this changed before calling it
            ext->glUniform1fv(u._computeDataUL, 5, &u._computeData[0]);

            instancer->cullTile(ri, u._tileCounter);
        }
    }

    else // DRAW shader
    {
        // check valid flag
        instancer->drawTile(ri, u._tileCounter);
    }

    ++u._tileCounter;
}

void
GroundCoverLayer::Renderer::resizeGLObjectBuffers(unsigned maxSize)
{
    _drawStateBuffer.resize(osg::maximum(maxSize, _drawStateBuffer.size()));
}

void
GroundCoverLayer::Renderer::releaseGLObjects(osg::State* state) const
{
    if (state)
    {
        DrawState& ds = _drawStateBuffer[GLUtils::getUniqueStateID(*state)];

        ds._uniforms.clear();
        ds._lastTileBatchID = -1;

        for (const auto& i : ds._instancers)
        {
            LegacyInstanceCloud* cloud = i.second.get();
            if (cloud)
            {
                cloud->releaseGLObjects(state);
            }
        }
    }
    else
    {
        for (unsigned i = 0; i < _drawStateBuffer.size(); ++i)
        {
            DrawState& ds = _drawStateBuffer[i];

            ds._uniforms.clear();
            ds._lastTileBatchID = -1;

            for (const auto& i : ds._instancers)
            {
                LegacyInstanceCloud* cloud = i.second.get();
                if (cloud)
                {
                    cloud->releaseGLObjects(state);
                }
            }
        }
    }

    _computeStateSet->releaseGLObjects(state);
    _a2cBlending->releaseGLObjects(state);
}

void
GroundCoverLayer::loadShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders shaders;

    if (_isModel)
    {
        shaders.load(vp, shaders.GroundCover_Model, options);
    }
    else // billboards
    {
        shaders.load(vp, shaders.GroundCover_Billboard, options);
    }
}

namespace {
    template<typename T>
    int indexOf(const std::vector<osg::ref_ptr<T> >& v, T* obj) {
        for(int i=0; i<v.size(); ++i) {
            if (v[i].get() == obj) {
                return i;
            }
        }
        return -1;
    }
}

void
GroundCoverLayer::loadAssets()
{
    typedef std::map<URI, osg::ref_ptr<osg::Object> > Cache;
    Cache _cache;

    osg::ref_ptr<osg::Image> standIn = new osg::Image();

    int landCoverGroupIndex = 0;

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
                data->_sideImageAtlasIndex = -1;
                data->_topImageAtlasIndex = -1;

                if (asset.options().sideBillboardURI().isSet())
                {
                    const URI& uri = asset.options().sideBillboardURI().get();
                    Cache::iterator ic = _cache.find(uri);
                    if (ic != _cache.end())
                    {
                        data->_sideImage = dynamic_cast<osg::Image*>(ic->second.get());
                        data->_sideImageAtlasIndex = indexOf(_atlasImages, data->_sideImage.get());
                    }
                    else
                    {
                        data->_sideImage = uri.getImage(getReadOptions());
                        if (!data->_sideImage.valid())
                        {
                            OE_WARN << LC << "Failed to load billboard side image from \"" << uri.full() << "\"" << std::endl;
                            // it's mandatory, so make an empty placeholder:
                            data->_sideImage = standIn.get();
                        }
                        _cache[uri] = data->_sideImage.get();
                        data->_sideImageAtlasIndex = _atlasImages.size();
                       _atlasImages.push_back(data->_sideImage.get());
                       OE_DEBUG << LC << "Loaded \"" << uri.full() << "\"" << std::endl;
                    }
                }

                if (asset.options().topBillboardURI().isSet())
                {
                    const URI& uri = asset.options().topBillboardURI().get();
                    Cache::iterator ic = _cache.find(uri);
                    if (ic != _cache.end())
                    {
                        data->_topImage = dynamic_cast<osg::Image*>(ic->second.get());
                        data->_topImageAtlasIndex = indexOf(_atlasImages, data->_topImage.get());
                    }
                    else
                    {
                        data->_topImage = uri.getImage(getReadOptions());
                        if (data->_topImage.valid())
                        {
                            _cache[uri] = data->_topImage.get();
                            data->_topImageAtlasIndex = _atlasImages.size();
                            _atlasImages.push_back(data->_topImage.get());
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load billboard side image from \"" << uri.full() << "\"" << std::endl;
                        }
                    }
                }

                if (asset.options().modelURI().isSet())
                {
                    const URI& uri = asset.options().modelURI().get();
                    Cache::iterator ic = _cache.find(uri);
                    if (ic != _cache.end())
                    {
                        data->_model = dynamic_cast<osg::Node*>(ic->second.get());
                        //data->_modelAtlasIndex = indexOf(_atlasImages, data->_model.get());
                    }
                    else
                    {
                        data->_model = uri.getNode(getReadOptions());
                        if (data->_model.valid())
                        {
                            _cache[uri] = data->_model.get();
                            //TODO: Crunch the model and store its texture(s) in the model atlas..?
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load model from \"" << uri.full() << "\"" << std::endl;
                        }
                    }
                }

                if (data->_sideImage.valid() || data->_model.valid())
                {
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
}

osg::Texture*
GroundCoverLayer::createTextureAtlas() const
{
    // Creates a texture array containing all the billboard images.
    // Each image is included only once.
    osg::Texture2DArray* tex = new osg::Texture2DArray();

    int arrayIndex = 0;
    float s = -1.0f, t = -1.0f;

    for(unsigned i=0; i<_atlasImages.size(); ++i)
    {
        osg::Image* image = _atlasImages[i].get();

        osg::ref_ptr<osg::Image> temp;

        // make sure the texture array is POT - required now for mipmapping to work
        if (s < 0)
        {
            s = osgEarth::nextPowerOf2(image->s());
            t = osgEarth::nextPowerOf2(image->t());
            tex->setTextureSize(s, t, _atlasImages.size());
        }

        if (image->s() != s || image->t() != t)
        {
            ImageUtils::resizeImage(image, s, t, temp);
        }
        else
        {
            temp = image;
        }

        tex->setImage(i, temp.get());
    }

    OE_INFO << LC << "Created atlas with " << _atlasImages.size() << " unique images" << std::endl;

    tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
    tex->setWrap(tex->WRAP_S, tex->CLAMP_TO_EDGE);
    tex->setWrap(tex->WRAP_T, tex->CLAMP_TO_EDGE);
    tex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
    tex->setMaxAnisotropy(4.0);

    // Let the GPU do it since we only download this at startup
    tex->setUseHardwareMipMapGeneration(true);

    return tex;
}

namespace {

}

osg::Shader*
GroundCoverLayer::createLUTShader() const
{
    std::stringstream landCoverGroupBuf;
    landCoverGroupBuf << std::fixed << std::setprecision(2);

    std::stringstream assetBuf;
    assetBuf << std::fixed << std::setprecision(1);

    int numBiomeZones = options().biomeZones().size();
    int numBiomeLayouts = numBiomeZones; // one per zone.
    int numLandCoverGroups = 0;
    int numAssets = 0;
    for(int i=0; i<numBiomeZones; ++i)
    {
        const BiomeZone& bz = options().biomeZones()[i];
        numLandCoverGroups += bz.getLandCoverGroups().size();
        // todo.
    }

    // encode all the biome data.

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

            startingAssetIndex = a;
            numAssetsInLandCoverGroup = 0;
            currentLandCoverGroupIndex = data->_landCoverGroupIndex;
            currentLandCoverGroup = data->_landCoverGroup;
        }

        // record an asset:
        float width = data->_asset->options().width().get();

        float height = data->_asset->options().height().get();

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
                << data->_sideImageAtlasIndex
                << ", " << data->_topImageAtlasIndex
                << ", " << width
                << ", " << height
                << ", " << sizeVariation
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
        << "); \n";

    std::stringstream assetWrapperBuf;
    assetWrapperBuf <<
        "struct oe_gc_Asset { \n"
        "    int atlasIndexSide; \n" // or 3D model texture..todo
        "    int atlasIndexTop; \n"
        "    float width; \n"
        "    float height; \n"
        "    float sizeVariation; \n"
        "}; \n"
        "const oe_gc_Asset oe_gc_assets[" << numAssetInstancesAdded << "] = oe_gc_Asset[" << numAssetInstancesAdded << "]( \n"
        << assetBuf.str()
        << "); \n";

    // next, create the master LUT.
    std::stringstream lutbuf;

    lutbuf << assetWrapperBuf.str() << "\n";

    lutbuf << landCoverGroupWrapperBuf.str() << "\n";

    lutbuf <<
        "bool oe_gc_getLandCoverGroup(in int zone, in int code, out oe_gc_LandCoverGroup result) { \n";

    std::unordered_set<std::string> exprs;
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
        << "#version " << std::to_string(Capabilities::get().getGLSLVersionInt()) << "\n"
        << lutbuf.str() << "\n");

    return shader;
}

osg::StateSet*
GroundCoverLayer::getZoneStateSet(unsigned index) const
{
    return index < _zoneStateSets.size() ? _zoneStateSets[index].get() : NULL;
}
