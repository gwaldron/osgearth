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
#include <osgUtil/CullVisitor>
#include <osgEarth/LineDrawable>
#include <osgEarth/NodeUtils>
#include <osg/BlendFunc>
#include <osg/Multisample>
#include <osg/Texture2D>
#include <osg/Depth>
#include <osg/Version>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
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
GroundCoverLayer::Options::fromConfig(const Config& conf)
{
    // defaults:
    lod().init(13u);
    castShadows().init(false);
    maxAlpha().init(0.15f);
    alphaToCoverage().init(true);

    maskLayer().get(conf, "mask_layer");
    colorLayer().get(conf, "color_layer");
    conf.get("color_min_saturation", colorMinSaturation());
    conf.get("lod", _lod);
    conf.get("cast_shadows", _castShadows);
    conf.get("max_alpha", maxAlpha());
    conf.get("alpha_to_coverage", alphaToCoverage());

    const Config* zones = conf.child_ptr("zones");
    if (zones)
    {
        const ConfigSet& children = zones->children();
        for (ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i)
        {
            _zones.push_back(ZoneOptions(*i));
        }
    }
    else // no zones, load GC directly
    {
        optional<GroundCoverOptions> gc;
        conf.get("groundcover", gc);
        if (gc.isSet())
        {
            ZoneOptions zo;
            zo.groundCover() = gc;
            _zones.push_back(zo);
        }
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
    bool isDepthCamera = CameraUtils::isDepthCamera(camera);
    if (isDepthCamera)
        return false;

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
    struct GroundCoverSA : public osg::StateAttribute
    {
        META_StateAttribute(osgEarth, GroundCoverSA, (osg::StateAttribute::Type)(osg::StateAttribute::CAPABILITY + 90210));
        GroundCover* _groundcover;
        GroundCoverSA() : _groundcover(NULL) { }
        GroundCoverSA(const GroundCoverSA& sa, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY) : osg::StateAttribute(sa, copyop), _groundcover(sa._groundcover) { }
        GroundCoverSA(GroundCover* gc) : _groundcover(gc) { }
        virtual int compare(const StateAttribute& sa) const { return 0; }
        static const GroundCoverSA* extract(const osg::State* state) {
            osg::State::AttributeMap::const_iterator i = state->getAttributeMap().find(
                std::make_pair((osg::StateAttribute::Type)(osg::StateAttribute::CAPABILITY + 90210), 0));
            if (i == state->getAttributeMap().end()) return NULL;
            if (i->second.attributeVec.empty()) return NULL;
            return dynamic_cast<const GroundCoverSA*>(i->second.attributeVec.front().first);
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
            GroundCover* gc = _layer->_zones[zoneIndex]->getGroundCover();
            if (gc)
            {
                zoneStateSet = gc->getStateSet();
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

OE_LAYER_PROPERTY_IMPL(GroundCoverLayer, unsigned, LOD, lod);
OE_LAYER_PROPERTY_IMPL(GroundCoverLayer, bool, CastShadows, castShadows);

void
GroundCoverLayer::init()
{
    PatchLayer::init();

    _zonesConfigured = false;

    // deserialize zone data
    for (std::vector<ZoneOptions>::const_iterator i = options().zones().begin();
        i != options().zones().end();
        ++i)
    {
        osg::ref_ptr<Zone> zone = new Zone(*i);
        _zones.push_back(zone.get());
    }

    setAcceptCallback(new LayerAcceptor(this));

    setCullCallback(new ZoneSelector(this));

#ifndef USE_GEOMETRY_SHADER
    // this layer will do its own custom rendering
    _renderer = new Renderer(this);
    setDrawCallback(_renderer.get());
#endif

    _debug = (::getenv("OSGEARTH_GROUNDCOVER_DEBUG") != NULL);

    installDefaultOpacityShader();
}

Status
GroundCoverLayer::openImplementation()
{
    Status parent = PatchLayer::openImplementation();
    if (parent.isError())
        return parent;

    return Status::NoError;
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

    for (Zones::iterator zone = _zones.begin(); zone != _zones.end(); ++zone)
    {
        zone->get()->configure(map, getReadOptions());
    }

    _zonesConfigured = true;

    // calculate the tile width based on the LOD:
    if (_renderer.valid() && _zones.size() > 0)
    {
        unsigned lod = getLOD();
        unsigned tx, ty;
        map->getProfile()->getNumTiles(lod, tx, ty);
        GeoExtent e = TileKey(lod, tx/2, ty/2, map->getProfile()).getExtent();
        GeoCircle c = e.computeBoundingGeoCircle();
        double width_m = 2.0 * c.getRadius() / 1.4142;
        _renderer->_settings._tileWidth = width_m;

        //OE_INFO << LC << "Instances across = " << _renderer->_settings._vboTileSize << std::endl;
    }

    buildStateSets();
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

namespace
{
    // Returns true if any billboard in the data model uses a "top-down" image.
    bool groundCoverModelUsesTopImages(const GroundCover* gc)
    {
        for(int i=0; i<gc->getBiomes().size(); ++i)
        {
            const GroundCoverBiome* biome = gc->getBiomes()[i].get();

            for(int j=0; j<biome->getObjects().size(); ++j)
            {
                const GroundCoverObject* object = biome->getObjects()[j].get();

                if (object->getType() == GroundCoverObject::TYPE_BILLBOARD)
                {
                    const GroundCoverBillboard* bb = dynamic_cast<const GroundCoverBillboard*>(object);
                    if (bb && bb->_topImage.valid())
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
}

void
GroundCoverLayer::buildStateSets()
{
    // assert we have the necessary TIUs:
    if (_groundCoverTexBinding.valid() == false) {
        OE_DEBUG << LC << "buildStateSets deferred.. bindings not reserved\n";
        return;
    }

    if (!_zonesConfigured) {
        OE_DEBUG << LC << "buildStateSets deferred.. zones not yet configured\n";
        return;
    }

    if (!getLandCoverDictionary()) {
        OE_DEBUG << LC << "buildStateSets deferred.. land cover dictionary not available\n";
        return;
    }

    //if (!getLandCoverLayer()) {
    //    OE_DEBUG << LC << "buildStateSets deferred.. land cover layer not available\n";
    //    return;
    //}

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

    float maxRangeAcrossZones = getMaxVisibleRange();

    for (Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        Zone* zone = z->get();
        GroundCover* groundCover = zone->getGroundCover();
        if (groundCover)
        {
            if (!groundCover->getBiomes().empty() || groundCover->getTotalNumObjects() > 0)
            {
                osg::StateSet* zoneStateSet = groundCover->getOrCreateStateSet();

                float maxDistance = osg::minimum(
                    getMaxVisibleRange(),
                    groundCover->getMaxDistance());

                zoneStateSet->addUniform(new osg::Uniform("oe_GroundCover_maxDistance", maxDistance));

                maxRangeAcrossZones = osg::maximum(maxRangeAcrossZones, maxDistance);

                zoneStateSet->addUniform(new osg::Uniform("oe_GroundCover_maxAlpha", getMaxAlpha()));

                if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
                {
                    zoneStateSet->setMode(GL_MULTISAMPLE, 1);
                }

                // Install the land cover shaders on the state set
                VirtualProgram* vp = VirtualProgram::getOrCreate(zoneStateSet);
                vp->setName("Ground cover (" + groundCover->getName() + ")");

                loadShaders(vp, getReadOptions());

                osg::Shader* covTest = groundCover->createPredicateShader(getLandCoverDictionary(), getLandCoverLayer());
                covTest->setName(covTest->getName() + "_VERTEX");
                covTest->setType(osg::Shader::VERTEX);
                vp->setShader(covTest);

                // for compute:
                _renderer->_computeProgram->addShader(new osg::Shader(osg::Shader::COMPUTE, covTest->getShaderSource()));

                osg::ref_ptr<osg::Shader> layerShader = groundCover->createShader();
                if (!layerShader.valid())
                {
                    setStatus(Status::ConfigurationError, "No GroundCover objects available");
                    setDrawCallback(NULL);
                    return;
                }
                layerShader->setType(osg::Shader::VERTEX);
                vp->setShader(layerShader.get());

                // for compute:                
                _renderer->_computeProgram->addShader(new osg::Shader(osg::Shader::COMPUTE, layerShader->getShaderSource()));

                // whether to support top-down image billboards. We disable it when not in use
                // for performance reasons.
                if (groundCoverModelUsesTopImages(groundCover))
                {
                    zoneStateSet->setDefine("OE_GROUNDCOVER_USE_TOP_BILLBOARDS");
                }

                OE_INFO << LC << "Established zone \"" << zone->getName() << "\" at LOD " << getLOD() << "\n";

                osg::Texture* tex = groundCover->createTexture();

                zoneStateSet->setTextureAttribute(_groundCoverTexBinding.unit(), tex);
                zoneStateSet->addUniform(new osg::Uniform(GCTEX_SAMPLER, _groundCoverTexBinding.unit()));

                zoneStateSet->setAttribute(new GroundCoverSA(groundCover));
            }
            else
            {
                OE_WARN << LC << "ILLEGAL: ground cover layer with no biomes or no billboards defined\n";
            }
        }
        else
        {
            // not an error.
            OE_DEBUG << LC << "zone contains no ground cover information\n";
        }
    }

    setMaxVisibleRange(maxRangeAcrossZones);
    OE_INFO << LC << "Max visible range set to " << maxRangeAcrossZones << std::endl;
}

void
GroundCoverLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    for (Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        z->get()->resizeGLObjectBuffers(maxSize);
    }

    if (_renderer.valid())
        _renderer->resizeGLObjectBuffers(maxSize);

    PatchLayer::resizeGLObjectBuffers(maxSize);
}

void
GroundCoverLayer::releaseGLObjects(osg::State* state) const
{
    for (Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        z->get()->releaseGLObjects(state);
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

    osg::Geometry* loadShape()
    {
        osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile("D:/data/models/rockinsoil/RockSoil.3DS.osg");
        osg::ref_ptr<osg::Geometry> geom = osgEarth::findTopMostNodeOfType<osg::Geometry>(node.get());
        node = NULL;
        if (!geom.valid()) OE_WARN << "ASDLAKSDJALSKDJALSKJASLKDJ" << std::endl;
        return geom.release();
    }
}

osg::Node*
GroundCoverLayer::createNodeImplementation(const DrawContext& dc)
{
    osg::Node* node = NULL;
    if (_debug)
        node = makeBBox(dc._geom->getBoundingBox(), *dc._key);
    return node;
}

osg::Geometry*
GroundCoverLayer::createGeometry(unsigned vboTileDim) const    
{
    unsigned numInstances = vboTileDim * vboTileDim;
    const unsigned vertsPerInstance = 8;
    const unsigned indiciesPerInstance = 12;

    osg::Geometry* out_geom = new osg::Geometry();
    out_geom->setUseVertexBufferObjects(true);

#ifdef TEST_MODEL_INSTANCING
    out_geom = makeShape();
    //out_geom = loadShape();
    out_geom->getPrimitiveSet(0)->setNumInstances(numInstances);
    return out_geom;
#endif

    static const GLushort indices[12] = { 0,1,2,2,1,3, 4,5,6,6,5,7 };
    out_geom->addPrimitiveSet(new osg::DrawElementsUShort(GL_TRIANGLES, 12, &indices[0], numInstances));

    // We don't actually need any verts. Is it OK not to set an array?
    //geom->setVertexArray(new osg::Vec3Array(8));

    //osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
    //normals->push_back(osg::Vec3f(0,0,1));
    //out_geom->setNormalArray(normals);

    return out_geom;
}

//........................................................................

// only used with non-GS implementation
GroundCoverLayer::Renderer::UniformState::UniformState()
{
    // initialize all the uniform locations - we will fetch these at draw time
    // when the program is active
    _LLUL = -1;
    _URUL = -1;
    _A2CUL = -1;
    _tileNumUL = -1;
    _tileCounter = 0;
    _numInstances1D = 0;
}

GroundCoverLayer::Renderer::Renderer(GroundCoverLayer* layer)
{
    _layer = layer;

    // create uniform IDs for each of our uniforms
    _LLUName = osg::Uniform::getNameID("oe_GroundCover_LL");
    _URUName = osg::Uniform::getNameID("oe_GroundCover_UR");
    _A2CName = osg::Uniform::getNameID("oe_GroundCover_A2C");
    _tileNumUName = osg::Uniform::getNameID("oe_GroundCover_tileNum");

    _drawStateBuffer.resize(256u);

    _settings._tileWidth = 0.0;

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
}

void
GroundCoverLayer::Renderer::draw(osg::RenderInfo& ri, const PatchLayer::TileBatch* tiles)
{
    DrawState& ds = _drawStateBuffer[ri.getContextID()];
    ds._renderer = this;
    osg::State* state = ri.getState();

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    // Need to unbind any VAO since we'll be doing straight GL calls
    //ri.getState()->unbindVertexArrayObject();
#endif

    // Push the pre-gen culling shader and run it:
    const GroundCoverSA* sa = GroundCoverSA::extract(ri.getState());
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[sa->_groundcover];
    if (!instancer.valid())
    {
        instancer = new InstanceCloud();
    }

    // Only run the compute shader when the tile batch has changed:
    bool needsCompute = false;
    if (ds._lastTileBatchID != tiles->getBatchID())
    {
        ds._lastTileBatchID = tiles->getBatchID();
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

        instancer->allocateGLObjects(ri, tiles->size());
        instancer->preCull(ri);
        _pass = 0;
        tiles->drawTiles(ri);
        instancer->postCull(ri);

        // restore previous program
        state->apply();

        // rendering pass:
        applyLocalState(ri, ds);
        _pass = 1;
        tiles->drawTiles(ri);

        state->popStateSet();
    }

    else
    {
        applyLocalState(ri, ds);
        _pass = 1;
        tiles->drawTiles(ri);
    }

    // Clean up and finish
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    // Need to unbind our VAO so as not to confuse OSG
    ri.getState()->unbindVertexArrayObject();

    //TODO: review this. I don't see why this should be necessary.
    ri.getState()->setLastAppliedProgramObject(NULL);
#endif
}

void
GroundCoverLayer::Renderer::applyLocalState(osg::RenderInfo& ri, DrawState& ds)
{
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp)
        return;

    osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);

    UniformState& u = ds._uniforms[pcp];

    if (u._LLUL < 0)
    {
        u._LLUL = pcp->getUniformLocation(_LLUName);
        u._URUL = pcp->getUniformLocation(_URUName);
        u._A2CUL = pcp->getUniformLocation(_A2CName);
        u._tileNumUL = pcp->getUniformLocation(_tileNumUName);
    }

    u._tileCounter = 0;
    u._LLAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);
    u._URAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);

    // Check for initialization in this zone:
    const GroundCover* groundcover = GroundCoverSA::extract(ri.getState())->_groundcover;
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[groundcover];

    if (!instancer->getGeometry() || u._numInstances1D == 0)
    {
        if (groundcover->options().spacing().isSet())
        {
            float spacing_m = groundcover->options().spacing().get();
            u._numInstances1D = _settings._tileWidth / spacing_m;
        }
        else if (groundcover->options().density().isSet())
        {
            float density_sqkm = groundcover->options().density().get();
            u._numInstances1D = 0.001 * _settings._tileWidth * sqrt(density_sqkm);
        }
        else
        {
            u._numInstances1D = 128;
        }

        if (instancer->getGeometry() == NULL)
        {
            instancer->setGeometry(_layer->createGeometry(u._numInstances1D));
            instancer->setNumInstances(u._numInstances1D, u._numInstances1D);
        }
    }

    GLint multiSamplesOn = ri.getState()->getLastAppliedMode(GL_MULTISAMPLE) ? 1 : 0;
    ri.getState()->applyMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, multiSamplesOn == 1);
    ri.getState()->applyAttribute(_a2cBlending.get());

    if (u._A2CUL >= 0)
    {
        ext->glUniform1i(u._A2CUL, multiSamplesOn);
    }
}

void
GroundCoverLayer::Renderer::drawTile(osg::RenderInfo& ri, const PatchLayer::DrawContext& tile)
{
    const GroundCoverSA* sa = GroundCoverSA::extract(ri.getState());
    DrawState& ds = _drawStateBuffer[ri.getContextID()];
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[sa->_groundcover];
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp)
        return;

    UniformState& u = ds._uniforms[pcp];

    if (_pass == 0) // COMPUTE shader
    {
        osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);

        // uniform sets the offset into the SSBO holding all the render data
        if (u._tileNumUL >= 0)
        {
            ext->glUniform1ui(u._tileNumUL, u._tileCounter);
        }

        if (u._LLUL >= 0 && u._URUL >= 0)
        {
            // transmit the extents of this tile to the shader, skipping the glUniform
            // call if the values have not changed. The shader will calculate the
            // instance positions by interpolating across the tile extents.
            const osg::BoundingBox& bbox = tile._geom->getBoundingBox();

            const osg::Vec3f& LL = bbox.corner(0);
            const osg::Vec3f& UR = bbox.corner(3);

            //if (LL != u._LLAppliedValue)
            {
                ext->glUniform3fv(u._LLUL, 1, LL.ptr());
                //u._LLAppliedValue = LL;
            }

            //if (UR != u._URAppliedValue)
            {
                ext->glUniform3fv(u._URUL, 1, UR.ptr());
                //u._URAppliedValue = UR;
            }
        }

        instancer->cullTile(ri, u._tileCounter);
    }

    else // DRAW shader
    {
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
    for(unsigned i=0; i<_drawStateBuffer.size(); ++i)
    {
        const DrawState& ds = _drawStateBuffer[i];
        for(DrawState::InstancerPerGroundCover::const_iterator j = ds._instancers.begin();
            j != ds._instancers.end();
            ++j)
        {
            if (j->second.valid())
            {
                j->second->releaseGLObjects(state);
            }
        }

        
    }
}

void
GroundCoverLayer::loadShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders shaders;
    shaders.load(vp, shaders.GroundCover_VS, options);
    shaders.load(vp, shaders.GroundCover_FS, options);
}
