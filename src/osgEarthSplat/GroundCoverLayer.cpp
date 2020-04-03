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

using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(groundcover, GroundCoverLayer);
REGISTER_OSGEARTH_LAYER(splat_groundcover, GroundCoverLayer);

// The TS/GS implementation uses GL_PATCHES to pass the terrain tile geometry
// to the tessellation shader and then the geometry shader where it generates
// billboards. Undef this to use a VS-only implementation (which is slower
// and still needs some work with the colors, etc.) but could be useful if
// GS are extremely slow or unavailable on your target platform.
//#define USE_GEOMETRY_SHADER

// If we're not using the GS, we have the option of using instancing or not.
// OFF benchmarks faster on older cards. On newer cards (RTX 2070 e.g.)
// it's slightly faster than using a giant DrawElementsUInt.
#define USE_INSTANCING_IN_VERTEX_SHADER

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
    LayerReference<LandCoverLayer>::set(conf, "land_cover_layer", landCoverLayerName(), landCoverLayer());
    LayerReference<ImageLayer>::set(conf, "mask_layer", maskLayerName(), maskLayer());
    LayerReference<ImageLayer>::set(conf, "color_layer", colorLayerName(), colorLayer());
    conf.set("lod", _lod);
    conf.set("cast_shadows", _castShadows);
    conf.set("grass", grass());

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
    _lod.init(13u);
    _castShadows.init(false);

    LayerReference<LandCoverLayer>::get(conf, "land_cover_layer", landCoverLayerName(), landCoverLayer());
    LayerReference<ImageLayer>::get(conf, "mask_layer", maskLayerName(), maskLayer());
    LayerReference<ImageLayer>::get(conf, "color_layer", colorLayerName(), colorLayer());
    conf.get("lod", _lod);
    conf.get("cast_shadows", _castShadows);
    conf.get("grass", grass());

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
        GroundCoverSA() { }
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

    _useCoverageToAlpha = true;

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

    //if (_renderer.valid())
    //    _renderer->_settings._grass = options().grass().get();

    return Status::OK();
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
    _maskLayer.setLayer(layer);
    if (layer)
    {
        buildStateSets();
    }
}

ImageLayer*
GroundCoverLayer::getMaskLayer() const
{
    return _maskLayer.getLayer();
}

void
GroundCoverLayer::setColorLayer(ImageLayer* value)
{
    _colorLayer.setLayer(value);
    if (value)
    {
        buildStateSets();
    }
}

ImageLayer*
GroundCoverLayer::getColorLayer() const
{
    return _colorLayer.getLayer();
}

void
GroundCoverLayer::addedToMap(const Map* map)
{
    PatchLayer::addedToMap(map);

    _landCoverDict.setLayer(map->getLayer<LandCoverDictionary>());
    _landCoverLayer.findInMap(map, options().landCoverLayerName());
    _maskLayer.findInMap(map, options().maskLayerName());
    _colorLayer.findInMap(map, options().colorLayerName());

    if (getMaskLayer())
    {
        OE_INFO << LC << "Mask layer is \"" << getMaskLayer()->getName() << "\"" << std::endl;
    }

    if (getColorLayer())
    {
        OE_INFO << LC << "Color modulation layer is \"" << getColorLayer()->getName() << "\"" << std::endl;
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

    _landCoverLayer.releaseFromMap(map);
    _maskLayer.releaseFromMap(map);
    _colorLayer.releaseFromMap(map);
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
    }

    // disable backface culling to support shadow/depth cameras,
    // for which the geometry shader renders cross hatches instead of billboards.
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::PROTECTED);

    if (_useCoverageToAlpha)
    {
        // enable alpha-to-coverage multisampling for vegetation.
        stateset->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);

        stateset->setAttributeAndModes(
            new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO),
            osg::StateAttribute::OVERRIDE);
    }

    // uniform that communicates the availability of multisampling.
    if (osg::DisplaySettings::instance()->getMultiSamples())
    {
        stateset->setDefine("OE_GROUNDCOVER_HAS_MULTISAMPLES");
    }

    float maxRange = 0.0f;

    for (Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        Zone* zone = z->get();
        GroundCover* groundCover = zone->getGroundCover();
        if (groundCover)
        {
            if (!groundCover->getBiomes().empty() || groundCover->getTotalNumObjects() > 0)
            {
                osg::StateSet* zoneStateSet = groundCover->getOrCreateStateSet();
                            
                // Install the land cover shaders on the state set
                VirtualProgram* vp = VirtualProgram::getOrCreate(zoneStateSet);
                vp->setName("Ground cover (" + groundCover->getName() + ")");

                // Generate the coverage acceptor shader
#ifdef USE_GEOMETRY_SHADER
                shaders.load(vp, shaders.GroundCover_TCS, getReadOptions());
                shaders.load(vp, shaders.GroundCover_TES, getReadOptions());
                shaders.load(vp, shaders.GroundCover_GS, getReadOptions());

                osg::Shader* covTest = groundCover->createPredicateShader(_landCoverDict.get(), _landCoverLayer.get());
                covTest->setName(covTest->getName() + "_GEOMETRY");
                covTest->setType(osg::Shader::GEOMETRY);
                vp->setShader(covTest);

                // If you define OE_GROUNDCOVER_COVERAGE_PRECHECK in the TCS, you'll need this:
                //osg::Shader* covTest2 = groundCover->createPredicateShader(_landCoverDict.get(), _landCoverLayer.get());
                //covTest->setName(covTest->getName() + "_TESSCONTROL");
                //covTest2->setType(osg::Shader::TESSCONTROL);
                //vp->setShader(covTest2);

                osg::ref_ptr<osg::Shader> layerShader = groundCover->createShader();
                if (!layerShader.valid())
                {
                    setStatus(Status::ConfigurationError, "No GroundCover objects available");
                    setDrawCallback(NULL);
                    return;
                }

                layerShader->setType(osg::Shader::GEOMETRY);
                vp->setShader(layerShader.get());
#else

                loadShaders(vp, getReadOptions());

                osg::Shader* covTest = groundCover->createPredicateShader(getLandCoverDictionary(), getLandCoverLayer());
                covTest->setName(covTest->getName() + "_VERTEX");
                covTest->setType(osg::Shader::VERTEX);
                vp->setShader(covTest);

                osg::ref_ptr<osg::Shader> layerShader = groundCover->createShader();
                if (!layerShader.valid())
                {
                    setStatus(Status::ConfigurationError, "No GroundCover objects available");
                    setDrawCallback(NULL);
                    return;
                }
                layerShader->setType(osg::Shader::VERTEX);
                vp->setShader(layerShader.get());

                //zoneStateSet->setDefine("OE_GROUNDCOVER_NUM_INSTANCES", "132");

#ifdef USE_INSTANCING_IN_VERTEX_SHADER
                zoneStateSet->setDefine("OE_GROUNDCOVER_USE_INSTANCING");
#endif

#endif

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

                if (groundCover->getMaxDistance() > maxRange)
                {
                    maxRange = groundCover->getMaxDistance();
                }

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

    if (maxRange > 0.0f)
    {
        setMaxVisibleRange(maxRange);
        OE_INFO << LC << "Max visible range set to " << maxRange << std::endl;
    }
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
        _renderer->releaseGLObjects(state);

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
        //osg::Geometry* x = dynamic_cast<osg::Geometry*>(osgDB::readNodeFile("../data/tree2.osg"));
        //osgUtil::Optimizer o;
        //o.optimize(x, o.VERTEX_PRETRANSFORM | o.VERTEX_POSTTRANSFORM);
        //return x;

        osg::Geometry* geom = new osg::Geometry();
        geom->setUseVertexBufferObjects(true);

        osg::Vec3Array* v = new osg::Vec3Array();
        v->reserve(3);
        v->push_back(osg::Vec3(-4, 0, 8));
        v->push_back(osg::Vec3(+4, 0, 8)); // bottom
        v->push_back(osg::Vec3( 0, 0, 0)); // left
        geom->setVertexArray(v);

        osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_OVERALL);
        c->push_back(osg::Vec4(1, .5, .2, 1));
        geom->setColorArray(c);

        osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
        normals->push_back(osg::Vec3f(0, 0, 1));
        geom->setNormalArray(normals);

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
        node = makeBBox(dc._geom->getBoundingBox(), *dc._key);
    return node;
}

osg::Geometry*
GroundCoverLayer::createGeometry(unsigned vboTileDim) const    
{
    unsigned numInstances = vboTileDim * vboTileDim;
    const unsigned vertsPerInstance = 8;
    const unsigned indiciesPerInstance = 12;

#ifdef TEST_MODEL_INSTANCING
    geom = makeShape();
    geom->getPrimitiveSet(0)->setNumInstances(numInstances);
    return;
#endif

    osg::Geometry* out_geom = new osg::Geometry();
    out_geom->setUseVertexBufferObjects(true);

#ifdef USE_INSTANCING_IN_VERTEX_SHADER

    static const GLubyte indices[12] = { 0,1,2,2,1,3, 4,5,6,6,5,7 };
    out_geom->addPrimitiveSet(new osg::DrawElementsUByte(GL_TRIANGLES, 12, &indices[0], numInstances));

    // We don't actually need any verts. Is it OK not to set an array?
    //geom->setVertexArray(new osg::Vec3Array(8));

    osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
    normals->push_back(osg::Vec3f(0,0,1));
    out_geom->setNormalArray(normals);

#else // big giant drawelements:

    // Do we need this at all?
    unsigned int numverts = numInstances * vertsPerInstance;
    osg::Vec3Array* coords = new osg::Vec3Array(numverts);
    out_geom->setVertexArray(coords);

    osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
    normals->push_back(osg::Vec3f(0, 0, 1));
    out_geom->setNormalArray(normals);

    osg::DrawElements* de =
        numverts > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt(GL_TRIANGLES)
        : (osg::DrawElements*)new osg::DrawElementsUShort(GL_TRIANGLES);

    const unsigned totalIndicies = numInstances * indiciesPerInstance;
    de->reserveElements(totalIndicies);

    for (unsigned i = 0; i < numInstances; ++i)
    {
        unsigned offset = i * vertsPerInstance;

        de->addElement(0 + offset);
        de->addElement(1 + offset);
        de->addElement(2 + offset);
        de->addElement(2 + offset);
        de->addElement(1 + offset);
        de->addElement(3 + offset);

        de->addElement(4 + offset);
        de->addElement(5 + offset);
        de->addElement(6 + offset);
        de->addElement(6 + offset);
        de->addElement(5 + offset);
        de->addElement(7 + offset);

        if (false) //settings->_grass) // third crosshatch
        {
            de->addElement(8 + offset);
            de->addElement(9 + offset);
            de->addElement(10 + offset);
            de->addElement(10 + offset);
            de->addElement(9 + offset);
            de->addElement(11 + offset);
        }
    }

    out_geom->addPrimitiveSet(de);

#endif // USE_INSTANCING_IN_VERTEX_SHADER

    return out_geom;
}

//........................................................................

// only used with non-GS implementation
GroundCoverLayer::Renderer::DrawState::DrawState()
{
    // initialize all the uniform locations - we will fetch these at draw time
    // when the program is active
    _pcp = NULL;
    _numInstancesUL = -1;
    _LLUL = -1;
    _URUL = -1;
    _LLNormalUL = -1;
    _URNormalUL = -1;
    _instancedModelUL = -1;
    _tilesDrawnThisFrame = 0;
    _numInstances1D = 0;
}

void
GroundCoverLayer::Renderer::DrawState::reset(const osg::State* state, Settings* settings)
{
    _tilesDrawnThisFrame = 0;
    _LLAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);
    _URAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);

    // Check for initialization in this zone:
    const GroundCover* groundcover = GroundCoverSA::extract(state)->_groundcover;
    osg::ref_ptr<osg::Geometry>& geom = _geom[groundcover];

    if (!geom.valid())
    {
        if (groundcover->options().spacing().isSet())
        {
            float spacing_m = groundcover->options().spacing().get();
            _numInstances1D = settings->_tileWidth / spacing_m;
        }
        else if (groundcover->options().density().isSet())
        {
            float density_sqkm = groundcover->options().density().get();
            _numInstances1D = 0.001 * settings->_tileWidth * sqrt(density_sqkm);
        }
        else
        {
            _numInstances1D = 128;
        }

        geom = _renderer->_layer->createGeometry(_numInstances1D);
    }
}

GroundCoverLayer::Renderer::Renderer(GroundCoverLayer* layer)
{
    _layer = layer;

    // create uniform IDs for each of our uniforms
    _numInstancesUName = osg::Uniform::getNameID("oe_GroundCover_numInstances");
    _LLUName = osg::Uniform::getNameID("oe_GroundCover_LL");
    _URUName = osg::Uniform::getNameID("oe_GroundCover_UR");
    _instancedModelUName = osg::Uniform::getNameID("oe_GroundCover_instancedModel");

    _drawStateBuffer.resize(64u);

    _settings._tileWidth = 0.0;
    //_settings._grass = false;
}

void
GroundCoverLayer::Renderer::preDraw(osg::RenderInfo& ri, osg::ref_ptr<osg::Referenced>& data)
{
    DrawState& ds = _drawStateBuffer[ri.getContextID()];
    ds._renderer = this;

    ds.reset(ri.getState(), &_settings);

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    // Need to unbind any VAO since we'll be doing straight GL calls
    //ri.getState()->unbindVertexArrayObject();
#endif

    // Testing - need this for glMultiDrawElementsIndirect
    //osg::PrimitiveSet* de = ds._geom->getPrimitiveSet(0);
    //osg::GLBufferObject* ebo = de->getOrCreateGLBufferObject(ri.getContextID());
    //ri.getState()->bindElementBufferObject(ebo);
}

namespace
{
    struct DrawElementsIndirectCommand {
        GLuint  count;
        GLuint  instanceCount;
        GLuint  firstIndex;
        GLuint  baseVertex;
        GLuint  baseInstance;
    };
}

void
GroundCoverLayer::Renderer::draw(osg::RenderInfo& ri, const DrawContext& tile, osg::Referenced* data)
{
    DrawState& ds = _drawStateBuffer[ri.getContextID()];
    osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);

    // find the uniform location for our uniforms if necessary.
    // (only necessary when the PCP has changed)
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (pcp != ds._pcp || ds._numInstancesUL < 0)
    {
        if (pcp == NULL)
        {
            //OE_WARN << "[GroundCoverLayer] ILLEGAL STATE - getLastAppliedProgramObject == NULL. Contact support." << std::endl;
            return;
        }
        ds._numInstancesUL = pcp->getUniformLocation(_numInstancesUName);
        ds._LLUL = pcp->getUniformLocation(_LLUName);
        ds._URUL = pcp->getUniformLocation(_URUName);
        ds._instancedModelUL = pcp->getUniformLocation(_instancedModelUName);

        ds._pcp = pcp;
    }

    // on the first draw call this frame, initialize the uniform that tells the
    // shader the total number of instances:
    if (ds._tilesDrawnThisFrame == 0)
    {
        osg::Vec2f numInstances(ds._numInstances1D, ds._numInstances1D);
        ext->glUniform2fv(ds._numInstancesUL, 1, numInstances.ptr());

#ifdef TEST_MODEL_INSTANCING
        const int useInstancedModel = 1;
#else
        const int useInstancedModel = 0;
#endif
        ext->glUniform1i(ds._instancedModelUL, useInstancedModel);
    }

    // transmit the extents of this tile to the shader, skipping the glUniform
    // call if the values have not changed. The shader will calculate the
    // instance positions by interpolating across the tile extents.
    const osg::BoundingBox& bbox = tile._geom->getBoundingBox();

    const osg::Vec3f& LL = bbox.corner(0);
    const osg::Vec3f& UR = bbox.corner(3);

    if (LL != ds._LLAppliedValue)
    {
        ext->glUniform3fv(ds._LLUL, 1, LL.ptr());
        ds._LLAppliedValue = LL;
    }

    if (UR != ds._URAppliedValue)
    {
        ext->glUniform3fv(ds._URUL, 1, UR.ptr());
        ds._URAppliedValue = UR;
    }

    const GroundCoverSA* sa = GroundCoverSA::extract(ri.getState());
    osg::ref_ptr<osg::Geometry>& geom = ds._geom[sa->_groundcover];

    // draw the instanced billboard geometry:
    geom->draw(ri);

    ++ds._tilesDrawnThisFrame;
}

void
GroundCoverLayer::Renderer::postDraw(osg::RenderInfo& ri, osg::Referenced* data)
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    // Need to unbind our VAO so as not to confuse OSG
    ri.getState()->unbindVertexArrayObject();

    //TODO: review this. I don't see why this should be necessary.
    ri.getState()->setLastAppliedProgramObject(NULL);
#endif
}

void
GroundCoverLayer::Renderer::resizeGLObjectBuffers(unsigned maxSize)
{
    _drawStateBuffer.resize(osg::maximum(maxSize, _drawStateBuffer.size()));

    for (unsigned i = 0; i < _drawStateBuffer.size(); ++i)
    {
        const DrawState& ds = _drawStateBuffer[i];
        for (DrawState::GeomPerGroundCover::const_iterator j = ds._geom.begin();
            j != ds._geom.end();
            ++j)
        {
            if (j->second.valid())
            {
                j->second->resizeGLObjectBuffers(maxSize);
            }
        }
    }
}

void
GroundCoverLayer::Renderer::releaseGLObjects(osg::State* state) const
{
    for(unsigned i=0; i<_drawStateBuffer.size(); ++i)
    {
        const DrawState& ds = _drawStateBuffer[i];
        for(DrawState::GeomPerGroundCover::const_iterator j = ds._geom.begin();
            j != ds._geom.end();
            ++j)
        {
            if (j->second.valid())
            {
                j->second->releaseGLObjects(state);
            }
        }
    }
}


Config
GroundCoverLayer::getConfig() const
{
    Config c = PatchLayer::getConfig();
    if (_landCoverDict.isSetByUser())
        c.set(_landCoverDict.getLayer()->getConfig());
    if (_landCoverLayer.isSetByUser())
        c.set(_landCoverLayer.getLayer()->getConfig());
    if (_maskLayer.isSetByUser())
        c.set(_maskLayer.getLayer()->getConfig());
    if (_colorLayer.isSetByUser())
        c.set(_colorLayer.getLayer()->getConfig());
    return c;
}

void
GroundCoverLayer::loadShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders shaders;
    shaders.load(vp, shaders.GroundCover_VS, options);
    shaders.load(vp, shaders.GroundCover_FS, options);
}