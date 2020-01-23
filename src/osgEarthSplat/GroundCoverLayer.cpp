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
// OFF by default since it benchmarks faster on older cards. On newer cards
// (RTX 2070 e.g.) it's about the same.
//#define USE_INSTANCING_IN_VERTEX_SHADER

//........................................................................

Config
GroundCoverLayer::Options::getConfig() const
{
    Config conf = PatchLayer::Options::getConfig();
    LayerReference<LandCoverLayer>::set(conf, "land_cover_layer", landCoverLayerName(), landCoverLayer());
    LayerReference<ImageLayer>::set(conf, "mask_layer", maskLayerName(), maskLayer());
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
    conf.get("lod", _lod);
    conf.get("cast_shadows", _castShadows);
    conf.get("grass", grass());

    const Config* zones = conf.child_ptr("zones");
    if (zones) {
        const ConfigSet& children = zones->children();
        for (ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i) {
            _zones.push_back(ZoneOptions(*i));
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
    _renderer = new Renderer();
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

    if (_renderer.valid())
        _renderer->_settings._grass = options().grass().get();

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
        OE_INFO << LC << "Mask layer is \"" << layer->getName() << "\"\n";
        buildStateSets();
    }
}

ImageLayer*
GroundCoverLayer::getMaskLayer() const
{
    return _maskLayer.getLayer();
}

void
GroundCoverLayer::addedToMap(const Map* map)
{
    PatchLayer::addedToMap(map);

    _landCoverDict.setLayer(map->getLayer<LandCoverDictionary>());
    _landCoverLayer.connect(map, options().landCoverLayerName());
    _maskLayer.connect(map, options().maskLayerName());

    for (Zones::iterator zone = _zones.begin(); zone != _zones.end(); ++zone)
    {
        zone->get()->configure(map, getReadOptions());
    }

    _zonesConfigured = true;

    // calculate the instance count based on the density and LOD.
    if (_renderer.valid() && _zones.size() > 0 && _zones[0]->getGroundCover())
    {
        unsigned lod = getLOD();
        unsigned tx, ty;
        map->getProfile()->getNumTiles(lod, tx, ty);
        GeoExtent e = TileKey(lod, tx/2, ty/2, map->getProfile()).getExtent();
        GeoCircle c = e.computeBoundingGeoCircle();
        double width_m = 2.0 * c.getRadius() / 1.4142;

        if (_zones[0]->getGroundCover()->options().spacing().isSet())
        {
            float spacing_m = _zones[0]->getGroundCover()->options().spacing().get();
            _renderer->_settings._vboTileSize = width_m / spacing_m;
        }
        else if (_zones[0]->getGroundCover()->options().density().isSet())
        {
            float density_sqkm = _zones[0]->getGroundCover()->options().density().get();
            _renderer->_settings._vboTileSize = 0.001 * width_m * sqrt(density_sqkm);
        }
        else
        {
            _renderer->_settings._vboTileSize = 128;
        }

        OE_INFO << LC << "Instances across = " << _renderer->_settings._vboTileSize << std::endl;
    }

    buildStateSets();
}

void
GroundCoverLayer::removedFromMap(const Map* map)
{
    PatchLayer::removedFromMap(map);
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

    // disable backface culling to support shadow/depth cameras,
    // for which the geometry shader renders cross hatches instead of billboards.
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::PROTECTED);

    // enable alpha-to-coverage multisampling for vegetation.
    stateset->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);

    // uniform that communicates the availability of multisampling.
    if (osg::DisplaySettings::instance()->getMultiSamples())
    {
        stateset->setDefine("OE_GROUNDCOVER_HAS_MULTISAMPLES");
    }

    stateset->setAttributeAndModes(
        new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO),
        osg::StateAttribute::OVERRIDE);

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
                shaders.load(vp, shaders.GroundCover_FS, getReadOptions());

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
                layerShader->setType(osg::Shader::GEOMETRY);
                vp->setShader(layerShader.get());
#else
                if (options().grass() == true)
                    shaders.load(vp, shaders.Grass_VS, getReadOptions());
                else
                    shaders.load(vp, shaders.GroundCover_VS, getReadOptions());

                osg::Shader* covTest = groundCover->createPredicateShader(getLandCoverDictionary(), getLandCoverLayer());
                covTest->setName(covTest->getName() + "_VERTEX");
                covTest->setType(osg::Shader::VERTEX);

                osg::ref_ptr<osg::Shader> layerShader = groundCover->createShader();
                layerShader->setType(osg::Shader::VERTEX);
                vp->setShader(layerShader.get());

                //zoneStateSet->setDefine("OE_GROUNDCOVER_NUM_INSTANCES", "132");

#ifdef USE_INSTANCING_IN_VERTEX_SHADER
                zoneStateSet->setDefine("OE_GROUNDCOVER_USE_INSTANCING");
#endif

#endif

                vp->setShader(covTest);

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
}

osg::Node*
GroundCoverLayer::createNodeImplementation(const DrawContext& dc)
{
    osg::Node* node = NULL;
    if (_debug)
        node = makeBBox(dc._geom->getBoundingBox(), *dc._key);
    return node;
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
}

void
GroundCoverLayer::Renderer::DrawState::reset(Settings* settings)
{
    _tilesDrawnThisFrame = 0;
    _LLAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);
    _URAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);

    if (!_geom.valid())
    {
        const unsigned numInstances = settings->_vboTileSize * settings->_vboTileSize;
        const unsigned vertsPerInstance = 8;
        const unsigned indiciesPerInstance = 12;

        _geom = new osg::Geometry();
        _geom->setDataVariance(osg::Object::STATIC);
        _geom->setUseVertexBufferObjects(true);

#ifdef USE_INSTANCING_IN_VERTEX_SHADER

        static const GLubyte indices[12] = { 0,1,2,1,2,3, 4,5,6,5,6,7 };
        _geom->addPrimitiveSet(new osg::DrawElementsUByte(GL_TRIANGLES, 12, &indices[0], numInstances));

#else // big giant drawelements:

        const unsigned totalIndicies = numInstances * indiciesPerInstance;
        std::vector<GLuint> indices;
        indices.reserve(totalIndicies);

        for(unsigned i=0; i<numInstances; ++i)
        {
            unsigned offset = i * vertsPerInstance;

            indices.push_back(0 + offset);
            indices.push_back(1 + offset);
            indices.push_back(2 + offset);
            indices.push_back(2 + offset);
            indices.push_back(1 + offset);
            indices.push_back(3 + offset);

            indices.push_back(4 + offset);
            indices.push_back(5 + offset);
            indices.push_back(6 + offset);
            indices.push_back(6 + offset);
            indices.push_back(5 + offset);
            indices.push_back(7 + offset);

            if (false) //settings->_grass)
            {
                indices.push_back(8 + offset);
                indices.push_back(9 + offset);
                indices.push_back(10 + offset);
                indices.push_back(10 + offset);
                indices.push_back(9 + offset);
                indices.push_back(11 + offset);
            }
        }

        _geom->addPrimitiveSet(new osg::DrawElementsUInt(GL_TRIANGLES, totalIndicies, indices.data(), 0));

        // Do we need this at all?
        unsigned int numverts = numInstances * vertsPerInstance;
        osg::Vec3Array* coords = new osg::Vec3Array(numverts);
        _geom->setVertexArray(coords);

#endif // USE_INSTANCING_IN_VERTEX_SHADER

        _numInstances1D = settings->_vboTileSize;
    }
}

GroundCoverLayer::Renderer::Renderer()
{
    // create uniform IDs for each of our uniforms
    _numInstancesUName = osg::Uniform::getNameID("oe_GroundCover_numInstances");
    _LLUName = osg::Uniform::getNameID("oe_GroundCover_LL");
    _URUName = osg::Uniform::getNameID("oe_GroundCover_UR");

    _drawStateBuffer.resize(64u);

    _settings._vboTileSize = 128;
    _settings._grass = false;
}

void
GroundCoverLayer::Renderer::preDraw(osg::RenderInfo& ri, osg::ref_ptr<osg::Referenced>& data)
{
    DrawState& ds = _drawStateBuffer[ri.getContextID()];

    ds.reset(&_settings);

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
        ds._numInstancesUL = pcp->getUniformLocation(_numInstancesUName);
        ds._LLUL = pcp->getUniformLocation(_LLUName);
        ds._URUL = pcp->getUniformLocation(_URUName);

        ds._pcp = pcp;
    }

    // on the first draw call this frame, initialize the uniform that tells the
    // shader the total number of instances:
    if (ds._tilesDrawnThisFrame == 0)
    {
        osg::Vec2f numInstances(ds._numInstances1D, ds._numInstances1D);
        ext->glUniform2fv(ds._numInstancesUL, 1, numInstances.ptr());
    }

    // transmit the extents of this tile to the shader, skipping the glUniform
    // call if the values have not changed. The shader will calculate the
    // instance positions by interpolating across the tile extents.
    const osg::BoundingBox& bbox = tile._geom->getBoundingBox();

    const osg::Vec3f& LL = bbox.corner(0);
    const osg::Vec3f& UR = bbox.corner(7);

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

    // draw the instanced billboard geometry:
    ds._geom->draw(ri);

    ++ds._tilesDrawnThisFrame;
}

void
GroundCoverLayer::Renderer::postDraw(osg::RenderInfo& ri, osg::Referenced* data)
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    // Need to unbind our VAO so as not to confuse OSG
    ri.getState()->unbindVertexArrayObject();
    ri.getState()->setLastAppliedProgramObject(NULL);
#endif
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
        if (_drawStateBuffer[i]._geom.valid())
            _drawStateBuffer[i]._geom->releaseGLObjects(state);
    }
}

