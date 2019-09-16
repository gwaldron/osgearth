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
#include <osgUtil/CullVisitor>
#include <osg/BlendFunc>
#include <osg/Multisample>
#include <osg/Texture2D>
#include <osg/Version>
#include <cstdlib> // getenv

#define LC "[GroundCoverLayer] " << getName() << ": "

#define GCTEX_SAMPLER "oe_GroundCover_billboardTex"
#define NOISE_SAMPLER "oe_GroundCover_noiseTex"

using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(groundcover, GroundCoverLayer);
REGISTER_OSGEARTH_LAYER(splat_groundcover, GroundCoverLayer);

//........................................................................

Config
GroundCoverLayer::Options::getConfig() const
{
    Config conf = PatchLayer::Options::getConfig();
    conf.set("land_cover_layer", _landCoverLayer);
    conf.set("mask_layer", _maskLayer);
    conf.set("lod", _lod);
    conf.set("cast_shadows", _castShadows);

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

    conf.get("land_cover_layer", _landCoverLayer);
    conf.get("mask_layer", _maskLayer);
    conf.get("lod", _lod);
    conf.get("cast_shadows", _castShadows);

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
    
    // this layer will do its own custom rendering
    _renderer = new Renderer();
    setDrawCallback(_renderer.get());
}

const Status&
GroundCoverLayer::open()
{
    return PatchLayer::open();
}

void
GroundCoverLayer::setLandCoverDictionary(LandCoverDictionary* layer)
{
    _landCoverDict = layer;
    if (layer)
        buildStateSets();
}

const LandCoverDictionary*
GroundCoverLayer::getLandCoverDictionary() const
{
    return _landCoverDict.get();
}

void
GroundCoverLayer::setLandCoverLayer(LandCoverLayer* layer)
{
    _landCoverLayer = layer;
    if (layer) {
        OE_INFO << LC << "Land cover layer is \"" << layer->getName() << "\"\n";
        buildStateSets();
    }
}

const LandCoverLayer*
GroundCoverLayer::getLandCoverLayer() const
{
    return _landCoverLayer.get();
}

void
GroundCoverLayer::setMaskLayer(ImageLayer* layer)
{
    _maskLayer = layer;
    if (layer)
    {
        OE_INFO << LC << "Mask layer is \"" << layer->getName() << "\"\n";
        buildStateSets();
    }
}

const ImageLayer*
GroundCoverLayer::getMaskLayer() const
{
    return _maskLayer.get();
}

void
GroundCoverLayer::addedToMap(const Map* map)
{
    PatchLayer::addedToMap(map);

    if (!_landCoverDict.valid())
    {
        _landCoverDictListener.listen(map, this, &GroundCoverLayer::setLandCoverDictionary);
    }

    if (!_landCoverLayer.valid() && options().landCoverLayer().isSet())
    {
        _landCoverListener.listen(map, options().landCoverLayer().get(), this, &GroundCoverLayer::setLandCoverLayer);
    }

    if (options().maskLayer().isSet())
    {
        _maskLayerListener.listen(map, options().maskLayer().get(), this, &GroundCoverLayer::setMaskLayer);
    }

    for (Zones::iterator zone = _zones.begin(); zone != _zones.end(); ++zone)
    {
        zone->get()->configure(map, getReadOptions());
    }

    _zonesConfigured = true;
    
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

    NoiseTextureFactory noise;
    osg::ref_ptr<osg::Texture> noiseTexture = noise.create(256u, 4u);

    GroundCoverShaders shaders;

    // Layer-wide stateset:
    osg::StateSet* stateset = getOrCreateStateSet();

    // bind the noise sampler.
    stateset->setTextureAttribute(_noiseBinding.unit(), noiseTexture.get());
    stateset->addUniform(new osg::Uniform(NOISE_SAMPLER, _noiseBinding.unit()));

    if (_maskLayer.valid())
    {
        stateset->setDefine("OE_GROUNDCOVER_MASK_SAMPLER", _maskLayer->shareTexUniformName().get());
        stateset->setDefine("OE_GROUNDCOVER_MASK_MATRIX", _maskLayer->shareTexMatUniformName().get());
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
                shaders.load(vp, shaders.GroundCover_VS, getReadOptions());
                shaders.load(vp, shaders.GroundCover_FS, getReadOptions());

                // Generate the coverage acceptor shader
                osg::Shader* covTest = groundCover->createPredicateShader(_landCoverDict.get(), _landCoverLayer.get());
                covTest->setName(covTest->getName() + "_VERTEX");
                covTest->setType(osg::Shader::VERTEX);
                vp->setShader(covTest);

                osg::ref_ptr<osg::Shader> layerShader = groundCover->createShader();
                layerShader->setType(osg::Shader::VERTEX);
                vp->setShader(layerShader.get());

                vp->addBindAttribLocation("oe_GroundCover_position", 6);

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

    _renderer->releaseGLObjects(state);

    PatchLayer::releaseGLObjects(state);

    // For some unknown reason, release doesn't work on the zone 
    // texture def data (SplatTextureDef). So we have to recreate
    // it here.
    const_cast<GroundCoverLayer*>(this)->buildStateSets();
}


//........................................................................

GroundCoverLayer::Renderer::DrawState::DrawState()
{
    const unsigned tileSize = 132u; //96u;
    unsigned numInstances = tileSize*tileSize;

    _geom = new osg::Geometry();
    _geom->setDataVariance(osg::Object::STATIC);
    _geom->setUseVertexBufferObjects(true);

    //osg::VertexBufferObject* vbo = new osg::VertexBufferObject();
    //vbo->setUsage(GL_STATIC_DRAW);

    // 8 verts to draw 2 quads
    //osg::Vec3Array* verts = new osg::Vec3Array(8);
    //verts->setVertexBufferObject(vbo);
    //_geom->setVertexArray(verts);

    static const GLubyte indices[12] = { 0,1,2,1,2,3, 4,5,6,5,6,7 };
    _geom->addPrimitiveSet(new osg::DrawElementsUByte(GL_TRIANGLES, 12, &indices[0], numInstances));

    // initialize all the uniform locations - we will fetch these at draw time
    // when the program is active
    _pcp = NULL;
    _numInstancesUL = -1;
    _LLUL = -1;
    _URUL = -1;
    _LLNormalUL = -1;
    _URNormalUL = -1;

    _numInstances1D = tileSize;
}

GroundCoverLayer::Renderer::Renderer()
{
    // create uniform IDs for each of our uniforms
    _numInstancesUName = osg::Uniform::getNameID("oe_GroundCover_numInstances");
    _LLUName = osg::Uniform::getNameID("oe_GroundCover_LL");
    _URUName = osg::Uniform::getNameID("oe_GroundCover_UR");

    _drawStateBuffer.resize(64u);
}

void
GroundCoverLayer::Renderer::preDraw(osg::RenderInfo& ri, osg::ref_ptr<osg::Referenced>& data)
{
    DrawState& ds = _drawStateBuffer[ri.getContextID()];

    ds._tilesDrawnThisFrame = 0;

    ds._LLAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);
    ds._URAppliedValue.set(FLT_MAX, FLT_MAX, FLT_MAX);

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    // Need to unbind any VAO since we'll be doing straight GL calls
    ri.getState()->unbindVertexArrayObject();
#endif
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
    osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(tile._geom->getVertexArray());

    osg::Vec3f LL = verts->front();
    osg::Vec3f UR = verts->back();

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
    // nop
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
        _drawStateBuffer[i]._geom->releaseGLObjects(state);
    }
}
