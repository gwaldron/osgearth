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

#define ATLAS_SAMPLER "oe_gc_atlas"
#define NOISE_SAMPLER "oe_gc_noiseTex"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(groundcover, GroundCoverLayer);
REGISTER_OSGEARTH_LAYER(splat_groundcover, GroundCoverLayer);


// TODO LIST
//
//  - Reduce the size of the RenderBuffer structure
//  - include a "model range" in the InstanceBuffer...?
//  - fade between model/imposter?
//  - thin out distant instances automatically in large tiles
//  - cull by "horizon" .. e.g., the lower you are, the fewer distant trees...?
//  - texture management as the catalog gets bigger (paged arrays or different sizes?)
//  - variable spacing or clumping by landcovergroup or asset...?
//  - fix the random asset select with weighting...just not really working well.
//  - deal with blending to fade in 3d models
//  - programmable SSE for models?
//  - (DONE - I think - GLUtils::deleteGLBuffer) Properly delete all GL memory (reimplement as osg::BufferObject's)
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
    _renderer->_atlas = new TextureAtlas(); //createTextureAtlas();

    // Load asset data from the configuration. This will populate
    // _liveAssets as well as the _imagesToAddToAtlas.
    loadAssets(_renderer->_atlas.get());

    // Prepare model assets and add their textures to the atlas:
    _renderer->_geomCloud = createGeometryCloud(_renderer->_atlas.get());

    // bind the cloud's stateset to this layer.
    if (_renderer->_geomCloud.valid())
    {
        osg::StateSet* cloudSS = _renderer->_geomCloud->getGeometry()->getStateSet();
        if (cloudSS)
            getOrCreateStateSet()->merge(*cloudSS);
    }

    // Install LUTs on the compute shader:
    osg::ref_ptr<osg::Shader> lutShader = createLUTShader();
    lutShader->setName("GroundCover CS LUT");
    _renderer->_generateProgram->addShader(lutShader.get());
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
        if (_atlasBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_atlasBinding, this, "GroundCover texture atlas") == false)
            {
                OE_WARN << LC << "No texture unit available for ground cover texture atlas\n";
            }
        }

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
    if (_atlasBinding.valid() == false) {
        OE_DEBUG << LC << "buildStateSets deferred.. bindings not reserved" << std::endl;
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

    if (osg::DisplaySettings::instance()->getNumMultiSamples() > 1)
        stateset->setMode(GL_MULTISAMPLE, 1);
    else
        stateset->removeMode(GL_MULTISAMPLE);

    // The billboard texture atlas:
    stateset->setTextureAttribute(_atlasBinding.unit(), _renderer->_atlas.get());
    stateset->addUniform(new osg::Uniform(ATLAS_SAMPLER, _atlasBinding.unit()));

    // Install the land cover shaders on the state set
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->setName("GroundCover");

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
        node = makeBBox(*dc._geomBBox, *dc._key);
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

#define PASS_GENERATE 0
#define PASS_CULL 1
#define PASS_DRAW 2

GroundCoverLayer::Renderer::DrawState::UniformState::UniformState()
{
    // initialize all the uniform locations - we will fetch these at draw time
    // when the program is active
    _generateDataUL = -1;
    _A2CUL = -1;
    _numCommandsUL = -1;
    _tileCounter = 0;
}

GroundCoverLayer::Renderer::Renderer(GroundCoverLayer* layer)
{
    _layer = layer;

    // create uniform IDs for each of our uniforms
    _A2CName = osg::Uniform::getNameID("oe_gc_useAlphaToCoverage");
    _computeDataUName = osg::Uniform::getNameID("oe_tile");
    _numCommandsUName = osg::Uniform::getNameID("oe_gc_numCommands");

    _drawStateBuffer.resize(256u);

    _tileWidth = 0.0;

    _a2cBlending = new osg::BlendFunc(GL_ONE, GL_ZERO, GL_ONE, GL_ZERO);

    // Load our compute shader
    GroundCoverShaders shaders;

    std::string generateSource = ShaderLoader::load(shaders.GroundCover_Generate, shaders, layer->getReadOptions());
    _generateStateSet = new osg::StateSet();

    _generateProgram = new osg::Program();
    osg::Shader* generateShader = new osg::Shader(osg::Shader::COMPUTE, generateSource);
    generateShader->setName(shaders.GroundCover_Generate);
    _generateProgram->addShader(generateShader);
    _generateStateSet->setAttribute(_generateProgram, osg::StateAttribute::ON);

    std::string cullSource = ShaderLoader::load(shaders.GroundCover_Sort_CS, shaders, layer->getReadOptions());
    _cullStateSet = new osg::StateSet();
    _cullProgram = new osg::Program();
    osg::Shader* cullShader = new osg::Shader(osg::Shader::COMPUTE, cullSource);
    cullShader->setName(shaders.GroundCover_Sort_CS);
    _cullProgram->addShader(cullShader);
    _cullStateSet->setAttribute(_cullProgram, osg::StateAttribute::ON);
}

GroundCoverLayer::Renderer::~Renderer()
{
    releaseGLObjects(0);
}

void
GroundCoverLayer::Renderer::visitTileBatch(osg::RenderInfo& ri, const PatchLayer::TileBatch* tiles)
{
    DrawState& ds = _drawStateBuffer[ri.getContextID()];
    ds._renderer = this;
    osg::State* state = ri.getState();

    const ZoneSA* sa = ZoneSA::extract(ri.getState());
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[sa->_obj];
    if (!instancer.valid())
    {
        instancer = new InstanceCloud();
    }

    // Only run the compute shader when the tile batch has changed:
    bool needsGenerate = false;
    if (ds._lastTileBatchID != tiles->getBatchID())
    {
        ds._lastTileBatchID = tiles->getBatchID();
        needsGenerate = true;
    }

    // not a bug. Do this as a 32-bit matrix to avoid wierd micro-precision changes.
    bool needsCull = needsGenerate;
    if (!needsCull)
    {
        osg::Matrixf mvp = state->getModelViewMatrix() * state->getProjectionMatrix();
        if (ds._lastMVP != mvp)
        {
            ds._lastMVP = mvp;
            needsCull = true;
        }
    }

    // I'm not sure why we have to push the layer's stateset here.
    // It should have been applied already in the render bin.
    // I am missing something. -gw 4/20/20
    state->pushStateSet(_layer->getStateSet());

    if (needsGenerate)
    {
        _pass = PASS_GENERATE;
        state->apply(_generateStateSet.get()); // activate gen program
        applyLocalState(ri, ds);               // reset counter and setup instancer
        instancer->allocateGLObjects(ri, tiles->size());  // ensure sufficient memory
    }

    instancer->newFrame(ri);

    if (needsGenerate)
    {
        instancer->generate_begin(ri);
        tiles->drawTiles(ri);
        instancer->generate_end(ri);
    }

    if (needsCull)
    {
        // per frame cull/sort:
        _pass = PASS_CULL;
        state->apply(_cullStateSet.get()); // activate cull program
        applyLocalState(ri, ds); // reset tile counter
        tiles->visitTiles(ri);   // collect tile matrix data
        instancer->cull(ri);     // cull and sort
    }

    // If we ran a compute shader, we replaced the state and 
    // now need to re-apply it before rendering.
    if (needsGenerate || needsCull)
    {
        state->apply();
    }

    // draw pass:
    _pass = PASS_DRAW;
    applyLocalState(ri, ds);
    instancer->draw(ri);

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
GroundCoverLayer::Renderer::applyLocalState(osg::RenderInfo& ri, DrawState& ds)
{
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp)
        return;

    osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);

    DrawState::UniformState& u = ds._uniforms[pcp];
    u._tileCounter = 0;

    // Check for initialization in this zone:
    const BiomeZone* bz = ZoneSA::extract(ri.getState())->_obj;
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[bz];

    // If the instancer is not yet initialized, do so now.
    // This happens here because the instance count varies 
    // depending on the BiomeZone.
    if (instancer->getNumInstancesPerTile() == 0)
    {
        unsigned numInstances1D = 64u;

        if (bz->options().spacing().isSet())
        {
            float spacing_m = bz->options().spacing()->as(Units::METERS);
            numInstances1D = _tileWidth / spacing_m;
            _spacing = spacing_m;
        }

        instancer->setGeometryCloud(_geomCloud.get());
        instancer->setNumInstancesPerTile(numInstances1D, numInstances1D);

        // TODO: FIX THIS - NOT GOOD. Cannot merge ALL the geometrycloud's
        // statesets into the layer..... but, this may change once we get
        // rid of BiomeZones.
        // This is here to integrate the model's texture atlas into the stateset
        //osg::StateSet* cloudStateSet = instancer->getGeometryCloud()->getGeometry()->getStateSet();
        //if (cloudStateSet)
        //    _layer->getOrCreateStateSet()->merge(*cloudStateSet);
    }

    if (_pass == PASS_CULL)
    {
        if (u._numCommandsUL < 0)
            u._numCommandsUL = pcp->getUniformLocation(_numCommandsUName);

        if (u._numCommandsUL >= 0)
            ext->glUniform1i(u._numCommandsUL, _geomCloud->getNumDrawCommands());
    }

    else if (_pass == PASS_DRAW)
    {
        GLint useA2C = 0;
        if (_layer->getUseAlphaToCoverage())
        {
            useA2C = ri.getState()->getLastAppliedMode(GL_MULTISAMPLE) ? 1 : 0;
            ri.getState()->applyMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, useA2C == 1);
            ri.getState()->applyAttribute(_a2cBlending.get());
        }

        if (u._numCommandsUL < 0)
            u._numCommandsUL = pcp->getUniformLocation(_numCommandsUName);

        if (u._numCommandsUL >= 0)
            ext->glUniform1i(u._numCommandsUL, _layer->_liveAssets.size());

        if (u._A2CUL < 0)
            u._A2CUL = pcp->getUniformLocation(_A2CName);

        if (u._A2CUL >= 0)
            ext->glUniform1i(u._A2CUL, useA2C);
    }
}

void
GroundCoverLayer::Renderer::visitTile(osg::RenderInfo& ri, const PatchLayer::DrawContext& tile)
{
    const ZoneSA* sa = ZoneSA::extract(ri.getState());
    DrawState& ds = _drawStateBuffer[ri.getContextID()];
    osg::ref_ptr<InstanceCloud>& instancer = ds._instancers[sa->_obj];
    const osg::Program::PerContextProgram* pcp = ri.getState()->getLastAppliedProgramObject();
    if (!pcp)
        return;

    DrawState::UniformState& u = ds._uniforms[pcp];

    if (_pass == PASS_GENERATE)
    {
        osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);

        if (u._generateDataUL < 0)
            u._generateDataUL = pcp->getUniformLocation(_computeDataUName);

        if (u._generateDataUL >= 0)
        {
            u._generateData[0] = tile._tileBBox->xMin();
            u._generateData[1] = tile._tileBBox->yMin();
            u._generateData[2] = tile._tileBBox->xMax();
            u._generateData[3] = tile._tileBBox->yMax();

            u._generateData[4] = (float)u._tileCounter;

            // TODO: check whether this changed before calling it
            ext->glUniform1fv(u._generateDataUL, 5, &u._generateData[0]);

            instancer->generate_tile(ri);
        }
    }

    else if (_pass == PASS_CULL)
    {
        // TODO: Collect matrices and send to instancer
        instancer->setMatrix(u._tileCounter, *tile._modelViewMatrix);
    }

    else // if (_pass == PASS_DRAW)
    {
        // NOP
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
            InstanceCloud* instancer = j->second.get();
            if (instancer)
                instancer->releaseGLObjects(state);
        }        
    }
}

void
GroundCoverLayer::loadRenderingShaders(VirtualProgram* vp, const osgDB::Options* options) const
{
    GroundCoverShaders shaders;
    shaders.load(vp, shaders.GroundCover_Render);
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

    struct ModelCacheEntry {
        osg::ref_ptr<osg::Node> _node;
        int _modelID;
    };
}

void
GroundCoverLayer::loadAssets(TextureAtlas* atlas)
{
    typedef std::map<URI, osg::ref_ptr<osg::Image> > ImageCache;
    ImageCache imagecache;

    typedef std::map<URI, ModelCacheEntry> ModelCache;
    ModelCache modelcache;

    ImageVector imagesToAddToAtlas;

    int assetIDGen = 0;
    int modelIDGen = 0;

    // all the zones (these will later be called "biomes")
    for(int z=0; z<getZones().size(); ++z)
    {
        // each zone has a single layout (used to be called "groundcover")
        const BiomeZone& zone = getZones()[z];

        // each layout has one or more groupings of land cover classes
        // (this used to be called a biome)
        for(int j=0; j<zone.getLandCoverGroups().size(); ++j)
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
                data->_landCoverGroupIndex = j;
                data->_landCoverGroup = &group;
                data->_asset = &asset;
                data->_numInstances = 0;
                data->_codes = codes;
                data->_sideImageAtlasIndex = -1;
                data->_topImageAtlasIndex = -1;
                data->_modelAtlasIndex = -1;
                data->_modelID = -1;
                data->_assetID = -1;

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
                    ImageCache::iterator ic = imagecache.find(uri);
                    if (ic != imagecache.end())
                    {
                        data->_sideImage = ic->second.get();
                        data->_sideImageAtlasIndex = indexOf(imagesToAddToAtlas, data->_sideImage.get());
                    }
                    else
                    {
                        data->_sideImage = uri.getImage(getReadOptions());
                        if (data->_sideImage.valid())
                        {
                            data->_sideImageAtlasIndex = imagesToAddToAtlas.size();
                            imagesToAddToAtlas.push_back(data->_sideImage.get());
                            imagecache[uri] = data->_sideImage.get();
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load billboard side image from \"" << uri.full() << "\"" << std::endl;
                        }
                    }
                }

                if (asset.options().topBillboardURI().isSet())
                {
                    const URI& uri = asset.options().topBillboardURI().get();
                    ImageCache::iterator ic = imagecache.find(uri);
                    if (ic != imagecache.end())
                    {
                        data->_topImage = ic->second.get();
                        data->_topImageAtlasIndex = indexOf(imagesToAddToAtlas, data->_topImage.get());
                    }
                    else
                    {
                        data->_topImage = uri.getImage(getReadOptions());
                        if (data->_topImage.valid())
                        {
                            data->_topImageAtlasIndex = imagesToAddToAtlas.size();
                            imagesToAddToAtlas.push_back(data->_topImage.get());
                            imagecache[uri] = data->_topImage.get();
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to load billboard top image from \"" << uri.full() << "\"" << std::endl;
                        }
                    }
                }

                if (data->_sideImage.valid() || data->_model.valid())
                {
                    // every asset gets a unique ID:
                    data->_assetID = assetIDGen++;
                    _liveAssets.push_back(data.get());
                }
            }
        }
    }

    // Add all discovered images to the atlas.
    if (atlas)
    {
        for(ImageVector::const_iterator i = imagesToAddToAtlas.begin();
            i != imagesToAddToAtlas.end();
            ++i)
        {
            atlas->addImage(i->get());
        }
    }


    if (_liveAssets.empty())
    {
        OE_WARN << LC << "Failed to load any assets!" << std::endl;
        // TODO: something?
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
            //startingAssetIndex = a;
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
        }

        float height = data->_asset->options().height().get();
        if (data->_asset->options().height().isSet() == false &&
            data->_modelAABB.valid())
        {
            height = data->_modelAABB.zMax() - data->_modelAABB.zMin();
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
                << ", " << data->_modelAtlasIndex
                << ", " << data->_sideImageAtlasIndex
                << ", " << data->_topImageAtlasIndex
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
        << "); \n";

    std::stringstream assetWrapperBuf;
    assetWrapperBuf <<
        "struct oe_gc_Asset { \n"
        "    int assetId; \n"
        "    int modelId; \n"
        "    int atlasIndexModel; \n" // first index of model's textures
        "    int atlasIndexSide; \n"
        "    int atlasIndexTop; \n"
        "    float width; \n"
        "    float height; \n"
        "    float sizeVariation; \n"
        "    float fill; \n"
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
        << lutbuf.str() << "\n");

    return shader;
}

osg::StateSet*
GroundCoverLayer::getZoneStateSet(unsigned index) const
{
    return index < _zoneStateSets.size() ? _zoneStateSets[index].get() : NULL;
}

GeometryCloud*
GroundCoverLayer::createGeometryCloud(TextureAtlas* atlas) const
{
    GeometryCloud* geomCloud = new GeometryCloud();

    // assign our pre-existing texture atlas so they will be combined into one.
    geomCloud->setAtlas(atlas);

    // First entry is the parametric group. Any instance without a 3D model,
    // or that is out of model range, gets rendered as a parametric billboard.
    osg::Geometry* pg = createParametricGeometry();
    geomCloud->add( pg, pg->getVertexArray()->getNumElements() );

    // Add each 3D model to the geometry cloud and update the texture
    // atlas along the way.
    UnorderedMap<int, int> visited;
    for(AssetDataVector::const_iterator a = _liveAssets.begin();
        a != _liveAssets.end();
        ++a)
    {
        AssetData* asset = a->get();
        if (asset->_modelID >= 0)
        {
            int atlasIndex = -1;
            UnorderedMap<int, int>::iterator i = visited.find(asset->_modelID);
            if (i == visited.end())
            {
                // adds the model, and returns the texture atlas index of the
                // FIRST discovered texture from the model.
                atlasIndex = geomCloud->add(asset->_model.get());
                visited[asset->_modelID] = atlasIndex;
            }
            else
            {
                atlasIndex = i->second;
            }
            asset->_modelAtlasIndex = atlasIndex;
        }
    }

    return geomCloud;
}
