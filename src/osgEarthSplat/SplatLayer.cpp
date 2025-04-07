/* osgEarth
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "SplatLayer"
#include "SplatShaders"
#include "NoiseTextureFactory"
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgUtil/CullVisitor>
#include <osg/BlendFunc>
#include <osg/Drawable>
#include <cstdlib> // getenv

#define LC "[SplatLayer] " << getName() << ": "

#define SPLAT_SAMPLER    "oe_splatTex"
#define NOISE_SAMPLER    "oe_splat_noiseTex"
#define LUT_SAMPLER      "oe_splat_coverageLUT"

using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(splat, SplatLayer);
REGISTER_OSGEARTH_LAYER(splatimage, SplatLayer);
REGISTER_OSGEARTH_LAYER(splat_imagery, SplatLayer);

//........................................................................

Config
SplatLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    conf.set("land_cover_layer", landCoverLayer() );

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
SplatLayer::Options::fromConfig(const Config& conf)
{
    conf.get("land_cover_layer", landCoverLayer() );

    const Config* zones = conf.child_ptr("zones");
    if (zones) {
        const ConfigSet& children = zones->children();
        for (ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i) {
            _zones.push_back(ZoneOptions(*i));
        }
    }
    else { // no zones?
        optional<SurfaceOptions> surface;
        conf.get("surface", surface);
        if (surface.isSet())
        {
            ZoneOptions zo;
            zo.surface() = surface;
            _zones.push_back(zo);
        }
    }
}

//........................................................................

void
SplatLayer::ZoneSelector::operator()(osg::Node* node, osg::NodeVisitor* nv) const
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

            osg::StateSet* zoneStateSet = nullptr;
            Surface* surface = _layer->_zones[zoneIndex]->getSurface();
            if (surface)
            {
                zoneStateSet = surface->getStateSet();
            }

            OE_SOFT_ASSERT_AND_RETURN(zoneStateSet != nullptr, void());

            cv->pushStateSet(zoneStateSet);
            traverse(node, nv);
            cv->popStateSet();
        }
    }
    else
    {
        traverse(node, nv);
    }
}

//........................................................................

void
SplatLayer::init()
{
    VisibleLayer::init();

    _zonesConfigured = false;

    _editMode = (::getenv("OSGEARTH_SPLAT_EDIT") != 0L); // TODO deprecate
    _gpuNoise = (::getenv("OSGEARTH_SPLAT_GPU_NOISE") != 0L); // TODO deprecate

    setRenderType(osgEarth::Layer::RENDERTYPE_TERRAIN_SURFACE);

    for (std::vector<ZoneOptions>::const_iterator i = options().zones().begin();
        i != options().zones().end();
        ++i)
    {
        osg::ref_ptr<Zone> zone = new Zone(*i);
        _zones.push_back(zone.get());
    }

    setCullCallback(new ZoneSelector(this));
}

void
SplatLayer::setLandCoverDictionary(LandCoverDictionary* layer)
{
    _landCoverDict.setLayer(layer);
    if (layer)
        buildStateSets();
}

void
SplatLayer::setLandCoverLayer(LandCoverLayer* layer)
{
    _landCoverLayer.setLayer(layer);
    if (layer) {
        buildStateSets();
    }
}

Status
SplatLayer::openImplementation()
{
    if (GLUtils::useNVGL())
    {
        return Status(Status::ResourceUnavailable, "Layer is not compatible with NVGL");
    }

    return VisibleLayer::openImplementation();
}

void
SplatLayer::addedToMap(const Map* map)
{
    VisibleLayer::addedToMap(map);

    if (!getLandCoverDictionary())
        setLandCoverDictionary(map->getLayer<LandCoverDictionary>());

    if (!getLandCoverLayer())
        setLandCoverLayer(map->getLayer<LandCoverLayer>());

    for (Zones::iterator zone = _zones.begin(); zone != _zones.end(); ++zone)
    {
        zone->get()->configure(map, getReadOptions());
    }

    _zonesConfigured = true;
    
    buildStateSets();
}

void
SplatLayer::removedFromMap(const Map* map)
{
    VisibleLayer::removedFromMap(map);
}

void
SplatLayer::prepareForRendering(TerrainEngine* engine)
{
    VisibleLayer::prepareForRendering(engine);

    TerrainResources* res = engine->getResources();
    if (res)
    {
        // TODO.
        // These reservations are Layer-specific, so we should add the
        // capability to TerrainResources to support per-Layer reservations.
        if (_splatBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_splatBinding, this, "Splat texture") == false)
            {
                OE_WARN << LC << "No texture unit available for splatting texture\n";
            }
        }

        if (_lutBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_lutBinding, this, "Splatting LUT") == false)
            {
                OE_WARN << LC << "No texture unit available for splatting LUT\n";
            }
        }

        if (_noiseBinding.valid() == false)
        {
            if (res->reserveTextureImageUnitForLayer(_noiseBinding, this, "Splat noise sampler") == false)
            {
                OE_WARN << LC << "No texture unit available for splatting Noise function\n";
            }
        }

        if (_splatBinding.valid() && _lutBinding.valid())
        {
            buildStateSets();
        }
    }
}

void
SplatLayer::buildStateSets()
{
    // assert we have the necessary TIUs:
    if (_splatBinding.valid() == false || _lutBinding.valid() == false) {
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

    // Load all the splatting textures
    for (Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        Zone* zone = z->get();
        Surface* surface = z->get()->getSurface();
        if (surface == 0L)
        {
            OE_WARN << LC << "No surface defined for zone " << zone->getName() << std::endl;
            return;
        }
        if (surface->loadTextures(getLandCoverDictionary(), getReadOptions()) == false)
        {
            OE_WARN << LC << "Texture load failed for zone " << zone->getName() << "\n";
            return;
        }
    }

    // Set up the zone-specific elements:
    for (Zones::iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        Zone* zone = z->get();

        osg::StateSet* zoneStateset = zone->getSurface()->getOrCreateStateSet();
        zoneStateset->setName("Splat Zone");

        // The texture array for the zone:
        const SplatTextureDef& texdef = zone->getSurface()->getTextureDef();

        // apply the splatting texture catalog:
        zoneStateset->setTextureAttribute(_splatBinding.unit(), texdef._texture.get());

        // apply the buffer containing the coverage-to-splat LUT:
        zoneStateset->setTextureAttribute(_lutBinding.unit(), texdef._splatLUTBuffer.get());

        OE_DEBUG << LC << "Installed getRenderInfo for zone \"" << zone->getName() << "\" (uid=" << zone->getUID() << ")\n";
    }

    // Next set up the elements that apply to all zones:
    osg::StateSet* stateset = this->getOrCreateStateSet();

    // Bind the texture image unit:
    stateset->addUniform(new osg::Uniform(SPLAT_SAMPLER, _splatBinding.unit()));

    // install the uniform for the splat LUT.
    stateset->addUniform(new osg::Uniform(LUT_SAMPLER, _lutBinding.unit()));
        
    if (_noiseBinding.valid())
    {
        NoiseTextureFactory noise;
        osg::ref_ptr<osg::Texture> noiseTexture = noise.create(256u, 1u);
        stateset->setTextureAttribute(_noiseBinding.unit(), noiseTexture.get());
        stateset->addUniform(new osg::Uniform(NOISE_SAMPLER, _noiseBinding.unit()));
        stateset->setDefine("OE_SPLAT_NOISE_SAMPLER", NOISE_SAMPLER);
    }

    stateset->addUniform(new osg::Uniform("oe_splat_scaleOffsetInt", 0));
    stateset->addUniform(new osg::Uniform("oe_splat_noiseScale", 12.0f));
    stateset->addUniform(new osg::Uniform("oe_splat_detailRange", 100000.0f));

    if (_editMode)
        stateset->setDefine("OE_SPLAT_EDIT_MODE");

    if (_gpuNoise)
        stateset->setDefine("OE_SPLAT_GPU_NOISE");

    stateset->setDefine("OE_USE_NORMAL_MAP");

    SplattingShaders splatting;
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    vp->setName(typeid(*this).name());
    splatting.load(vp, splatting.VertModel);
    splatting.load(vp, splatting.VertView);
    splatting.load(vp, splatting.Frag);
    splatting.load(vp, splatting.Util);

    OE_DEBUG << LC << "Statesets built!! Ready!\n";
}


void
SplatLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    for (Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        z->get()->resizeGLObjectBuffers(maxSize);
    }

    VisibleLayer::resizeGLObjectBuffers(maxSize);
}

void
SplatLayer::releaseGLObjects(osg::State* state) const
{
    for (Zones::const_iterator z = _zones.begin(); z != _zones.end(); ++z)
    {
        z->get()->releaseGLObjects(state);
    }

    VisibleLayer::releaseGLObjects(state);
}


Config
SplatLayer::getConfig() const
{
    Config c = VisibleLayer::getConfig();
    if (_landCoverDict.isSetByUser())
        c.set(_landCoverDict.getLayer()->getConfig());
    if (_landCoverLayer.isSetByUser())
        c.set(_landCoverLayer.getLayer()->getConfig());
    return c;
}
