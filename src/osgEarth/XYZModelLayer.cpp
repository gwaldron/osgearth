/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/XYZModelLayer>
#include <osgEarth/Metrics>
#include <osgEarth/Registry>
#include <osgEarth/Progress>

using namespace osgEarth;

#define LC "[XYZModelLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(XYZModel, XYZModelLayer);

//...........................................................................

void XYZModelLayer::Options::fromConfig(const Config& conf)
{
    invertY().setDefault(false);        
    conf.get("url", url());
    conf.get("min_level", minLevel());
    conf.get("max_level", maxLevel());    
    conf.get("profile", profile());
}

Config
XYZModelLayer::Options::getConfig() const
{
    Config conf = TiledModelLayer::Options::getConfig();
    conf.set("url", url());
    conf.set("min_level", minLevel());
    conf.set("max_level", maxLevel());
    conf.set("profile", profile());

    return conf;
}

//...........................................................................

OE_LAYER_PROPERTY_IMPL(XYZModelLayer, URI, URL, url);

void
XYZModelLayer::setMinLevel(unsigned value) {
    options().minLevel() = value;
}

unsigned
XYZModelLayer::getMinLevel() const {
    return options().minLevel().get();
}

void
XYZModelLayer::setMaxLevel(unsigned value) {
    options().maxLevel() = value;
}

unsigned
XYZModelLayer::getMaxLevel() const {
    return options().maxLevel().get();
}

void XYZModelLayer::init()
{
    super::init();    
}

void XYZModelLayer::addedToMap(const Map* map)
{
    _readOptions = osgEarth::Registry::instance()->cloneOrCreateOptions(getReadOptions());
    _readOptions->setObjectCacheHint(osgDB::Options::CACHE_IMAGES);

#if 0
    _statesetCache = new StateSetCache();

    if (*options().useNVGL() == true && GLUtils::useNVGL() && !_textures.valid())
    {
        _textures = new TextureArena();
        getOrCreateStateSet()->setAttribute(_textures, 1);

        // auto release requires that we install this update callback!
        _textures->setAutoRelease(true);

        getNode()->addUpdateCallback(new LambdaCallback<>([this](osg::NodeVisitor& nv)
            {
                _textures->update(nv);
                return true;
            }));

        getOrCreateStateSet()->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }
#endif

    super::addedToMap(map);
}

XYZModelLayer::~XYZModelLayer()
{
    //NOP
}

void XYZModelLayer::setProfile(const Profile* profile)
{
    _profile = profile;
    if (_profile)
    {
        options().profile() = profile->toProfileOptions();
    }
}

Config
XYZModelLayer::getConfig() const
{
    Config conf = TiledModelLayer::getConfig();
    return conf;
}

Status
XYZModelLayer::openImplementation()
{
    Status parent = super::openImplementation();
    if (parent.isError())
        return parent;

    _profile = Profile::create(*options().profile());

    return Status::NoError;
}

osg::ref_ptr<osg::Node>
XYZModelLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;
    if (progress && progress->isCanceled())
        return nullptr;

    unsigned x, y;
    key.getTileXY(x, y);
    unsigned cols = 0, rows = 0;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), cols, rows);
    unsigned inverted_y = rows - y - 1;

    if (*options().invertY() == true)
    {
        y = inverted_y;
    }

    std::string location = options().url()->full();

    // support OpenLayers template style:
    replaceIn(location, "${x}", Stringify() << x);
    replaceIn(location, "${y}", Stringify() << y);
    replaceIn(location, "${-y}", Stringify() << inverted_y);
    replaceIn(location, "${z}", Stringify() << key.getLevelOfDetail());

    // failing that, legacy osgearth style:
    replaceIn(location, "{x}", Stringify() << x);
    replaceIn(location, "{y}", Stringify() << y);
    replaceIn(location, "{-y}", Stringify() << inverted_y);
    replaceIn(location, "{z}", Stringify() << key.getLevelOfDetail());

    URI myUri(location, options().url()->context());

    osg::ref_ptr<osg::Node> node = myUri.readNode(_readOptions.get()).getNode();

    if (node.valid())
    {
#if 0
        if (_textures.valid())
        {
            auto xform = findTopMostNodeOfType<osg::MatrixTransform>(node.get());

            // Convert the geometry into chonks
            ChonkFactory factory(_textures);

            factory.setGetOrCreateFunction(
                ChonkFactory::getWeakTextureCacheFunction(
                    _texturesCache, _texturesCacheMutex));

            osg::ref_ptr<ChonkDrawable> drawable = new ChonkDrawable();

            if (xform)
            {
                for (unsigned i = 0; i < xform->getNumChildren(); ++i)
                {
                    drawable->add(xform->getChild(i), factory);
                }
                xform->removeChildren(0, xform->getNumChildren());
                xform->addChild(drawable);
                node = xform;
            }
            else
            {
                if (drawable->add(node.get(), factory))
                {
                    node = drawable;
                }
            }
        }
        else
        {
            osgEarth::Registry::shaderGenerator().run(node.get(), _statesetCache);
        }
#endif
        return node.release();
    }
    return nullptr;
}
