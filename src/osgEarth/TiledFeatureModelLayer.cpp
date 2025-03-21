/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/TiledFeatureModelLayer>
#include <osgEarth/Registry>
#include <osgEarth/FeatureStyleSorter>

using namespace osgEarth;

#define LC "[TiledFeatureModelLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(TiledFeatureModel, TiledFeatureModelLayer);

//...........................................................................

TiledFeatureModelLayer::Options::Options() :
    TiledModelLayer::Options(),
    FeatureModelOptions(),
    GeometryCompilerOptions()
{
    fromConfig(_conf);
}

TiledFeatureModelLayer::Options::Options(const ConfigOptions& options) :
    TiledModelLayer::Options(options),
    FeatureModelOptions(options),
    GeometryCompilerOptions(options)
{
    fromConfig(_conf);
}

void TiledFeatureModelLayer::Options::fromConfig(const Config& conf)
{
    features().get(conf, "features");
    conf.get("crop_features", cropFeaturesToTile());
}

Config
TiledFeatureModelLayer::Options::getConfig() const
{
    Config conf = TiledModelLayer::Options::getConfig();

    Config fmConf = FeatureModelOptions::getConfig();
    conf.merge(fmConf);

    Config gcConf = GeometryCompilerOptions::getConfig();
    conf.merge(gcConf);

    features().set(conf, "features");

    conf.set("crop_features", cropFeaturesToTile());

    return conf;
}

void TiledFeatureModelLayer::Options::mergeConfig(const Config& conf)
{
    TiledModelLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

//...........................................................................

OE_LAYER_PROPERTY_IMPL(TiledFeatureModelLayer, bool, AlphaBlending, alphaBlending);
OE_LAYER_PROPERTY_IMPL(TiledFeatureModelLayer, bool, EnableLighting, enableLighting);

TiledFeatureModelLayer::~TiledFeatureModelLayer()
{
    //NOP
}

void
TiledFeatureModelLayer::init()
{
    TiledModelLayer::init();
}

Config
TiledFeatureModelLayer::getConfig() const
{
    Config conf = TiledModelLayer::getConfig();
    return conf;
}

void
TiledFeatureModelLayer::setFeatureSource(FeatureSource* source)
{
    if (getFeatureSource() != source)
    {
        options().features().setLayer(source);

        if (source && source->getStatus().isError())
        {
            setStatus(source->getStatus());
            return;
        }

        dirty();
    }
}

FeatureSource*
TiledFeatureModelLayer::getFeatureSource() const
{
    return options().features().getLayer();
}

void
TiledFeatureModelLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        options().styleSheet().setLayer(value);
        dirty();
    }
}

StyleSheet*
TiledFeatureModelLayer::getStyleSheet() const
{
    return options().styleSheet().getLayer();
}

Status
TiledFeatureModelLayer::openImplementation()
{
    Status parent = TiledModelLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status fsStatus = options().features().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus = options().styleSheet().open(getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    return Status::NoError;
}

Status
TiledFeatureModelLayer::closeImplementation()
{
    super::closeImplementation();
    options().features().close();
    options().styleSheet().close();
    return getStatus();
}

const GeoExtent&
TiledFeatureModelLayer::getExtent() const
{
    static GeoExtent s_invalid;

    FeatureSource* fs = getFeatureSource();
    return fs && fs->getFeatureProfile() ?
        fs->getFeatureProfile()->getExtent() :
        s_invalid;
}

void
TiledFeatureModelLayer::addedToMap(const Map* map)
{
    OE_TEST << LC << "addedToMap" << std::endl;
    
    options().features().addedToMap(map);
    options().styleSheet().addedToMap(map);

    if (getFeatureSource() && getStyleSheet())
    {
        if (getFeatureSource()->getStatus().isError())
        {
            setStatus(getFeatureSource()->getStatus());
            return;
        }

        // Save a reference to the map since we'll need it to
        // create a new session object later.
        _session = new Session(
            map,
            getStyleSheet(),
            getFeatureSource(),
            getReadOptions());

        // connect the session to the features:
        _session->setFeatureSource(getFeatureSource());
        _session->setResourceCache(new ResourceCache());

        if (options().featureIndexing()->enabled() == true)
        {
            FeatureSourceIndexOptions indexOptions;
            indexOptions.enabled() = true;
            indexOptions.embedFeatures() = true;

            _featureIndex = new FeatureSourceIndex(
                getFeatureSource(),
                Registry::objectIndex(),
                indexOptions);
        }
    }
    else
    {
        OE_WARN << LC << "Missing either feature source or stylesheet - nothing will render" << std::endl;
    }

    _filters = FeatureFilterChain::create(options().filters(), getReadOptions());

    TiledModelLayer::addedToMap(map);    
}

void
TiledFeatureModelLayer::removedFromMap(const Map* map)
{
    super::removedFromMap(map);

    _filters.clear();
    _session = nullptr;
    _featureIndex = nullptr;

    options().features().removedFromMap(map);
    options().styleSheet().removedFromMap(map);
}

osg::ref_ptr<osg::Node>
TiledFeatureModelLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (progress && progress->isCanceled())
        return nullptr;

    // Get features for this key
    // set up for feature indexing if appropriate:
    osg::ref_ptr<FeatureSourceIndexNode> index = 0L;
    if (_featureIndex.valid())
    {
        index = new FeatureSourceIndexNode(_featureIndex.get());
    }

    GeomFeatureNodeFactory factory(options());

    if (progress && progress->isCanceled())
        return nullptr;

    auto featureProfile = getFeatureSource()->getFeatureProfile();
    OE_SOFT_ASSERT_AND_RETURN(featureProfile, {});
    
    FilterContext context(_session.get(), featureProfile, key.getExtent(), index);
    Query query(key);


    osg::ref_ptr<osg::Group> group = new osg::Group();

    auto compile = [&](const Style& style, FeatureList& features, ProgressCallback* progress)
        {
            if (options().cropFeaturesToTile() == true)
            {
                FeatureList temp;
                temp.swap(features);

                auto extent = key.getExtent().transform(featureProfile->getSRS());
                for (auto& feature : temp)
                {
                    auto cropped = feature->getGeometry()->crop(extent.bounds());
                    if (cropped)
                    {
                        feature->setGeometry(cropped);
                        features.emplace_back(feature);
                    }
                }
            }

            for (auto& feature : features)
            {
                feature->set("level", (long long)key.getLOD());
            }

            osg::ref_ptr<osg::Node> node;
            FeatureListCursor cursor(features);
            if (factory.createOrUpdateNode(&cursor, style, context, node, query))
            {
                group->addChild(node);
            }
        };

    FeatureStyleSorter().sort(key, {}, _session.get(), _filters, nullptr, compile, progress);

    if (group->getNumChildren() == 0 || group->getBound().valid() == false)
    {
        return {};
    }

    if (index.valid())
    {
        index->addChild(group);
        group = index;
    }

    return group;
}

const Profile*
TiledFeatureModelLayer::getProfile() const
{
    OE_HARD_ASSERT(getFeatureSource() != nullptr);
    OE_HARD_ASSERT(getFeatureSource()->getFeatureProfile() != nullptr);
    OE_SOFT_ASSERT_AND_RETURN(getFeatureSource() != nullptr, nullptr);
    OE_SOFT_ASSERT_AND_RETURN(getFeatureSource()->getFeatureProfile() != nullptr, nullptr);

    static const Profile* s_fallback = Profile::create(Profile::GLOBAL_GEODETIC);
    auto profile = getFeatureSource()->getFeatureProfile()->getTilingProfile();
    return profile ? profile : s_fallback;
}

unsigned
TiledFeatureModelLayer::getMinLevel() const
{
    if (options().minLevel().isSet())
        return options().minLevel().value();
    else
        return getFeatureSource()->getFeatureProfile()->getFirstLevel();
}

unsigned
TiledFeatureModelLayer::getMaxLevel() const
{
    if (options().maxLevel().isSet())
        return options().maxLevel().value();
    else
        return getFeatureSource()->getFeatureProfile()->getMaxLevel();
}
