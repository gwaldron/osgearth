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
#include <osgEarth/NetworkMonitor>
#include <osgEarth/Registry>

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

    options().features().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    _session = 0L;
}

osg::ref_ptr<osg::Node>
TiledFeatureModelLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (progress && progress->isCanceled())
        return nullptr;

    NetworkMonitor::ScopedRequestLayer layerRequest(getName());

    // Get features for this key
    Query query;
    query.tileKey() = key;

    GeoExtent dataExtent = key.getExtent();

    // set up for feature indexing if appropriate:
    osg::ref_ptr< FeatureSourceIndexNode > index = 0L;

    if (_featureIndex.valid())
    {
        index = new FeatureSourceIndexNode(_featureIndex.get());
    }

    FilterContext fc(_session.get(), new FeatureProfile(dataExtent), dataExtent, index);

    GeomFeatureNodeFactory factory(options());

    if (progress && progress->isCanceled())
        return nullptr;

    auto cursor = getFeatureSource()->createFeatureCursor(query, _filters, &fc, progress);

    osg::ref_ptr<osg::Node> node = new osg::Group;
    if (cursor)
    {
        if (progress && progress->isCanceled())
            return nullptr;

        FeatureList features;
        cursor->fill(features);

        if (options().cropFeaturesToTile() == true)
        {
            FeatureList tocrop;
            tocrop.swap(features);

            for (auto& feature : tocrop)
            {
                if (auto cropped = feature->getGeometry()->crop(dataExtent.bounds()))
                {
                    feature->setGeometry(cropped.get());
                    features.push_back(feature);
                }
            }
        }

        if (getStyleSheet() && getStyleSheet()->getSelectors().size() > 0)
        {
            osg::Group* group = new osg::Group;

            for(auto& i : getStyleSheet()->getSelectors())
            {
                using StyleToFeaturesMap = std::map<std::string, FeatureList>;
                StyleToFeaturesMap styleToFeatures;

                // pull the selected style...
                const StyleSelector& sel = i.second;

                if (sel.styleExpression().isSet())
                {
                    // establish the working bounds and a context:
                    StringExpression styleExprCopy(sel.styleExpression().get());

                    for(auto& feature : features)
                    {
                        //TODO: rename this?
                        feature->set("level", (long long)key.getLevelOfDetail());

                        const std::string& styleString = feature->eval(styleExprCopy, &fc); 

                        if (!styleString.empty() && styleString != "null")
                        {
                            styleToFeatures[styleString].push_back(feature);
                        }

                        if (progress && progress->isCanceled())
                            return nullptr;
                    }
                }

                std::unordered_map<std::string, Style> literal_styles;

                //OE_INFO << "Found styles:" << std::endl;

                for (StyleToFeaturesMap::iterator itr = styleToFeatures.begin(); itr != styleToFeatures.end(); ++itr)
                {
                    const std::string& styleString = itr->first;
                    Style* style = nullptr;

                    //OE_INFO << "  " << styleString << std::endl;

                    if (styleString.length() > 0 && styleString[0] == '{')
                    {
                        Config conf("style", styleString);
                        conf.setReferrer(sel.styleExpression().get().uriContext().referrer());
                        conf.set("type", "text/css");
                        Style& literal_style = literal_styles[conf.toJSON()];
                        if (literal_style.empty())
                            literal_style = Style(conf);
                        style = &literal_style;
                    }
                    else
                    {
                        // no default fallback!
                        style = getStyleSheet()->getStyle(styleString, false);
                    }

                    if (style)
                    {
                        osg::Group* styleGroup = factory.getOrCreateStyleGroup(*style, _session.get());
                        osg::ref_ptr<osg::Node>  styleNode;
                        osg::ref_ptr<FeatureListCursor> cursor = new FeatureListCursor(itr->second);
                        Query query;
                        factory.createOrUpdateNode(cursor.get(), *style, fc, styleNode, query);
                        if (styleNode.valid())
                        {
                            styleGroup->addChild(styleNode);
                            if (!group->containsNode(styleGroup))
                            {
                                group->addChild(styleGroup);
                            }
                        }
                    }
                }
            }


            node = group;
        }
        else if (getStyleSheet() && getStyleSheet()->getDefaultStyle())
        {
            osg::ref_ptr< FeatureListCursor> cursor = new FeatureListCursor(features);
            osg::ref_ptr< osg::Group > group = new osg::Group;
            osg::ref_ptr< osg::Group > styleGroup = factory.getOrCreateStyleGroup(*getStyleSheet()->getDefaultStyle(), _session.get());
            osg::ref_ptr< osg::Node>  styleNode;
            factory.createOrUpdateNode(cursor.get(), *getStyleSheet()->getDefaultStyle(), fc, styleNode, query);
            if (styleNode.valid())
            {
                group->addChild(styleGroup);
                styleGroup->addChild(styleNode);
                node = group;
            }
        }
    }

    if (!node->getBound().valid())
    {
        return nullptr;
    }

    if (index.valid())
    {
        index->addChild(node);
        return index;
    }

    return node;
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
