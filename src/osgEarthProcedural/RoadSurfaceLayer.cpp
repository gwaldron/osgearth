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
#include "RoadSurfaceLayer"
#include <osgEarth/Utils>
#include <osgEarth/Map>
#include <osgEarth/TileRasterizer>
#include <osgEarth/VirtualProgram>
#include <osgEarth/FilterContext>
#include <osgEarth/GeometryCompiler>
#include <osgEarth/Containers>
#include <osgEarth/Metrics>

#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Procedural;

#define LC "[RoadSurfaceLayer] "


REGISTER_OSGEARTH_LAYER(roadsurface, RoadSurfaceLayer);
REGISTER_OSGEARTH_LAYER(road_surface, RoadSurfaceLayer);

//........................................................................

Config
RoadSurfaceLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");
    styleSheet().set(conf, "styles");
    conf.set("buffer_width", featureBufferWidth());
    return conf;
}

void
RoadSurfaceLayer::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");
    conf.get("buffer_width", featureBufferWidth());
}

//........................................................................

void
RoadSurfaceLayer::setFeatureBufferWidth(const Distance& value) {
    options().featureBufferWidth() = value;
}

const Distance&
RoadSurfaceLayer::getFeatureBufferWidth() const {
    return options().featureBufferWidth().get();
}

void
RoadSurfaceLayer::init()
{
    ImageLayer::init();

    // Generate Mercator tiles by default.
    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));

    if (getName().empty())
        setName("Road surface");

    _lru = std::unique_ptr<FeatureListCache>(new FeatureListCache(true, 1u));
}

Status
RoadSurfaceLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // assert a feature source:
    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    Status ssStatus = options().styleSheet().open(getReadOptions());
    if (ssStatus.isError())
        return ssStatus;

    // Create a rasterizer for rendering nodes to images.
    if (_rasterizer.valid() == false)
    {
        _rasterizer = new TileRasterizer(
            getTileSize(),
            getTileSize());
    }

    return Status::NoError;
}

Status
RoadSurfaceLayer::closeImplementation()
{
    _rasterizer = nullptr;
    return ImageLayer::closeImplementation();
}

void
RoadSurfaceLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    // create a session for feature processing based in the Map,
    // but don't set the feature source yet.
    _session = new Session(map, getStyleSheet(), nullptr, getReadOptions());
    _session->setResourceCache(new ResourceCache());

    options().featureSource().addedToMap(map);
    options().styleSheet().addedToMap(map);
}

void
RoadSurfaceLayer::removedFromMap(const Map* map)
{
    ImageLayer::removedFromMap(map);
    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);
    _session = nullptr;
}

osg::Node*
RoadSurfaceLayer::getNode() const
{
    return _rasterizer.get();
}

void
RoadSurfaceLayer::setFeatureSource(FeatureSource* layer)
{
    if (getFeatureSource() != layer)
    {
        options().featureSource().setLayer(layer);
        if (layer && layer->getStatus().isError())
        {
            setStatus(layer->getStatus());
        }
    }
}

FeatureSource*
RoadSurfaceLayer::getFeatureSource() const
{
    return options().featureSource().getLayer();
}

void
RoadSurfaceLayer::setStyleSheet(StyleSheet* value)
{
    options().styleSheet().setLayer(value);
}

StyleSheet*
RoadSurfaceLayer::getStyleSheet() const
{
    return options().styleSheet().getLayer();
}

namespace
{
    typedef std::vector< std::pair< Style, FeatureList > > StyleToFeatures;

    void addFeatureToMap(Feature* feature, const Style& style, StyleToFeatures& map)
    {
        bool added = false;

        if (!style.getName().empty())
        {
            // Try to find the style by name
            for (int i = 0; i < map.size(); i++)
            {
                if (map[i].first.getName() == style.getName())
                {
                    map[i].second.push_back(feature);
                    added = true;
                    break;
                }
            }
        }

        if (!added)
        {
            FeatureList list;
            list.push_back(feature);
            map.push_back(std::pair< Style, FeatureList>(style, list));
        }
    }

    void sortFeaturesIntoStyleGroups(StyleSheet* styles, FeatureList& features, FilterContext &context, StyleToFeatures& map)
    {
        if (styles == nullptr)
            return;

        if (styles->getSelectors().size() > 0)
        {
            for (StyleSelectors::const_iterator i = styles->getSelectors().begin();
                i != styles->getSelectors().end();
                ++i)
            {
                const StyleSelector& sel = i->second;

                if (sel.styleExpression().isSet())
                {
                    // establish the working bounds and a context:
                    StringExpression styleExprCopy(sel.styleExpression().get());

                    for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                    {
                        Feature* feature = itr->get();

                        // resolve the style:
                        Style combinedStyle;

                        if (feature->style().isSet())
                        {
                            // embedde style:
                            combinedStyle = feature->style().get();
                        }
                        else
                        {
                            // evaluated style:
                            const std::string& styleString = feature->eval(styleExprCopy, &context);
                            if (!styleString.empty() && styleString != "null")
                            {
                                // if the style string begins with an open bracket, it's an inline style definition.
                                if (styleString.length() > 0 && styleString[0] == '{')
                                {
                                    Config conf("style", styleString);
                                    conf.setReferrer(sel.styleExpression().get().uriContext().referrer());
                                    conf.set("type", "text/css");
                                    combinedStyle = Style(conf);
                                }

                                // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
                                // style in this case: for style expressions, the user must be explicity about
                                // default styling; this is because there is no other way to exclude unwanted
                                // features.
                                else
                                {
                                    const Style* selectedStyle = styles->getStyle(styleString, false);
                                    if (selectedStyle)
                                        combinedStyle = *selectedStyle;
                                }
                            }
                        }

                        if (!combinedStyle.empty())
                        {
                            addFeatureToMap(feature, combinedStyle, map);
                        }

                    }
                }
            }
        }
        else
        {
            const Style* style = styles->getDefaultStyle();
            for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
            {
                Feature* feature = itr->get();
                // resolve the style:
                if (feature->style().isSet())
                {
                    addFeatureToMap(feature, feature->style().get(), map);
                }
                else
                {
                    addFeatureToMap(feature, *style, map);
                }
            }
        }
    }
}

GeoImage
RoadSurfaceLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return GeoImage::INVALID;
    }

    // take local refs to isolate this method from the member objects
    osg::ref_ptr<FeatureSource> featureSource(getFeatureSource());
    osg::ref_ptr<StyleSheet> styleSheet(getStyleSheet());
    osg::ref_ptr<TileRasterizer> rasterizer(_rasterizer);
    osg::ref_ptr<Session> session(_session);

    if (!featureSource.valid())
    {
        setStatus(Status(Status::ServiceUnavailable, "No feature source"));
        return GeoImage::INVALID;
    }

    if (featureSource->getStatus().isError())
    {
        setStatus(featureSource->getStatus());
        return GeoImage::INVALID;
    }

    osg::ref_ptr<const FeatureProfile> featureProfile = featureSource->getFeatureProfile();
    if (!featureProfile.valid())
    {
        setStatus(Status(Status::ConfigurationError, "Feature profile is missing"));
        return GeoImage::INVALID;
    }

    if (!rasterizer.valid() || !session.valid())
    {
        return GeoImage::INVALID;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status(Status::ConfigurationError, "Feature profile has no SRS"));
        return GeoImage::INVALID;
    }

    // Fetch the set of features to render
    FeatureList features;
    getFeatures(featureSource.get(), key, features, progress);

    if (!features.empty())
    {
        GeoExtent featureExtent = key.getExtent().transform(featureSRS);

        // Create the output extent:
        GeoExtent outputExtent = key.getExtent();

        // Establish a local tangent plane near the output extent. This will allow
        // the compiler to render the tile in a location cartesian space.
        const SpatialReference* keySRS = outputExtent.getSRS();
        osg::Vec3d pos(outputExtent.west(), outputExtent.south(), 0);
        osg::ref_ptr<const SpatialReference> srs = keySRS->createTangentPlaneSRS(pos);
        outputExtent = outputExtent.transform(srs.get());

        // Set the LTP as our output SRS.
        // The geometry compiler will transform all our features into the
        // LTP so we can render using an orthographic camera (TileRasterizer)
        FilterContext fc(session.get(), featureProfile.get(), featureExtent);
        fc.setOutputSRS(outputExtent.getSRS());

        // compile the features into a node.
        GeometryCompiler compiler;
        StyleToFeatures mapping;
        sortFeaturesIntoStyleGroups(styleSheet.get(), features, fc, mapping);
        osg::ref_ptr< osg::Group > group;
        if (!mapping.empty())
        {
            OE_PROFILING_ZONE_NAMED("Style");

            group = new osg::Group();
            for (unsigned int i = 0; i < mapping.size(); i++)
            {
                osg::ref_ptr<osg::Node> node = compiler.compile(mapping[i].second, mapping[i].first, fc);
                if (node.valid() && node->getBound().valid())
                {
                    group->addChild(node);
                }
            }
        }

        if (group && group->getBound().valid())
        {
            OE_PROFILING_ZONE_NAMED("Rasterize");

            group->setName(key.str());

            Future<osg::ref_ptr<osg::Image>> result = rasterizer->render(
                group.release(),
                outputExtent);

            osg::ref_ptr<ProgressCallback> local_progress = new ProgressCallback(
                progress,
                [&]() { return !isOpen(); }
            );

            // Immediately blocks on the result.
            const osg::ref_ptr<osg::Image>& image = result.join(local_progress.get());

            if (image.valid() && image->data() != nullptr)
                return GeoImage(image.get(), key.getExtent());
            else
                return GeoImage::INVALID;
        }
    }

    return GeoImage::INVALID;
}

void
RoadSurfaceLayer::getFeatures(
    FeatureSource* fs,
    const TileKey& key,
    FeatureList& output,
    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    OE_SOFT_ASSERT_AND_RETURN(fs != nullptr, void());

    // Get the collection of keys accounting for the buffer width
    std::unordered_set<TileKey> keys;
    fs->getKeys(key, options().featureBufferWidth().get(), keys);

    // Collect all the features, using a small LRU cache and a
    // Gate to optimize fetching and sharing with other threads
    osg::ref_ptr<FeatureCursor> cursor;

    for (const auto& subkey : keys)
    {
        FeatureList sublist;

        FeatureListCache::Record r;
        if (_lru->get(subkey, r))
        {
            sublist = r.value();
        }
        else
        {
            // the Gate prevents 2 threads that requesting the same TileKey
            // at the same time from the featuresource.
            ScopedGate<TileKey> gatelock(_keygate, subkey);

            // double-check the cache now that we are gate-locked:
            if (_lru->get(subkey, r))
            {
                sublist = r.value();
            }
            else
            {
                cursor = fs->createFeatureCursor(subkey, progress);
                if (cursor.valid())
                {
                    cursor->fill(
                        sublist, 
                        [](const Feature* f) { return f->getGeometry()->isLinear(); });

                    _lru->insert(subkey, sublist);
                }
            }
        }

        // Clone features onto the end of the output list.
        // We must always clone since osgEarth modifies the feature data
        for (auto& f : sublist)
            output.push_back(osg::clone(f.get(), osg::CopyOp::DEEP_COPY_ALL));
    }
}