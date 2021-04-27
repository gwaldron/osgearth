/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarth/FeatureImageLayer>
#include <osgEarth/Session>
#include <osgEarth/FeatureCursor>
#include <osgEarth/TransformFilter>
#include <osgEarth/BufferFilter>
#include <osgEarth/ResampleFilter>
#include <osgEarth/StyleSheet>
#include <osgEarth/Registry>
#include <osgEarth/Progress>
#include <osgEarth/LandCover>
#include <osgEarth/Metrics>
#include <osgEarth/SDF>
#include <osgEarth/JsonUtils>

using namespace osgEarth;

#define LC "[FeatureImageLayer] " << getName() << ": "


REGISTER_OSGEARTH_LAYER(featureimage, FeatureImageLayer);
REGISTER_OSGEARTH_LAYER(feature_image, FeatureImageLayer);

//........................................................................

Config
FeatureImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    featureSource().set(conf, "features");
    styleSheet().set(conf, "styles");
    conf.set("gamma", gamma());
    conf.set("sdf", sdf());
    conf.set("sdf_invert", sdf_invert());

    if (filters().empty() == false)
    {
        Config temp;
        for(unsigned i=0; i<filters().size(); ++i)
            temp.add( filters()[i].getConfig() );
        conf.set( "filters", temp );
    }

    return conf;
}

void
FeatureImageLayer::Options::fromConfig(const Config& conf)
{
    gamma().setDefault(1.3);
    sdf().setDefault(false);
    sdf_invert().setDefault(false);

    featureSource().get(conf, "features");
    styleSheet().get(conf, "styles");
    conf.get("gamma", gamma());
    conf.get("sdf", sdf());
    conf.get("sdf_invert", sdf_invert());

    const Config& filtersConf = conf.child("filters");
    for(ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back( ConfigOptions(*i) );
}

//........................................................................

void
FeatureImageLayer::init()
{
    ImageLayer::init();

    // Default profile (WGS84) if not set
    if (!getProfile())
    {
        setProfile(Profile::create("global-geodetic"));
    }
}

Status
FeatureImageLayer::openImplementation()
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

    establishProfile();

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());

    return Status::NoError;
}

void
FeatureImageLayer::establishProfile()
{
    if (getProfile() == nullptr && getFeatureSource() != nullptr)
    {
        const FeatureProfile* fp = getFeatureSource()->getFeatureProfile();

        if (fp->getTilingProfile())
        {
            setProfile(fp->getTilingProfile());
        }
        else if (fp->getSRS())
        {
            setProfile(Profile::create(fp->getSRS()));
        }
    }
}

void
FeatureImageLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    options().featureSource().addedToMap(map);
    options().styleSheet().addedToMap(map);

    if (getFeatureSource())
    {
        establishProfile();
        _session = new Session(map, getStyleSheet(), getFeatureSource(), getReadOptions());
        updateSession();
    }
}

void
FeatureImageLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);
    options().styleSheet().removedFromMap(map);

    ImageLayer::removedFromMap(map);
}

void
FeatureImageLayer::setFeatureSource(FeatureSource* fs)
{
    if (getFeatureSource() != fs)
    {
        options().featureSource().setLayer(fs);
        _featureProfile = 0L;

        if (fs)
        {
            if (fs->getStatus().isError())
            {
                setStatus(fs->getStatus());
            }
            else
            {
                // with a new feature source, we need to re-establish
                // the data extents and open a new session.
                updateSession();
            }
        }
    }
}

void
FeatureImageLayer::setStyleSheet(StyleSheet* value)
{
    if (getStyleSheet() != value)
    {
        options().styleSheet().setLayer(value);
        if (_session.valid())
        {
            _session->setStyles(getStyleSheet());
        }
    }
}

void
FeatureImageLayer::updateSession()
{
    if (_session.valid() && getFeatureSource())
    {
        const FeatureProfile* fp = getFeatureSource()->getFeatureProfile();

        dataExtents().clear();

        if (fp)
        {
            // recalculate the data extents based on the feature source.
            if (fp->getTilingProfile() != NULL)
            {
                // Use specified profile's GeoExtent
                unsigned maxLevel = fp->getMaxLevel();
                if (options().maxDataLevel().isSet())
                    maxLevel = osg::maximum(maxLevel, options().maxDataLevel().get());
                dataExtents().push_back(DataExtent(fp->getTilingProfile()->getExtent(), fp->getFirstLevel(), maxLevel));
            }
            else if (fp->getExtent().isValid() == true)
            {
                // Use FeatureProfile's GeoExtent
                dataExtents().push_back(DataExtent(fp->getExtent()));
            }

            // warn the user if the feature data is tiled and the
            // layer profile doesn't match the feature source profile
            if (fp->isTiled() &&
                fp->getTilingProfile()->isHorizEquivalentTo(getProfile()) == false)
            {
                OE_WARN << LC << "Layer profile doesn't match feature tiling profile - data may not render properly" << std::endl;
                OE_WARN << LC << "(Feature tiling profile = " << fp->getTilingProfile()->toString() << ")" << std::endl;
            }
        }

        _session->setFeatureSource(getFeatureSource());
        _session->setStyles(getStyleSheet());
    }
}

GeoImage
FeatureImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return GeoImage::INVALID;
    }

    if (!getFeatureSource())
    {
        setStatus(Status::ServiceUnavailable, "No feature source");
        return GeoImage::INVALID;
    }

    const FeatureProfile* featureProfile = getFeatureSource()->getFeatureProfile();
    if (!featureProfile)
    {
        setStatus(Status::ConfigurationError, "Feature profile is missing");
        return GeoImage::INVALID;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status::ConfigurationError, "Feature profile has no SRS");
        return GeoImage::INVALID;
    }

    if (!_session.valid())
    {
        setStatus(Status::AssertionFailure, "_session is NULL - call support");
        return GeoImage::INVALID;
    }

    osg::Image* result = render(key, _session.get(), getStyleSheet(), progress);
    if (result)
    {
        return GeoImage(result, key.getExtent());
    }
    else
    {
        return GeoImage::INVALID;
    }
}

//........................................................................

osg::Image*
FeatureImageLayer::render(
    const TileKey& key,
    Session* session,
    const StyleSheet* styles,
    ProgressCallback* progress
) const
{
    FeatureRasterizer featureRasterizer(getTileSize(), getTileSize(), key.getExtent());

    OE_PROFILING_ZONE;

    Query defaultQuery;
    defaultQuery.tileKey() = key;

    FeatureSource* features = session->getFeatureSource();
    if (!features)
        return nullptr;

    // figure out if and how to style the geometry.
    if (features->hasEmbeddedStyles())
    {
        // Each feature has its own embedded style data, so use that:
        FilterContext context;

        osg::ref_ptr<FeatureCursor> cursor = features->createFeatureCursor(
            defaultQuery,
            _filterChain.get(),
            &context,
            progress);

        while (cursor.valid() && cursor->hasMore())
        {
            osg::ref_ptr< Feature > feature = cursor->nextFeature();
            if (feature)
            {
                FeatureList list;
                list.push_back(feature);

                featureRasterizer.render(session, *feature->style(), getFeatureSource()->getFeatureProfile(), list);
            }
        }
    }
    else if (styles)
    {
        if (styles->getSelectors().size() > 0)
        {
            for (StyleSelectors::const_iterator i = styles->getSelectors().begin();
                i != styles->getSelectors().end();
                ++i)
            {
                const StyleSelector& sel = i->second;

                if (sel.styleExpression().isSet())
                {
                    const FeatureProfile* featureProfile = features->getFeatureProfile();

                    // establish the working bounds and a context:
                    FilterContext context(session, featureProfile);
                    StringExpression styleExprCopy(sel.styleExpression().get());

                    FeatureList features;
                    getFeatures(session, defaultQuery, key.getExtent(), features, progress);
                    if (!features.empty())
                    {
                        std::unordered_map<std::string, Style> literal_styles;
                        std::map<const Style*, FeatureList> style_buckets;

                        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                        {
                            Feature* feature = itr->get();

                            const std::string& styleString = feature->eval(styleExprCopy, &context);
                            if (!styleString.empty() && styleString != "null")
                            {
                                // resolve the style:
                                //Style combinedStyle;
                                const Style* resolved_style = nullptr;

                                // if the style string begins with an open bracket, it's an inline style definition.
                                if (styleString.length() > 0 && styleString[0] == '{')
                                {
                                    Config conf("style", styleString);
                                    conf.setReferrer(sel.styleExpression().get().uriContext().referrer());
                                    conf.set("type", "text/css");
                                    Style& literal_style = literal_styles[conf.toJSON()];
                                    if (literal_style.empty())
                                        literal_style = Style(conf);
                                    resolved_style = &literal_style;
                                }

                                // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
                                // style in this case: for style expressions, the user must be explicity about
                                // default styling; this is because there is no other way to exclude unwanted
                                // features.
                                else
                                {
                                    const Style* selected_style = session->styles()->getStyle(styleString, false);
                                    if (selected_style)
                                        resolved_style = selected_style;
                                }

                                if (resolved_style)
                                {
                                    style_buckets[resolved_style].push_back(feature);
                                }
                            }
                        }

                        for (auto& iter : style_buckets)
                        {
                            const Style* style = iter.first;
                            FeatureList& list = iter.second;

                            featureRasterizer.render(session, *style, getFeatureSource()->getFeatureProfile(), list);
                        }
                    }
                }
                else
                {
                    const Style* style = styles->getStyle(sel.getSelectedStyleName());
                    Query query = sel.query().get();
                    query.tileKey() = key;


                    // Get the features
                    FeatureList features;
                    getFeatures(session, query, key.getExtent(), features, progress);

                    // Render the features
                    featureRasterizer.render(session, *style, getFeatureSource()->getFeatureProfile(), features);
                }
            }
        }
        else
        {
            const Style* style = styles->getDefaultStyle();

            // Get the features
            FeatureList features;
            getFeatures(session, defaultQuery, key.getExtent(), features, progress);
            // Render the features
            featureRasterizer.render(session, *style, getFeatureSource()->getFeatureProfile(), features);
        }
    }
    else
    {
        FeatureList features;
        getFeatures(session, defaultQuery, key.getExtent(), features, progress);

        // Render the features
        featureRasterizer.render(session, Style(), getFeatureSource()->getFeatureProfile(), features);
    }

    return featureRasterizer.finalize();
}

void
FeatureImageLayer::getFeatures(
    Session* session,
    const Query& query,
    const GeoExtent& imageExtent,
    FeatureList& features,
    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    // first we need the overall extent of the layer:
    const GeoExtent& featuresExtent = session->getFeatureSource()->getFeatureProfile()->getExtent();

    // convert them both to WGS84, intersect the extents, and convert back.
    GeoExtent featuresExtentWGS84 = featuresExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent imageExtentWGS84 = imageExtent.transform( featuresExtent.getSRS()->getGeographicSRS() );
    GeoExtent queryExtentWGS84 = featuresExtentWGS84.intersectionSameSRS( imageExtentWGS84 );
    if ( queryExtentWGS84.isValid() )
    {
        GeoExtent queryExtent = queryExtentWGS84.transform( featuresExtent.getSRS() );

        // incorporate the image extent into the feature query for this style:
        Query localQuery = query;
        localQuery.bounds() =
            query.bounds().isSet() ? query.bounds()->unionWith( queryExtent.bounds() ) :
            queryExtent.bounds();

        FilterContext context(session, session->getFeatureSource()->getFeatureProfile(), queryExtent);

        // now copy the resulting feature set into a list, converting the data
        // types along the way if a geometry override is in place:
        while (features.empty())
        {
            if (progress && progress->isCanceled())
                break;

            // query the feature source:
            //osg::ref_ptr<FeatureCursor> cursor = createCursor(session->getFeatureSource(), _filterChain.get(), context, localQuery, progress);

            osg::ref_ptr<FeatureCursor> cursor = session->getFeatureSource()->createFeatureCursor(
                localQuery,
                _filterChain.get(),
                &context,
                progress);

            while( cursor.valid() && cursor->hasMore() )
            {
                Feature* feature = cursor->nextFeature();
                if (feature->getGeometry())
                {
                    features.push_back( feature );
                }
            }

            // If we didn't get any features and we have a tilekey set, try falling back.
            if (features.empty() &&
                localQuery.tileKey().isSet() &&
                localQuery.tileKey()->valid())
            {
                localQuery.tileKey() = localQuery.tileKey().get().createParentKey();
                if (!localQuery.tileKey()->valid())
                {
                    // We fell back all the way to lod 0 and got nothing, so bail.
                    break;
                }
            }
            else
            {
                // Just bail, we didn't get any features and aren't using tilekeys
                break;
            }
        }
    }
}

