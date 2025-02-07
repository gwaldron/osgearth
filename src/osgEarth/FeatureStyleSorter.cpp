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
#include <osgEarth/FeatureStyleSorter>
#include <osgEarth/Registry>
#include <osgEarth/FeatureSource>
#include <osgEarth/StyleSheet>
#include <osgText/String>
#include <osgEarth/BuildConfig>

using namespace osgEarth;

void
FeatureStyleSorter::sort_usingEmbeddedStyles(
    const TileKey& key,
    const Distance& buffer,
    const FeatureFilterChain& filters,
    Session* session,
    FeatureStyleSorter::Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    // Each feature has its own embedded style data, so use that:
    FilterContext context;

    Query query;
    query.tileKey() = key;
    query.buffer() = buffer;

    osg::ref_ptr<FeatureCursor> cursor = session->getFeatureSource()->createFeatureCursor(query, filters, &context, progress);

    while (cursor.valid() && cursor->hasMore())
    {
        auto feature = cursor->nextFeature();
        if (feature)
        {
            FeatureList data;
            data.push_back(feature);
            processFeaturesForStyle(feature->style().get(), data, progress);
        }
    }
}

void
FeatureStyleSorter::sort_usingSelectors(
    const TileKey& key,
    const Distance& buffer,
    const FeatureFilterChain& filters,
    Session* session,
    FeatureStyleSorter::Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    FeatureSource* features = session->getFeatureSource();

    Query defaultQuery;
    defaultQuery.tileKey() = key;

    for (auto& iter : session->styles()->getSelectors())
    {
        const StyleSelector& sel = iter.second;
        if (sel.styleExpression().isSet())
        {
            const FeatureProfile* featureProfile = features->getFeatureProfile();

            // establish the working bounds and a context:
            FilterContext context(session, featureProfile);
            StringExpression styleExprCopy(sel.styleExpression().get());

            FeatureList features;
            getFeatures(session, defaultQuery, buffer, key.getExtent(), filters, features, progress);
            if (!features.empty())
            {
                std::unordered_map<std::string, std::pair<Style, int>> literal_styles;

                // keep ordered.
                std::map<int, std::pair<const Style*, FeatureList>> style_buckets;

                for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                {
                    Feature* feature = itr->get();

                    const std::string& delimitedStyleStrings = feature->eval(styleExprCopy, &context);
                    if (!delimitedStyleStrings.empty() && delimitedStyleStrings != "null")
                    {
                        auto styleStrings = StringTokenizer()
                            .delim(",")
                            .standardQuotes()
                            .tokenize(delimitedStyleStrings);

                        for (auto& styleString : styleStrings)
                        {
                            // resolve the style:
                            const Style* resolved_style = nullptr;
                            int resolved_index = 0;

                            // if the style string begins with an open bracket, it's an inline style definition.
                            if (styleString.length() > 0 && styleString[0] == '{')
                            {
                                Config conf("style", styleString);
                                conf.setReferrer(sel.styleExpression().get().uriContext().referrer());
                                conf.set("type", "text/css");
                                auto& literal_style_and_index = literal_styles[conf.toJSON()];
                                if (literal_style_and_index.first.empty())
                                {
                                    literal_style_and_index.first = Style(conf);
                                    // literal styles always come AFTER sheet styles
                                    literal_style_and_index.second = literal_styles.size() + session->styles()->getStyles().size();
                                }
                                resolved_style = &literal_style_and_index.first;
                                resolved_index = literal_style_and_index.second;
                            }

                            // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
                            // style in this case: for style expressions, the user must be explicit about
                            // default styling; this is because there is no other way to exclude unwanted
                            // features.
                            else
                            {
                                auto style_and_index = session->styles()->getStyleAndIndex(styleString);

                                //const Style* selected_style = session->styles()->getStyle(styleString, false);
                                if (style_and_index.first)
                                {
                                    resolved_style = style_and_index.first;
                                    resolved_index = style_and_index.second;
                                }
                            }

                            if (resolved_style)
                            {
                                auto& bucket = style_buckets[resolved_index];
                                bucket.first = resolved_style;
                                bucket.second.emplace_back(feature);
                            }
                        }
                    }
                }

                // in order:
                for (auto& iter : style_buckets)
                {
                    const Style* style = iter.second.first;
                    FeatureList& list = iter.second.second;
                    processFeaturesForStyle(*style, list, progress);
                }
            }
        }
        else
        {
            const Style* style = session->styles()->getStyle(sel.getSelectedStyleName());
            Query query = sel.query().get();
            query.tileKey() = key;

            // Get the features
            FeatureList features;
            getFeatures(session, query, buffer, key.getExtent(), filters, features, progress);

            processFeaturesForStyle(*style, features, progress);
        }
    }
}

void
FeatureStyleSorter::sort_usingOneStyle(
    const Style& style,
    const TileKey& key,
    const Distance& buffer,
    const FeatureFilterChain& filters,
    Session* session,
    Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    Query defaultQuery;
    defaultQuery.tileKey() = key;

    FeatureList features;
    getFeatures(session, defaultQuery, buffer, key.getExtent(), filters, features, progress);

    processFeaturesForStyle(style, features, progress);
}

void
FeatureStyleSorter::sort(
    const TileKey& key,
    const Distance& buffer,
    Session* session,
    const FeatureFilterChain& filters,
    Function processFeaturesForStyle,
    ProgressCallback* progress) const
{
    OE_SOFT_ASSERT_AND_RETURN(session, void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource(), void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource()->getFeatureProfile(), void());

    if (session->getFeatureSource()->hasEmbeddedStyles())
    {
        sort_usingEmbeddedStyles(
            key,
            buffer,
            filters,
            session,
            processFeaturesForStyle,
            progress);

    }
    else if (session->styles())
    {
        if (session->styles()->getSelectors().size() > 0)
        {
            sort_usingSelectors(
                key,
                buffer,
                filters,
                session,
                processFeaturesForStyle,
                progress);

        }
        else
        {
            sort_usingOneStyle(
                *session->styles()->getDefaultStyle(),
                key,
                buffer,
                filters,
                session,
                processFeaturesForStyle,
                progress);
        }
    }
    else
    {
        sort_usingOneStyle(
            Style(), // empty style
            key,
            buffer,
            filters,
            session,
            processFeaturesForStyle,
            progress);
    }
}

void
FeatureStyleSorter::getFeatures(
    Session* session,
    const Query& query,
    const Distance& buffer,
    const GeoExtent& workingExtent,
    const FeatureFilterChain& filters,
    FeatureList& features,
    ProgressCallback* progress) const
{
    OE_SOFT_ASSERT_AND_RETURN(session != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource() != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(session->getFeatureSource()->getFeatureProfile() != nullptr, void());
    OE_SOFT_ASSERT_AND_RETURN(workingExtent.isValid(), void());

    // first we need the overall extent of the layer:
    const GeoExtent& featuresExtent = session->getFeatureSource()->getFeatureProfile()->getExtent();

    // convert them both to WGS84, intersect the extents, and convert back.
    GeoExtent featuresExtentWGS84 = featuresExtent.transform(featuresExtent.getSRS()->getGeographicSRS());
    GeoExtent workingExtentWGS84 = workingExtent.transform(featuresExtent.getSRS()->getGeographicSRS());
    GeoExtent queryExtentWGS84 = featuresExtentWGS84.intersectionSameSRS(workingExtentWGS84);
    if (queryExtentWGS84.isValid())
    {
        GeoExtent queryExtent = queryExtentWGS84.transform(featuresExtent.getSRS());

        // incorporate the image extent into the feature query for this style:
        Query localQuery = query;
        localQuery.bounds() =
            query.bounds().isSet() ? unionOf(query.bounds().get(), queryExtent.bounds()) :
            queryExtent.bounds();

        FilterContext context(session, session->getFeatureSource()->getFeatureProfile(), queryExtent);

        // now copy the resulting feature set into a list, converting the data
        // types along the way if a geometry override is in place:
        while (features.empty())
        {
            if (progress && progress->isCanceled())
                break;

            osg::ref_ptr<FeatureCursor> cursor;
            
            if (localQuery.tileKey().isSet())
            {
                localQuery.buffer() = buffer;
            }

            cursor = session->getFeatureSource()->createFeatureCursor(localQuery, filters, &context, progress);

            while (cursor.valid() && cursor->hasMore())
            {
                Feature* feature = cursor->nextFeature();
                if (feature->getGeometry())
                {
                    features.push_back(feature);
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
