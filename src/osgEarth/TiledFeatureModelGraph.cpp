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
#include <osgEarth/TiledFeatureModelGraph>
#include <osgEarth/GeometryCompiler>
#include <osgEarth/FeatureModelSource>
#include <osgEarth/NetworkMonitor>

using namespace osgEarth;

TiledFeatureModelGraph::TiledFeatureModelGraph(const osgEarth::Map* map,
                                               FeatureSource* features,
                                               StyleSheet* styleSheet,
                                               Session* session) :
    SimplePager(map, features->getFeatureProfile()->getTilingProfile()),
    _features(features),
    _styleSheet(styleSheet),
    _session(session)
{
    setMinLevel(features->getFeatureProfile()->getFirstLevel());
    setMaxLevel(features->getFeatureProfile()->getMaxLevel());

    _session->setResourceCache(new ResourceCache());


    FeatureSourceIndexOptions indexOptions;
    indexOptions.enabled() = true;

    _featureIndex = new FeatureSourceIndex(
        features,
        Registry::objectIndex(),
        indexOptions);
}

void
TiledFeatureModelGraph::setFilterChain(FeatureFilterChain* chain)
{
    _filterChain = chain;
}

void
TiledFeatureModelGraph::setOwnerName(const std::string& value)
{
    _ownerName = value;
}


FeatureCursor*
TiledFeatureModelGraph::createCursor(FeatureSource* fs, FilterContext& cx, const Query& query, ProgressCallback* progress) const
{
    NetworkMonitor::ScopedRequestLayer layerRequest(_ownerName);
    FeatureCursor* cursor = fs->createFeatureCursor(query, progress);
    if (cursor && _filterChain.valid())
    {
        cursor = new FilteredFeatureCursor(cursor, _filterChain.get(), &cx);
    }
    return cursor;
}

osg::ref_ptr<osg::Node>
TiledFeatureModelGraph::createNode(const TileKey& key, ProgressCallback* progress)
{
    if (progress && progress->isCanceled())
        return nullptr;

    NetworkMonitor::ScopedRequestLayer layerRequest(_ownerName);
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

    GeometryCompilerOptions options;
    options.instancing() = true;
    //options.mergeGeometry() = true;
    GeometryCompiler gc(options);

    GeomFeatureNodeFactory factory(options);

    if (progress && progress->isCanceled())
        return nullptr;

    osg::ref_ptr< FeatureCursor > cursor = _features->createFeatureCursor(
        query,
        _filterChain.get(),
        &fc,
        progress);

    osg::ref_ptr<osg::Node> node = new osg::Group;
    if (cursor)
    {
        if (progress && progress->isCanceled())
            return nullptr;

        FeatureList features;
        cursor->fill(features);

        if (_styleSheet->getSelectors().size() > 0)
        {
            osg::Group* group = new osg::Group;

            typedef std::map< std::string, FeatureList > StyleToFeaturesMap;
            StyleToFeaturesMap styleToFeatures;

            for (StyleSelectors::const_iterator i = _styleSheet->getSelectors().begin();
                i != _styleSheet->getSelectors().end();
                ++i)
            {
                // pull the selected style...
                const StyleSelector& sel = i->second;

                // if the selector uses an expression to select the style name, then we must perform the
                // query and then SORT the features into style groups.
                if (sel.styleExpression().isSet())
                {
                    // establish the working bounds and a context:
                    StringExpression styleExprCopy(sel.styleExpression().get());
                    for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
                    {
                        Feature* feature = itr->get();

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
            }

            for (StyleToFeaturesMap::iterator itr = styleToFeatures.begin(); itr != styleToFeatures.end(); ++itr)
            {
                const Style* style = _styleSheet->getStyle(itr->first);
                if (style)
                {
                    osg::Group* styleGroup = factory.getOrCreateStyleGroup(*style, _session.get());
                    osg::ref_ptr< osg::Node>  styleNode;
                    osg::ref_ptr< FeatureListCursor> cursor = new FeatureListCursor(itr->second);
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
            node = group;
        }
        else if (_styleSheet->getDefaultStyle())
        {
            osg::ref_ptr< FeatureListCursor> cursor = new FeatureListCursor(features);
            osg::ref_ptr< osg::Group > group = new osg::Group;
            osg::ref_ptr< osg::Group > styleGroup = factory.getOrCreateStyleGroup(*_styleSheet->getDefaultStyle(), _session.get());
            osg::ref_ptr< osg::Node>  styleNode;
            factory.createOrUpdateNode(cursor.get(), *_styleSheet->getDefaultStyle(), fc, styleNode, query);
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