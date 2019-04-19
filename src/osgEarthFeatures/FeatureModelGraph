/* --*-c++-*-- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#ifndef OSGEARTHFEATURES_FEATURE_MODEL_GRAPH_H
#define OSGEARTHFEATURES_FEATURE_MODEL_GRAPH_H 1

#include <osgEarthFeatures/Common>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthSymbology/Style>
#include <osgEarth/NodeUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/SceneGraphCallback>
#include <osgDB/Callbacks>
#include <osg/Node>
#include <set>

namespace osgEarth { namespace Features
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    class Session;

    /**
     * A scene graph that renders feature data.
     * This class will handle all the internals of selecting features, gridding feature
     * data if required, and sorting features based on style. Then for each cell and each
     * style, it will invoke the FeatureNodeFactory to create the actual data for each set.
     */
    class OSGEARTHFEATURES_EXPORT FeatureModelGraph : public osg::Group
    {
    public:
        /**
         * Constructs a new model graph.
         *
         * @param session
         *      Session under which to create elements in this graph
         * @param options
         *      Model source options
         * @param factory
         *      Node factory that will be invoked to compile feature data into nodes
         */
        FeatureModelGraph(
            Session*                         session,
            const FeatureModelSourceOptions& options,
            FeatureNodeFactory*              factory,
            SceneGraphCallbacks*             callbacks);

        /**
         * Constructs a new model graph.
         *
         * @param session
         *      Session under which to create elements in this graph
         * @param options
         *      Model source options
         * @param factory
         *      Node factory that will be invoked to compile feature data into nodes
         * @param callbacks
         *      List of callbacks to invoke as nodes are created and merged into the scene graph.
         */
        FeatureModelGraph(
            Session*                         session,
            const FeatureModelSourceOptions& options,
            FeatureNodeFactory*              factory,
            ModelSource*                     modelSource,
            SceneGraphCallbacks*             callbacks);

        /**
         * Loads and returns a subnode. Used internally for paging.
         */
        osg::Node* load(
            unsigned lod, unsigned tileX, unsigned tileY,
            const std::string& uri,
            const osgDB::Options* readOptions);

        /**
         * Set a scene graph callback host for this FMG
         */
        void setSceneGraphCallbacks(SceneGraphCallbacks* callbacks);

        /**
         * Style sheet associated with this feature graph.
         */
        StyleSheet* getStyles() { return _session->styles(); }

        /**
         * Sets the style sheet to use to render features 
         */
        void setStyles( StyleSheet* styles );

        /**
         * Session associated with this feature graph.
         */
        Session* getSession();

        /**
         * Mark the feature graph dirty and in need of regeneration 
         */
        void dirty();

        /**
         * Access to the features levels
         */
        const std::vector<const FeatureLevel*>& getLevels() const { return _lodmap; };

        //! Options passed in.
        const FeatureModelSourceOptions& options() const { return _options; }

    public: // osg::Node

        virtual void traverse(osg::NodeVisitor& nv);

    protected:

        virtual ~FeatureModelGraph();

        osg::Node* setupPaging();

        osg::Group* buildTile( 
            const FeatureLevel&   level, 
            const GeoExtent&      extent, 
            const TileKey*        key,
            const osgDB::Options* readOptions);

        osg::Group* build( 
            const Style&          baseStyle, 
            const Query&          baseQuery, 
            const GeoExtent&      extent, 
            FeatureIndexBuilder*  index,
            const osgDB::Options* readOptions,
            ProgressCallback*     progress);


    private:

        void ctor();
        
        osg::Group* createStyleGroup(
            const Style&          style, 
            const Query&          query, 
            FeatureIndexBuilder*  index,
            const osgDB::Options* readOptions,
            ProgressCallback*     progress);

        osg::Group* createStyleGroup(
            const Style&          style, 
            FeatureList&          workingSet, 
            const FilterContext&  contextPrototype,
            const osgDB::Options* readOptions);

        void buildStyleGroups(
            const StyleSelector*  selector,
            const Query&          baseQuery,
            FeatureIndexBuilder*  index,
            osg::Group*           parent,
            const osgDB::Options* readOptions,
            ProgressCallback*     progress);

        void queryAndSortIntoStyleGroups(
            const Query&            query,
            const StringExpression& styleExpr,
            FeatureIndexBuilder*    index,
            osg::Group*             parent,
            const osgDB::Options*   readOptions,
            ProgressCallback*       progress);

        osg::Group* getOrCreateStyleGroupFromFactory(
            const Style& style);
       
        osg::BoundingSphered getBoundInWorldCoords( 
            const GeoExtent& extent ) const;

        void buildSubTilePagedLODs(
            unsigned lod, unsigned tileX, unsigned tileY,
            osg::Group* parent,
            const osgDB::Options* readOptions);
        
        osg::Group* readTileFromCache(
            const std::string&    cacheKey,
            const osgDB::Options* readOptions);

        bool writeTileToCache(
            const std::string&    cacheKey,
            osg::Group*           tile,
            const osgDB::Options* readOptions);

        void redraw();

    private:
        FeatureModelSourceOptions        _options;
        osg::ref_ptr<FeatureNodeFactory> _factory;
        osg::ref_ptr<Session>            _session;
        std::set<std::string>            _blacklist;
        Threading::ReadWriteMutex        _blacklistMutex;
        GeoExtent                        _usableFeatureExtent;
        bool                             _featureExtentClamped;
        GeoExtent                        _usableMapExtent;
        osg::BoundingSphered             _fullWorldBound;
        bool                             _useTiledSource;
        osgEarth::Revision               _featureSourceRev, _modelSourceRev;
        bool                             _dirty;
        bool                             _pendingUpdate;
        std::vector<const FeatureLevel*> _lodmap;
        OpenThreads::ReentrantMutex      _redrawMutex;

        OpenThreads::Atomic _cacheReads;
        OpenThreads::Atomic _cacheHits;

        osg::ref_ptr<osgDB::FileLocationCallback> _defaultFileLocationCallback;

        osg::observer_ptr<ModelSource> _modelSource;

        osg::ref_ptr<FeatureSourceIndex> _featureIndex;

        osg::ref_ptr<SceneGraphCallbacks> _sgCallbacks;

        osg::ref_ptr<osgDB::ObjectCache> _nodeCachingImageCache;

        void runPreMergeOperations(osg::Node* node);
        void runPostMergeOperations(osg::Node* node);
        void applyRenderSymbology(const Style& style, osg::Node* node);
        bool createOrUpdateNode(FeatureCursor*, const Style&, FilterContext&, const osgDB::Options*, osg::ref_ptr<osg::Node>& output);
    };

} } // namespace osgEarth::Features

#endif // OSGEARTHFEATURES_FEATURE_MODEL_GRAPH_H
