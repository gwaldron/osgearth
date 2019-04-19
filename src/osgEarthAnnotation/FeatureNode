/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#ifndef OSGEARTH_ANNO_FEATURE_NODE_H
#define OSGEARTH_ANNO_FEATURE_NODE_H 1

#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarth/MapNode>
#include <osgEarth/GeometryClamper>
#include <osgEarthSymbology/StyleSheet>
#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/GeometryCompiler>
#include <osg/Polytope>

namespace osgEarth { namespace Annotation
{
    using namespace osgEarth;
    using namespace osgEarth::Features;
    using namespace osgEarth::Symbology;

    /**
     * Renders a single feature. Since no feature profile is provided,
     * the feature must contain geometry that is in the same SRS as the map.
     * The feature must also include the Style you wish to use.
     */
    class OSGEARTHANNO_EXPORT FeatureNode : public AnnotationNode
    {
    public:
        META_AnnotationNode(osgEarthAnnotation, FeatureNode);

        /**
         * Construct a new FeatureNode from a single Feature.
         */
        FeatureNode(
            Feature* feature,
            const Style& style = Style(),
            const GeometryCompilerOptions& options = GeometryCompilerOptions(),
            StyleSheet* styleSheet = 0);

        /**
         * Constuct a new FeatureNode from a list of features.
         */
        FeatureNode(
            const FeatureList& features,
            const Style& style = Style(),
            const GeometryCompilerOptions& options = GeometryCompilerOptions(),
            StyleSheet* styleSheet = 0);

         /**
         * Gets the list of features
         */
        FeatureList& getFeatures() { return _features; }

        /**
         * Utility that lets you work on this FeatureNode as a single Feature instead of a list
         */
        Feature* getFeature();

        /**
         * Sets the contents of this FeatureNode to a single feature.
         */
        void setFeature(Feature* feature);

        /**
         * Gets the StyleSheet for the session.
         */
        StyleSheet* getStyleSheet() const;

        /**
         * Sets the StyleSheet for the session.
         */
        void setStyleSheet(StyleSheet* styleSheet);

        /**
         * Gets the FeatureIndexBuilder
         */
        FeatureIndexBuilder* getIndex();

        /**
        * Sets the FeatureIndexBuilder
        */
        void setIndex(FeatureIndexBuilder* index);

        /**
         * Call init to force a rebuild of this FeatureNode.  If you modify the features in the features list or add/remove features
         * call this function to rebuild the node.
         */
        void dirty();

        //! @deprecated backwards compatiblity (remove after 2.10)
        void init() { dirty(); }

    public: // AnnotationNode

        /**
         * Gets the Style for this FeatureNode.
         */
        virtual const Style& getStyle() const;

        /**
         * Sets the style for this FeatureNode.
         * Note:  Do NOT use embedded feature styles if you need to change the style of the FeatureNode at runtime using this method.
         *        You will need to set the style of the features themselves and call init on this FeatureNode if you use embedded styles.
         */
        virtual void setStyle(const Style& style);

    public: // osg::Node

        virtual void traverse(osg::NodeVisitor&);

    public: // MapNodeObserver

        virtual void setMapNode( MapNode* mapNode );

    public:

        FeatureNode(const Config& conf, const osgDB::Options* options);
        virtual Config getConfig() const;

    protected:

        virtual ~FeatureNode() { }

        FeatureList                  _features;
        GeometryCompilerOptions      _options;
        osg::Group*                  _attachPoint;
        osg::Polytope                _featurePolytope;
        Style                        _style;
        bool                         _needsRebuild;
        GeoExtent                    _extent;

        typedef TerrainCallbackAdapter<FeatureNode> ClampCallback;
        osg::ref_ptr<ClampCallback> _clampCallback;
        bool _clampDirty;
        GeometryClamper::LocalData _clamperData;

        osg::ref_ptr< osg::Node >    _compiled;

        osg::ref_ptr< StyleSheet >   _styleSheet;

        FeatureIndexBuilder* _index;

        FeatureNode() { }
        FeatureNode(const FeatureNode& rhs, const osg::CopyOp& op) { }

        void clamp(osg::Node* graph, const Terrain* terrain);

        void build();

        //void construct();

    public:

        void onTileAdded(
            const TileKey&          key,
            osg::Node*              graph,
            TerrainCallbackContext& context);
    };

} } // namespace osgEarth::Annotation

#endif // OSGEARTH_ANNO_FEATURE_NODE_H
