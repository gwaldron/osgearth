#ifndef OSGEARTHUTIL_CLUSTERNODE_H
#define OSGEARTHUTIL_CLUSTERNODE_H

#include <osgEarthUtil/Common>
#include <osg/Node>

#include <osgEarthAnnotation/PlaceNode>

namespace osgEarth {
    namespace Util
    {
        using namespace osgEarth;
        using namespace osgEarth::Annotation;

        typedef std::vector< osg::ref_ptr< PlaceNode > > PlaceNodeList;

        /**
         * ClusterNode clusters overlapping nodes together into PlaceNodes on the screen to avoid visual clutter and increase performance.
         */
        class OSGEARTHUTIL_EXPORT ClusterNode : public osg::Node
        {
        public:
            struct Cluster
            {
                osg::ref_ptr< PlaceNode > marker;
                osg::NodeList nodes;
            };

            typedef std::vector< Cluster > ClusterList;

            class StyleClusterCallback : public osg::Referenced
            {
            public:
                virtual void operator()(Cluster& cluster) {}
            };

            class CanClusterCallback : public osg::Referenced
            {
            public:
                virtual bool operator()(osg::Node* a, osg::Node* b) { return true; }
            };


        public:
            ClusterNode(MapNode* mapNode = 0, osg::Image* defaultImage = 0);

            MapNode* getMapNode() const;
            void setMapNode(MapNode* mapNode);

            void addNode(osg::Node* node);
            void removeNode(osg::Node* node);
            void clear();

            unsigned int getRadius() const;
            void setRadius(unsigned int radius);

            bool getEnabled() const;
            void setEnabled(bool enabled);

            StyleClusterCallback* getStyleCallback();
            void setStyleCallback(StyleClusterCallback* callback);

            CanClusterCallback* getCanClusterCallback();
            void setCanClusterCallback(CanClusterCallback* callback);

            virtual void traverse(osg::NodeVisitor& nv);

        protected:

            PlaceNode* getOrCreateLabel();

            void getClusters(osgUtil::CullVisitor* cv, ClusterList& out);
            void buildIndex();

            osg::NodeList _nodes;

            unsigned int _radius;

            osg::ref_ptr< osg::Image > _defaultImage;

            PlaceNodeList _labelPool;
            unsigned int _nextLabel;
            osg::observer_ptr< MapNode > _mapNode;

            osg::ref_ptr< StyleClusterCallback > _styleCallback;
            osg::ref_ptr< CanClusterCallback > _canClusterCallback;

            osg::ref_ptr< Horizon > _horizon;

            osg::Matrixd _lastViewMatrix;

            ClusterList _clusters;

            osg::NodeList _clusterIndex;
            bool _dirtyIndex;

            bool _dirty;

            bool _enabled;
        };
    }
}

#endif