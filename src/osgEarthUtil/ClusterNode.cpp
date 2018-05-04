#include <osgEarthUtil/ClusterNode>

#include <osgEarthUtil/kdbush.hpp>

typedef std::pair<int, int> TPoint;
typedef std::vector< std::size_t > TIds;

using namespace osgEarth::Util;

ClusterNode::ClusterNode(MapNode* mapNode, osg::Image* defaultImage) :
    _radius(50),
    _mapNode(mapNode),
    _nextLabel(0),
    _enabled(true),
    _dirty(true),
    _defaultImage(defaultImage)
{
    setNumChildrenRequiringUpdateTraversal(1);
    setCullingActive(false);

    _horizon = new Horizon();
}

void ClusterNode::addNode(osg::Node* node)
{
    _nodes.push_back(node);
    _dirty = true;
}

void ClusterNode::removeNode(osg::Node* node)
{
    osg::NodeList::iterator itr = std::find(_nodes.begin(), _nodes.end(), node);
    if (itr != _nodes.end())
    {
        _nodes.erase(itr);
    }
    _dirty = true;
}

unsigned int ClusterNode::getRadius() const
{
    return _radius;
}

void ClusterNode::setRadius(unsigned int radius)
{
    _radius = radius;
    _dirty = true;
}

bool ClusterNode::getEnabled() const
{
    return _enabled;
}

void ClusterNode::setEnabled(bool enabled)
{
    _enabled = enabled;
    _dirty = true;
}

ClusterNode::StyleClusterCallback* ClusterNode::getStyleCallback()
{
    return _styleCallback.get();
}

void
ClusterNode::setStyleCallback(ClusterNode::StyleClusterCallback* callback)
{
    _styleCallback = callback;
    _dirty = true;
}

ClusterNode::CanClusterCallback* ClusterNode::getCanClusterCallback()
{
    return _canClusterCallback.get();
}

void ClusterNode::setCanClusterCallback(ClusterNode::CanClusterCallback* callback)
{
    _canClusterCallback = callback;
    _dirty = true;
}


void ClusterNode::getClusters(osg::Camera* camera, ClusterList& out)
{
    _nextLabel = 0;

    osg::Viewport* viewport = camera->getViewport();
    if (!viewport)
    {
        return;
    }

    osg::Matrixd mvpw = camera->getViewMatrix() *
        camera->getProjectionMatrix() *
        camera->getViewport()->computeWindowMatrix();

    std::vector<TPoint> points;

    osg::NodeList validPlaces;

    for (unsigned int i = 0; i < _nodes.size(); i++)
    {
        osg::Vec3d world = _nodes[i]->getBound().center();

        if (!_horizon->isVisible(world))
        {
            continue;
        }

        osg::Vec3d screen = world * mvpw;

        if (screen.x() >= 0 && screen.x() <= viewport->width() &&
            screen.y() >= 0 && screen.y() <= viewport->height())
        {
            validPlaces.push_back(_nodes[i]);
            points.push_back(TPoint(screen.x(), screen.y()));
        }
    }

    if (validPlaces.size() == 0) return;

    kdbush::KDBush<TPoint> index(points);
    std::set< unsigned int > clustered;

    for (unsigned int i = 0; i < validPlaces.size(); i++)
    {
        TPoint &screen = points[i];
        //PlaceNode* place = validPlaces[i].get();
        osg::Node* node = validPlaces[i].get();

        // If this thing is already part of a cluster then just continue.
        if (clustered.find(i) != clustered.end())
        {
            continue;
        }

        osg::Vec3d world = node->getBound().center();

        // Get any matching indices that are part of this cluster.
        TIds indices;

        index.range(screen.first - _radius, screen.second - _radius, screen.first + _radius, screen.second + _radius, indices);
        //index.within(screen.first, screen.second, _radius, [&indices](const auto id) { indices.push_back(id); });

        // Create a new cluster.
        Cluster cluster;

        unsigned int actualCount = 0;

        // Add all of the points to the cluster.
        for (unsigned int j = 0; j < indices.size(); j++)
        {
            if (clustered.find(indices[j]) == clustered.end())
            {
                if (_canClusterCallback.valid())
                {
                    bool canCluster = (*_canClusterCallback)(node, validPlaces[indices[j]].get());
                    if (!canCluster) {
                        continue;
                    }
                }
                cluster.nodes.push_back(validPlaces[indices[j]]);
                actualCount++;
                clustered.insert(indices[j]);
            }
        }

        std::stringstream buf;
        buf << actualCount << std::endl;

        PlaceNode* marker = getOrCreateLabel();
        GeoPoint markerPos;
        markerPos.fromWorld(_mapNode->getMapSRS(), world);
        marker->setPosition(markerPos);
        marker->setText(buf.str());

        cluster.marker = marker;
        out.push_back(cluster);

        clustered.insert(i);
    }
}

void ClusterNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

        // If we aren't enabled just traverse all the placenodes.
        if (!_enabled)
        {
            for (osg::NodeList::iterator itr = _nodes.begin(); itr != _nodes.end(); ++itr)
            {
                itr->get()->accept(nv);
            }            
        }
        else
        {
            const osg::Matrixd &currentViewMatrix = cv->getCurrentCamera()->getViewMatrix();
            if (_lastViewMatrix != currentViewMatrix || _dirty)
            {
                osg::Vec3d eye, center, up;
                cv->getCurrentCamera()->getViewMatrixAsLookAt(eye, center, up);

                _horizon->setEye(eye);

                _clusters.clear();
                getClusters(cv->getCurrentCamera(), _clusters);

                // Style the clusters if need be
                if (_styleCallback)
                {
                    for (ClusterList::iterator itr = _clusters.begin(); itr != _clusters.end(); ++itr)
                    {
                        Cluster& cluster = *itr;
                        (*_styleCallback)(cluster);
                    }
                }
            }

            for (ClusterList::iterator itr = _clusters.begin(); itr != _clusters.end(); ++itr)
            {
                Cluster& cluster = *itr;
                
                // If we have more than 1 place, traverse the representative marker
                if (cluster.nodes.size() > 1)
                {
                    itr->marker->accept(nv);
                }
                else
                {
                    // Otherwise just traverse the first node
                    cluster.nodes[0]->accept(nv);
                }
            }

            _dirty = false;
            _lastViewMatrix = currentViewMatrix;
        }
    }
    else
    {
        // Other visitors just traverse all the placenodes.
        for (osg::NodeList::iterator itr = _nodes.begin(); itr != _nodes.end(); ++itr)
        {
            itr->get()->accept(nv);
        }
    }
}

PlaceNode* ClusterNode::getOrCreateLabel()
{
    PlaceNode* node = 0;
    if (_labelPool.size() <= _nextLabel)
    {
        // set up a style to use for placemarks:
        Style placeStyle;
        placeStyle.getOrCreate<TextSymbol>()->declutter() = false;
        node = new PlaceNode(_mapNode.get(), GeoPoint(SpatialReference::create("wgs84"), 0, 0, 0), _defaultImage.get(), "", placeStyle);
        node->setDynamic(true);
        _labelPool.push_back(node);
    }
    else
    {
        node = _labelPool[_nextLabel].get();
    }

    ++_nextLabel;

    return node;


}