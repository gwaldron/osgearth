#include "ClusterNode"

ClusterNode::ClusterNode():
    _radius(50)
{
    setNumChildrenRequiringUpdateTraversal(1);
    setCullingActive(false);
}

void ClusterNode::addNode(PlaceNode* node)
{
    _placeNodes.push_back(node);
}

void ClusterNode::removeNode(PlaceNode* node)
{
    PlaceNodeList::iterator itr = std::find(_placeNodes.begin(), _placeNodes.end(), node);
    if (itr != _placeNodes.end())
    {
        _placeNodes.erase(itr);
    }
}

unsigned int ClusterNode::getRadius() const
{
    return _radius;
}

void ClusterNode::setRadius(unsigned int radius)
{
    _radius = radius;
}

void ClusterNode::getPlaces(osg::Camera* camera, PlaceNodeList& out)
{
    osg::Viewport* viewport = camera->getViewport();
    if (!viewport)
    {
        return;
    }

    osg::Matrixd mvpw = camera->getViewMatrix() *
        camera->getProjectionMatrix() *
        camera->getViewport()->computeWindowMatrix();

    std::vector<TPoint> points;

    std::vector< osg::ref_ptr< PlaceNode > > validPlaces;

    for (unsigned int i = 0; i < _placeNodes.size(); i++)
    {
        osg::Vec3d world;
        _placeNodes[i]->getPosition().toWorld(world);
        osg::Vec3d screen = world * mvpw;

        if (screen.x() >= 0 && screen.x() <= viewport->width() &&
            screen.y() >= 0 && screen.y() <= viewport->height())
        {
            validPlaces.push_back(_placeNodes[i]);
            points.push_back({ screen.x(), screen.y() });
        }
    }

    kdbush::KDBush<TPoint> index(points);
    std::set< unsigned int > clustered;

    for (unsigned int i = 0; i < validPlaces.size(); i++)
    {
        TPoint &screen = points[i];
        PlaceNode* place = validPlaces[i].get();

        // If this thing is already part of a cluster then just continue.
        if (clustered.find(i) != clustered.end())
        {
            continue;
        }

        // Get any matching indices that are part of this cluster.
        TIds indices;
        index.within(screen.first, screen.second, _radius, [&indices](const auto id) { indices.push_back(id); });

        unsigned int actualCount = 0;

        // Add all of the points to the cluster.
        for (unsigned int j = 0; j < indices.size(); j++)
        {
            if (clustered.find(indices[j]) == clustered.end())
            {
                actualCount++;
                clustered.insert(indices[j]);
            }
        }

        std::stringstream buf;
        buf << actualCount << std::endl;
        place->setText(buf.str());

        out.push_back(place);

        clustered.insert(i);
    }
}

void ClusterNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = nv.asCullVisitor();

        PlaceNodeList places;
        getPlaces(cv->getCurrentCamera(), places);

        for (PlaceNodeList::iterator itr = places.begin(); itr != places.end(); ++itr)
        {
            itr->get()->accept(nv);
        }        
    }
    else
    {
        for (PlaceNodeList::iterator itr = _placeNodes.begin(); itr != _placeNodes.end(); ++itr)
        {
            itr->get()->accept(nv);
        }
    }
}