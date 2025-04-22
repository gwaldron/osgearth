/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/NodeUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef  LC
#define LC "[ObserverGroup] "

ObserverGroup::ObserverGroup()
{
    ADJUST_EVENT_TRAV_COUNT(this, 1);
}

void
ObserverGroup::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == nv.EVENT_VISITOR )
    {
        // check for orphans:
        for(osg::NodeList::iterator itr = _children.begin(); itr != _children.end(); ++itr )
        {
            if ( (*itr)->referenceCount() == 1 )
            {
                // found one, queue an update traversal so we can safely delete it.
                // (it's probably safe to just delete it here, but anyway)
                if ( _orphans.insert(itr->get()).second == true )
                {
                    ADJUST_UPDATE_TRAV_COUNT( this, 1 );
                }
            }
        }
    }

    else if ( nv.getVisitorType() == nv.UPDATE_VISITOR && _orphans.size() > 0 )
    {
        // delete orphans:
        for( std::set<osg::Node*>::iterator i = _orphans.begin(); i != _orphans.end(); ++i )
        {
            this->removeChild( *i );
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        }

        _orphans.clear();
    }

    osg::Group::traverse( nv );
}

//----------------------------------------------------------------------------
EnableAutoUnloadVisitor::EnableAutoUnloadVisitor() :
    osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
}

void EnableAutoUnloadVisitor::apply(osg::Node& node)
{
    LoadableNode* loadableNode = dynamic_cast<LoadableNode*>(&node);
    if (loadableNode)
    {
        loadableNode->setAutoUnload(true);
    }
    traverse(node);
}

//----------------------------------------------------------------------------

LoadDataVisitor::LoadDataVisitor() :
    osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
}

bool LoadDataVisitor::isFullyLoaded() const
{
    return _fullyLoaded;
}

void LoadDataVisitor::reset()
{
    _fullyLoaded = true;
}

bool LoadDataVisitor::intersects(osg::Node& node)
{
    static osg::Matrix identity;
    osg::Matrix& matrix = _matrixStack.empty() ? identity : _matrixStack.back();

    osg::BoundingSphere nodeBounds = node.getBound();
    osg::BoundingSphered worldBounds(nodeBounds.center(), nodeBounds.radius());
    worldBounds.center() = worldBounds.center() * matrix;

    for (auto& bs : _areasToLoad)
    {
        if (bs.intersects(worldBounds))
        {
            return true;
        }
    }
    return false;
}

std::vector<osg::BoundingSphered>& LoadDataVisitor::getAreasToLoad() { return _areasToLoad; }

bool LoadDataVisitor::getLoadHighestResolutionOnly() const
{
    return _loadHighestResolutionOnly;
}

void LoadDataVisitor::setLoadHighestResolutionOnly(bool value)
{
    _loadHighestResolutionOnly = value;
}

void LoadDataVisitor::apply(LoadableNode& node)
{
    if (_loadHighestResolutionOnly)
    {
        if (!node.isLoadComplete())
        {
            if (node.getRefinePolicy() == REFINE_ADD || node.isHighestResolution())
            {
                node.setAutoUnload(false);
                node.load();
                _fullyLoaded = false;
            }
        }
    }
    else
    {
        node.setAutoUnload(false);
        if (!node.isLoadComplete())
        {
            node.load();
            _fullyLoaded = false;
        }
    }
}

void LoadDataVisitor::apply(osg::Node& node)
{
    if (intersects(node))
    {
        LoadableNode* loadableNode = dynamic_cast<LoadableNode*>(&node);
        if (loadableNode)
        {
            apply(*loadableNode);
        }

        PagingManager* pagingManager = dynamic_cast<PagingManager*>(&node);
        if (pagingManager)
        {
            _pagingManagers.insert(pagingManager);
        }

        traverse(node);
    }
}

void LoadDataVisitor::apply(osg::Transform& transform)
{
    if (intersects(transform))
    {
        osg::Matrix matrix;
        if (!_matrixStack.empty()) matrix = _matrixStack.back();
        transform.computeLocalToWorldMatrix(matrix, this);
        pushMatrix(matrix);

        LoadableNode* loadableNode = dynamic_cast<LoadableNode*>(&transform);
        if (loadableNode)
        {
            apply(*loadableNode);
        }


        traverse(transform);
        popMatrix();
    }
}

void LoadDataVisitor::manualUpdate()
{
    for (auto& m : _pagingManagers)
    {
        m->update();
    }
}
//----------------------------------------------------------------------------

void osgEarth::Util::loadData(osg::Node* node, std::vector<osg::BoundingSphered>& areasToLoad)
{
    // Add the areas to load to the visitor.
    LoadDataVisitor v;
    for (auto& a: areasToLoad)
    {
        v.getAreasToLoad().push_back(a);
    }

    // Send the visitor down the scene graph, loading data incrementally until it's fully loaded.
    bool fullyLoaded = false;
    while (!fullyLoaded)
    {
        v.reset();
        node->accept(v);
        fullyLoaded = v.isFullyLoaded();
        // Call manual update on the PagingManger to peform merges.
        v.manualUpdate();
    }
}