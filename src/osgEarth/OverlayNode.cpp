/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/OverlayNode>
#include <osgEarth/CullingUtils>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/PrimitiveIntersector>
#include <osgEarth/DPLineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#define LC "[OverlayNode] "

using namespace osgEarth;

namespace
{
    /**
     * When draping is enabled, the actual active graph goes under an OverlayProxy
     * group. It tracks the accumulated stateset and nodemask of the Drapeable
     * itself and applies it to the active geometry (which is installed under the
     * MapNode's OverlayDecorator).
     */
    struct OverlayProxy : public osg::Group
    {
        OverlayProxy( osg::Node* owner ) 
            : _owner(owner) { }

        void traverse(osg::NodeVisitor& nv)
        {
            // only allow CULL and OD-internal traversal:
            if ( dynamic_cast<OverlayDecorator::InternalNodeVisitor*>(&nv) )
            {
                osg::Group::traverse( nv );
            }
            else if ( nv.getVisitorType() == nv.CULL_VISITOR && _owner.valid() )
            {
                // first find the highest ancestor in the owner's node parental node path that does
                // not occur in the visitor's node path. That is where we want to begin collecting
                // state.
                const osg::NodePath& visitorPath = nv.getNodePath();

                // get the owner's node path (just use the first one)
                osg::NodePathList ownerPaths;
                ownerPaths = _owner->getParentalNodePaths();

                // note: I descovered that getParentalNodePaths will stop when it finds an "invalid"
                // node mask (e.g., == zero).. so indeed it's possible for there to be zero node paths.
                if ( ownerPaths.size() > 0 )
                {
                    const osg::NodePath& ownerPath = ownerPaths[0];

                    // first check the owner's traversal mask.
                    bool visible = true;
                    for( int k = 0; visible && k < (int)ownerPath.size(); ++k )
                    {
                        visible = nv.validNodeMask(*ownerPath[k]);
                    }

                    if ( visible )
                    {
                        // find the intersection point:
                        int i = findIndexOfNodePathConvergence( visitorPath, ownerPath );

                        if ( i >= 0 && i < (int)ownerPath.size()-1 )
                        {
                            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

                            int pushes = 0;
                            for( int k = i+1; k < (int)ownerPath.size(); ++k )
                            {
                                osg::Node* node = ownerPath[k];
                                osg::StateSet* ss = ownerPath[k]->getStateSet();
                                if ( ss )
                                {
                                    cv->pushStateSet( ss );
                                    ++pushes;
                                }
                            }
                        
                            osg::Group::traverse( nv );

                            for( int k = 0; k < pushes; ++k )
                            {
                                cv->popStateSet();
                            }
                        }
                    }
                }
            }
        }


        // returns the deepest index into the ownerPath at which the two paths converge
        // (i.e. share a node pointer)
        int findIndexOfNodePathConvergence(const osg::NodePath& visitorPath, const osg::NodePath& ownerPath)
        {
            // use the knowledge that a NodePath is a vector.

            for( int vi = visitorPath.size()-1; vi >= 0; --vi )
            {
                osg::Node* visitorNode = visitorPath[vi];
                for( int oi = ownerPath.size()-1; oi >= 0; --oi )
                {
                    if ( ownerPath[oi] == visitorNode )
                    {
                        // found the deepest intersection, so set the start index to one higher.
                        return oi;
                    }
                }
            }   

            // no convergence. 
            return -1;
        }

        osg::observer_ptr<osg::Node> _owner;
    };
}

//------------------------------------------------------------------------

OverlayNode::OverlayNode( MapNode* mapNode, bool active, OverlayNode::TechniqueProvider provider ) :
_newActive( active ),
_active   ( false ),
_dirty    ( false ),
_getGroup ( provider )
{
    // create a container group that will house the culler. This culler
    // allows a active node, which sits under the MapNode's OverlayDecorator,
    // to "track" the traversal state of the OverlayNode itself.
    _overlayProxyContainer = new OverlayProxy( this );

    setMapNode( mapNode );

    if ( mapNode )
    {
        // If draping is requested, set up to apply it on the first update traversal.
        // Can't apply it until then since we need safe access to the MapNode.
        setActive( active );
    }
}

void
OverlayNode::setMapNode( MapNode* mapNode )
{
    MapNode* oldMapNode = getMapNode();

    if ( oldMapNode != mapNode )
    {
        if ( oldMapNode && _getGroup && _active && _overlayProxyContainer->getNumParents() > 0 )
        {
            osg::Group* group = _getGroup( oldMapNode );
            if ( group )
                group->removeChild( _overlayProxyContainer.get() );
        }

        _mapNode = mapNode;

        applyChanges();
    }
}

void
OverlayNode::setTechniqueProvider( OverlayNode::TechniqueProvider p )
{
    osg::ref_ptr<MapNode> save = getMapNode();
    if ( save.valid() )
        setMapNode( 0L );

    _getGroup = p;

    if ( save.valid() )
        setMapNode( save.get() );
}

void
OverlayNode::applyChanges()
{
    _active = _newActive;

    if ( getMapNode() && _getGroup )
    {
        if ( _active && _overlayProxyContainer->getNumParents() == 0 )
        {
            osg::Group* group = _getGroup( getMapNode() );
            if ( group )
                group->addChild( _overlayProxyContainer.get() );
        }
        else if ( !_active && _overlayProxyContainer->getNumParents() > 0 )
        {
            osg::Group* group = _getGroup( getMapNode() );
            if ( group )
                group->removeChild( _overlayProxyContainer.get() );
        }

        dirtyBound();
    }
}

void
OverlayNode::setActive( bool active )
{    
    if ( active != _active )
    {        
        _newActive = active;
        if ( !_dirty )
        {
            _dirty = true;
            ADJUST_UPDATE_TRAV_COUNT( this, 1 );
        }        
    }
}

bool
OverlayNode::addChild( osg::Node* child )
{
    bool ok = osg::Group::addChild( child );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->addChild( child );
    return ok;
}

bool
OverlayNode::insertChild( unsigned i, osg::Node* child )
{
    bool ok = osg::Group::insertChild( i, child );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->insertChild( i, child );
    return ok;
}

bool
OverlayNode::removeChild( osg::Node* child )
{
    bool ok = osg::Group::removeChild( child );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->removeChild( child );
    return ok;
}

bool
OverlayNode::replaceChild( osg::Node* oldChild, osg::Node* newChild )
{
    bool ok = osg::Group::replaceChild( oldChild, newChild );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->replaceChild( oldChild, newChild );
    return ok;
}

void
OverlayNode::traverse( osg::NodeVisitor& nv )
{
    if ( !_overlayProxyContainer.valid() )
    {
        osg::Group::traverse( nv );
    }
    else
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            if ( _active )
            {
                // do nothing -- culling will happen via the OverlayProxy instead.
            }
            else
            {
                // for a non-active node, just traverse children as usual.
                osg::Group::traverse( nv );
            }
        }

        else if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
        {
            if ( _dirty )
            {
                applyChanges();
                _dirty = false;
                ADJUST_UPDATE_TRAV_COUNT( this, -1 );
            }
            
            // traverse children directly, regardles of active status
            osg::Group::traverse( nv );
        }

        else if (dynamic_cast<osgUtil::IntersectionVisitor*>(&nv))
        {
            /*
               In order to properly intersect with overlay geometries, attempt to find the point on the terrain where the pick occurred
               cast a second intersector vertically at that point.

               Currently this is only imlpemented for our custom PrimitiveIntersector.
            */
            osgUtil::IntersectionVisitor* iv = dynamic_cast<osgUtil::IntersectionVisitor*>(&nv);
            osgEarth::PrimitiveIntersector* pi = dynamic_cast<osgEarth::PrimitiveIntersector *>(iv->getIntersector());

            if (pi && !pi->getOverlayIgnore() && getMapNode())
            {
                osg::ref_ptr<MapNode> mapNode = getMapNode();
 
                osg::NodePath path = iv->getNodePath();
                osg::NodePath prunedNodePath( path.begin(), path.end()-1 );
                osg::Matrix modelToWorld = osg::computeLocalToWorld(prunedNodePath);
                osg::Vec3d worldStart = pi->getStart() * modelToWorld;
                osg::Vec3d worldEnd = pi->getEnd() * modelToWorld;

                osg::ref_ptr<DPLineSegmentIntersector> lsi = new DPLineSegmentIntersector(worldStart, worldEnd);
                osgUtil::IntersectionVisitor ivTerrain(lsi.get());
                _mapNode->getTerrainEngine()->accept(ivTerrain);

                if (lsi->containsIntersections())
                {
                  osg::Vec3d worldIntersect = lsi->getFirstIntersection().getWorldIntersectPoint();
                  
                  GeoPoint mapIntersect;
                  mapIntersect.fromWorld(mapNode->getMapSRS(), worldIntersect);

                  osg::Vec3d newMapStart(mapIntersect.x(), mapIntersect.y(), 25000.0);
                  osg::Vec3d newMapEnd(mapIntersect.x(), mapIntersect.y(), -25000.0);

                  osg::Vec3d newWorldStart;
                  mapNode->getMapSRS()->transformToWorld(newMapStart, newWorldStart);

                  osg::Vec3d newWorldEnd;
                  mapNode->getMapSRS()->transformToWorld(newMapEnd, newWorldEnd);

                  osg::Matrix worldToModel;
                  worldToModel.invert(modelToWorld);

                  osg::Vec3d newModelStart = newWorldStart * worldToModel;
                  osg::Vec3d newModelEnd = newWorldEnd * worldToModel;

                  osg::ref_ptr<osgEarth::PrimitiveIntersector> pi2 = new osgEarth::PrimitiveIntersector(osgUtil::Intersector::MODEL, newModelStart, newModelEnd, pi->getThickness(), true);
                  osgUtil::IntersectionVisitor iv2(pi2);
                  iv2.setTraversalMask(iv->getTraversalMask());
                  path[0]->accept(iv2);

                  if (pi2->containsIntersections())
                  {
                    // Insert newlly found intersections into the original intersector.
                    for (PrimitiveIntersector::Intersections::iterator it = pi2->getIntersections().begin(); it != pi2->getIntersections().end(); ++it)
                      pi->insertIntersection(*it);
                  }
                }
                else
                {
                  //OE_WARN << LC << "No hits on terrain!" << std::endl;
                }
            }
            else
            {
              osg::Group::traverse( nv );
            }
        }

        // handle other visitor types (like intersections, etc) by simply
        // traversing the child graph.
        else // if ( nv.getNodeVisitor() == osg::NodeVisitor::NODE_VISITOR )
        {
            osg::Group::traverse( nv );
        }
    }
}
