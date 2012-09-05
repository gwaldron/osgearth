/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

#include <osgEarth/DrapeableNode>
#include <osgEarth/CullingUtils>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgUtil/IntersectionVisitor>

#define LC "[DrapeableNode] "

using namespace osgEarth;

namespace
{
#if 0
    // Custom group that limits traversals to CULL and any visitor internal to
    // the operation of the OverlayDecorator.
    struct OverlayTraversalGroup : public osg::Group {
        virtual void traverse(osg::NodeVisitor& nv) {
            if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR ||  
                 dynamic_cast<OverlayDecorator::InternalNodeVisitor*>(&nv) )
            {
                osg::Group::traverse(nv);
            }
        }
    };
#endif

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
                    for( int k = 0; visible && k < ownerPath.size(); ++k )
                    {
                        visible = nv.validNodeMask(*ownerPath[k]);
                    }

                    if ( visible )
                    {
                        // find the intersection point:
                        int i = findIndexOfNodePathConvergence( visitorPath, ownerPath );

                        if ( i >= 0 && i < ownerPath.size()-1 )
                        {
                            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

                            int pushes = 0;
                            for( int k = i+1; k < ownerPath.size(); ++k )
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

DrapeableNode::DrapeableNode( MapNode* mapNode, bool draped ) :
_newDraped( draped ),
_draped   ( false ),
_dirty    ( false )
{
    // create a container group that will house the culler. This culler
    // allows a draped node, which sits under the MapNode's OverlayDecorator,
    // to "track" the traversal state of the DrapeableNode itself.
    _overlayProxyContainer = new OverlayProxy( this );

    setMapNode( mapNode );

    if ( mapNode )
    {
        // If draping is requested, set up to apply it on the first update traversal.
        // Can't apply it until then since we need safe access to the MapNode.
        setDraped( draped );
    }
    else
    {
        OE_DEBUG << LC << "Creates a drapeable without a MapNode; draping will be disabled" << std::endl;
    }
}

void
DrapeableNode::setMapNode( MapNode* mapNode )
{
    MapNode* oldMapNode = getMapNode();

    if ( oldMapNode != mapNode )
    {
        if ( oldMapNode && _draped && _overlayProxyContainer->getNumParents() > 0 )
        {
            oldMapNode->getOverlayGroup()->removeChild( _overlayProxyContainer.get() );
            oldMapNode->updateOverlayGraph();
        }

        _mapNode = mapNode;

        applyChanges();
    }
}

void
DrapeableNode::applyChanges()
{
    _draped = _newDraped;

    if ( getMapNode() )
    {
        if ( _draped && _overlayProxyContainer->getNumParents() == 0 )
        {
            getMapNode()->getOverlayGroup()->addChild( _overlayProxyContainer.get() );
            getMapNode()->updateOverlayGraph();
        }
        else if ( !_draped && _overlayProxyContainer->getNumParents() > 0 )
        {
            getMapNode()->getOverlayGroup()->removeChild( _overlayProxyContainer.get() );
            getMapNode()->updateOverlayGraph();
        }

        dirtyBound();
    }
}

void
DrapeableNode::setDraped( bool draped )
{    
    if ( draped != _draped && getMapNode() )
    {        
        _newDraped = draped;
        if ( !_dirty )
        {
            _dirty = true;
            ADJUST_UPDATE_TRAV_COUNT( this, 1 );
        }        
    }
}

bool
DrapeableNode::addChild( osg::Node* child )
{
    bool ok = osg::Group::addChild( child );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->addChild( child );
    return ok;
}

bool
DrapeableNode::insertChild( unsigned i, osg::Node* child )
{
    bool ok = osg::Group::insertChild( i, child );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->insertChild( i, child );
    return ok;
}

bool
DrapeableNode::removeChild( osg::Node* child )
{
    bool ok = osg::Group::removeChild( child );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->removeChild( child );
    return ok;
}

bool
DrapeableNode::replaceChild( osg::Node* oldChild, osg::Node* newChild )
{
    bool ok = osg::Group::replaceChild( oldChild, newChild );
    if ( _overlayProxyContainer.valid() )
        _overlayProxyContainer->replaceChild( oldChild, newChild );
    return ok;
}

void
DrapeableNode::traverse( osg::NodeVisitor& nv )
{
    if ( !_overlayProxyContainer.valid() )
    {
        osg::Group::traverse( nv );
    }
    else
    {
        if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            if ( _draped )
            {
                // do nothing -- culling will happen via the OverlayProxy instead.
            }
            else
            {
                // for a non-draped node, just traverse children as usual.
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
            
            // traverse children directly, regardles of draped status
            osg::Group::traverse( nv );
        }

        // handle other visitor types (like intersections, etc) by simply
        // traversing the child graph.
        else // if ( nv.getNodeVisitor() == osg::NodeVisitor::NODE_VISITOR )
        {
            osg::Group::traverse( nv );
        }
    }
}
