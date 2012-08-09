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
    _overlayProxyContainer = new OverlayTraversalGroup();
    _overlayProxyContainer->setCullCallback( new CullNodeByFrameNumber() );
    _overlayProxyContainer->setStateSet( this->getOrCreateStateSet() ); // share the stateset

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
            //Set a giant bounding sphere on this node so it always passes the view frustum test and will be included in the OverlayDecorator.
            setComputeBoundingSphereCallback( new StaticBound( osg::BoundingSphere(osg::Vec3f(0,0,0), FLT_MAX)));
        }
        else if ( !_draped && _overlayProxyContainer->getNumParents() > 0 )
        {
            getMapNode()->getOverlayGroup()->removeChild( _overlayProxyContainer.get() );
            getMapNode()->updateOverlayGraph();
            //Remove the bounding sphere callback, it's just a regular node.
            setComputeBoundingSphereCallback( 0L );
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
                // for a draped node, inform the proxy that we are visible. But do NOT traverse the
                // child graph (since it will be traversed by the OverlayDecorator).
                CullNodeByFrameNumber* cb = static_cast<CullNodeByFrameNumber*>(_overlayProxyContainer->getCullCallback());
                if (cb)
                {
                    cb->_frame = nv.getFrameStamp()->getFrameNumber();
                }
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
