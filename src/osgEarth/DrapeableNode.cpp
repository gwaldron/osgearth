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
_mapNode  ( mapNode ),
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

    // If draping is requested, set up to apply it on the first update traversal.
    // Can't apply it until then since we need safe access to the MapNode.
    setDraped( draped );

    if ( !mapNode )
    {
        OE_WARN << LC << "Creates a drapeable without a MapNode; draping will be disabled" << std::endl;
    }
}

void
DrapeableNode::applyChanges()
{
    _draped = _newDraped;

    if ( _mapNode.valid() )
    {
        if ( _draped && _overlayProxyContainer->getNumParents() == 0 )
        {
            _mapNode->getOverlayGroup()->addChild( _overlayProxyContainer.get() );
            _mapNode->updateOverlayGraph();
        }
        else if ( !_draped && _overlayProxyContainer->getNumParents() > 0 )
        {
            _mapNode->getOverlayGroup()->removeChild( _overlayProxyContainer.get() );
            _mapNode->updateOverlayGraph();
        }
    }
}

void
DrapeableNode::setDraped( bool draped )
{
    if ( draped != _draped && _mapNode.valid() )
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
    osg::Group::addChild( child );
    return _overlayProxyContainer->addChild( child );
}

bool
DrapeableNode::insertChild( unsigned i, osg::Node* child )
{
    osg::Group::insertChild( i, child );
    return _overlayProxyContainer->insertChild( i, child );
}

bool
DrapeableNode::removeChild( osg::Node* child )
{
    osg::Group::removeChild( child );
    return _overlayProxyContainer->removeChild( child );
}

bool
DrapeableNode::replaceChild( osg::Node* oldChild, osg::Node* newChild )
{
    osg::Group::replaceChild( oldChild, newChild );
    return _overlayProxyContainer->replaceChild( oldChild, newChild );
}

void
DrapeableNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        if ( _draped )
        {
            // for a draped node, inform the proxy that we are visible. But do NOT traverse the
            // child graph (since it will be traversed by the OverlayDecorator).
            CullNodeByFrameNumber* cb = static_cast<CullNodeByFrameNumber*>(_overlayProxyContainer->getCullCallback());
            cb->_frame = nv.getFrameStamp()->getFrameNumber();
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
