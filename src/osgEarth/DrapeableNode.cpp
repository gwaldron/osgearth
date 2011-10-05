/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/Utils>
#include <osgEarth/FindNode>

using namespace osgEarth;

DrapeableNode::DrapeableNode( MapNode* mapNode, bool draped ) :
_mapNode  ( mapNode ),
_newDraped( draped ),
_draped   ( false ),
_dirty    ( false )
{
    // create a container group that will house the culler. This culler
    // allows a draped node, which sits under the MapNode's OverlayDecorator,
    // to "track" the traversal state of the DrapeableNode itself.
    _nodeContainer = new osg::Group();
    _nodeContainer->setCullCallback( new CullNodeByFrameNumber() );
    _nodeContainer->setStateSet( this->getOrCreateStateSet() ); // share the stateset
}

void
DrapeableNode::applyChanges()
{    
    if ( _newDraped != _draped )
    {
        setDrapedImpl( _newDraped );
    }

    if ( _newNode.valid() )
    {
        setNodeImpl( _newNode.get() );
        _newNode = 0L;
    }
}

void
DrapeableNode::setNode( osg::Node* node )
{
    _newNode = node;
    if ( !_dirty )
    {
        _dirty = true;
        ADJUST_UPDATE_TRAV_COUNT( this, 1 );
    }
}

void
DrapeableNode::setDraped( bool draped )
{
    _newDraped = draped;
    if ( !_dirty )
    {
        _dirty = true;
        ADJUST_UPDATE_TRAV_COUNT( this, 1 );
    }
}

void
DrapeableNode::setNodeImpl( osg::Node* node )
{
    if ( _node.valid() )
    {
        if ( _draped && _mapNode.valid() )
        {
            _mapNode->getOverlayGroup()->removeChild( _nodeContainer.get() );
            _mapNode->updateOverlayGraph();
        }
        else
        {
            this->removeChild( _node.get() );
        }
    }

    _node = node;
    _nodeContainer->removeChildren( 0, _nodeContainer->getNumChildren() );

    if ( _node.valid() )
    {
        if ( _draped && _mapNode.valid() )
        {
            _nodeContainer->addChild( _node.get() );
            _mapNode->getOverlayGroup()->addChild( _nodeContainer.get() );
            _mapNode->updateOverlayGraph();
        }
        else
        {
            this->addChild( _node.get() );
        }
    }
}

void
DrapeableNode::setDrapedImpl( bool value )
{
    if ( _draped != value )
    {
        osg::ref_ptr<osg::Node> save = _node.get();
        if ( save.valid() )
            setNode( 0L );

        _draped = value;

        if ( save.valid() )
            setNode( save.get() );
    }
}


void
DrapeableNode::traverse( osg::NodeVisitor& nv )
{
    if ( _draped && nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR && _node.valid() && _mapNode.valid() )
    {
        CullNodeByFrameNumber* cb = static_cast<CullNodeByFrameNumber*>(_nodeContainer->getCullCallback());
        cb->_frame = nv.getFrameStamp()->getFrameNumber();
    }

    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        if ( _dirty )
        {
            applyChanges();

            _dirty = false;
            ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        }

        // traverse the subgraph
        if ( _nodeContainer.valid() && this->getNumChildrenRequiringUpdateTraversal() > 0 )
        {
            _nodeContainer->accept( nv );
        }
    }

    osg::Group::traverse( nv );
}
