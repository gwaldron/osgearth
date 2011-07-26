/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2011 Pelican Mapping
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

using namespace osgEarth;

//------------------------------------------------------------------------

DrapeableNode::DrapeableNode( MapNode* mapNode, bool draped ) :
_mapNode( mapNode ),
_draped ( draped )
{
    //nop
}

void
DrapeableNode::setDraped( bool value )
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
DrapeableNode::setNode( osg::Node* node )
{
    if ( _node.valid() )
    {
        if ( _draped && _mapNode.valid() )
        {
            _mapNode->getOverlayGroup()->removeChild( _node.get() );
            _mapNode->updateOverlayGraph();
        }
        else
        {
            this->removeChild( _node.get() );
        }
    }

    _node = node;

    if ( _node.valid() )
    {
        if ( _draped && _mapNode.valid() )
        {
            _mapNode->getOverlayGroup()->addChild( _node.get() );
            _mapNode->updateOverlayGraph();
        }
        else
        {
            this->addChild( _node.get() );
        }
    }
}
