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
#include "QuadTileNode"

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[QuadTileNode] "

QuadTileNode::QuadTileNode() :
_range( FLT_MAX ),
_timestamp( DBL_MAX )
{
    //NOP
}

void
QuadTileNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR && nv.getFrameStamp() )
    {
        // record the timestamp of the last traversal.
        _timestamp = nv.getFrameStamp()->getReferenceTime();
    }

    bool traverseChildren = true;
    for( int i=0; i<4; ++i )
    {
        if ( !_children[i].valid() || !_children[i]->isActive() )
        {
            traverseChildren = false;
            break;
        }
    }

    if ( traverseChildren )
    {
        for( int i=0; i<4; ++i )
        {
            _children[i]->accept( nv );
        }
    }
    else
    {
        // traverse this group's child (the local tile itself)
        osg::Group::traverse( nv );
    }        
}

