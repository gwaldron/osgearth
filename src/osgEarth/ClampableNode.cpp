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

#include <osgEarth/ClampableNode>
#include <osgEarth/ClampingTechnique>
#include <osgEarth/DepthOffset>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/MapNode>
#include <osgEarth/VirtualProgram>

#define LC "[ClampableNode] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    static osg::Group* getTechniqueGroup(MapNode* m)
    {
        return m ? m->getOverlayDecorator()->getGroup<ClampingTechnique>() : 0L;
    }
}

//------------------------------------------------------------------------

ClampableNode::ClampableNode( MapNode* mapNode, bool active ) :
OverlayNode( mapNode, active, &getTechniqueGroup ),
_updatePending( false )
{
    _adapter.setGraph( this );

    if ( _adapter.isDirty() )
        _adapter.recalculate();
}

void
ClampableNode::setDepthOffsetOptions(const DepthOffsetOptions& options)
{
    _adapter.setDepthOffsetOptions(options);
    if ( _adapter.isDirty() && !_updatePending )
        scheduleUpdate();
}

const DepthOffsetOptions&
ClampableNode::getDepthOffsetOptions() const
{
    return _adapter.getDepthOffsetOptions();
}

void
ClampableNode::scheduleUpdate()
{
    if ( !_updatePending && getDepthOffsetOptions().enabled() == true )
    {
        ADJUST_UPDATE_TRAV_COUNT(this, 1);
        _updatePending = true;
    }
}

osg::BoundingSphere
ClampableNode::computeBound() const
{
    static Threading::Mutex s_mutex;
    {
        Threading::ScopedMutexLock lock(s_mutex);
        const_cast<ClampableNode*>(this)->scheduleUpdate();
    }
    return OverlayNode::computeBound();
}

void
ClampableNode::traverse(osg::NodeVisitor& nv)
{
    if ( _updatePending && nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        _adapter.recalculate();
        ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        _updatePending = false;
    }
    OverlayNode::traverse( nv );
}
