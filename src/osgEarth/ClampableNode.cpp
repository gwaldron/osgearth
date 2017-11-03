/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/NodeUtils>

#define LC "[ClampableNode] "

using namespace osgEarth;


ClampableNode::ClampableNode() :
_depthOffsetUpdateRequested(false),
_mapNodeUpdateRequested(true)
{
    _adapter.setGraph( this );

    // for the mapnode update:
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}

void
ClampableNode::setDepthOffsetOptions(const DepthOffsetOptions& options)
{
    _adapter.setDepthOffsetOptions(options);
    if ( _adapter.isDirty() && !_depthOffsetUpdateRequested )
        scheduleDepthOffsetUpdate();
}

const DepthOffsetOptions&
ClampableNode::getDepthOffsetOptions() const
{
    return _adapter.getDepthOffsetOptions();
}

void
ClampableNode::scheduleDepthOffsetUpdate()
{
    if ( !_depthOffsetUpdateRequested && getDepthOffsetOptions().enabled() == true )
    {
        ADJUST_UPDATE_TRAV_COUNT(this, +1);
        _depthOffsetUpdateRequested = true;
    }
}

void
ClampableNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // find the cull set for this camera:
        osg::ref_ptr<MapNode> mapNode;
        if (_mapNode.lock(mapNode))
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
            ClampingCullSet& cullSet = mapNode->getClampingManager()->get( cv->getCurrentCamera() );
            cullSet.push( this, cv->getNodePath(), nv.getFrameStamp() );
        }
    }

    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {        
        if (_mapNodeUpdateRequested)
        {
            if (_mapNode.valid() == false)
            {
                _mapNode = osgEarth::findInNodePath<MapNode>(nv);
            }

            if (_mapNode.valid())
            {
                _mapNodeUpdateRequested = false;
                ADJUST_UPDATE_TRAV_COUNT(this, -1);
            }
        }

        if (_depthOffsetUpdateRequested)
        {
            _adapter.recalculate();
            _depthOffsetUpdateRequested = false;
            ADJUST_UPDATE_TRAV_COUNT(this, -1);
        }

        osg::Group::traverse(nv);
    }
    else
    {
        osg::Group::traverse(nv);
    }
}


//...........................................................................

#undef  LC
#define LC "[ClampableNode Serializer] "

#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

namespace
{
    REGISTER_OBJECT_WRAPPER(
        ClampableNode,
        new osgEarth::ClampableNode,
        osgEarth::ClampableNode,
        "osg::Object osg::Node osg::Group osgEarth::ClampableNode")
    {
        //nop
    }
}
