/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/DrapingCullSet>
#include <osgEarth/CullingUtils>
#include <osgEarth/NodeUtils>

#define LC "[DrapeableNode] "

using namespace osgEarth;


DrapeableNode::DrapeableNode() :
_drapingEnabled( true ),
_updateRequested( true )
{
    // Unfortunetly, there's no way to return a correct bounding sphere for
    // the node since the draping will move it to the ground. The bounds
    // check has to be done by the Draping Camera at cull time. Therefore we
    // have to ensure that this node makes it into the draping cull set so it
    // can be frustum-culled at the proper time.
    setCullingActive( !_drapingEnabled );

    // activate an update traversal to find the MapNode
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}

DrapeableNode::DrapeableNode(const DrapeableNode& rhs, const osg::CopyOp& copy) :
osg::Group(rhs, copy)
{
    _drapingEnabled = rhs._drapingEnabled;
    _updateRequested = rhs._updateRequested;
}

void
DrapeableNode::setDrapingEnabled(bool value)
{
    if ( value != _drapingEnabled )
    {
        _drapingEnabled = value;
        setCullingActive( !_drapingEnabled );
    }
}

void
DrapeableNode::traverse(osg::NodeVisitor& nv)
{
    if ( _drapingEnabled && nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // find the cull set for this camera:
        osg::ref_ptr<MapNode> mapNode;
        if (_mapNode.lock(mapNode))
        {
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            if (cv->getCurrentRenderBin()->getName() != "OE_EMPTY_RENDER_BIN")
            {
                DrapingCullSet& cullSet = mapNode->getDrapingManager()->get(cv->getCurrentCamera());
                cullSet.push(this, cv->getNodePath(), nv.getFrameStamp());
            }
        }
    }

    else if (_drapingEnabled && nv.getVisitorType() == nv.UPDATE_VISITOR)
    {        
        if (_updateRequested)
        {
            if (_mapNode.valid() == false)
            {
                _mapNode = osgEarth::findInNodePath<MapNode>(nv);
            }

            if (_mapNode.valid())
            {
                _updateRequested = false;
                ADJUST_UPDATE_TRAV_COUNT(this, -1);
            }
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
#define LC "[DrapeableNode Serializer] "


namespace osgEarth { namespace Serializers { namespace DrapeableNode
{
    REGISTER_OBJECT_WRAPPER(
        DrapeableNode,
        new osgEarth::DrapeableNode,
        osgEarth::DrapeableNode,
        "osg::Object osg::Node osg::Group osgEarth::DrapeableNode")
    {
        ADD_BOOL_SERIALIZER(DrapingEnabled, true);
    }
} } }
