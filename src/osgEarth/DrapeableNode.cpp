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

#include <osgEarth/DrapeableNode>
#include <osgEarth/DrapingCullSet>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>

#define LC "[DrapeableNode] "

using namespace osgEarth;


DrapeableNode::DrapeableNode() :
_drapingEnabled( true )
{
    // Unfortunetly, there's no way to return a correct bounding sphere for
    // the node since the draping will move it to the ground. The bounds
    // check has to be done by the Draping Camera at cull time. Therefore we
    // have to ensure that this node makes it into the draping cull set so it
    // can be frustum-culled at the proper time.
    setCullingActive( !_drapingEnabled );
}

DrapeableNode::DrapeableNode(const DrapeableNode& rhs, const osg::CopyOp& copy) :
osg::Group(rhs, copy)
{
    _drapingEnabled = rhs._drapingEnabled;
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
        // access the cull visitor:
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

        // find the cull set for this camera:
        DrapingCullSet& cullSet = DrapingCullSet::get( cv->getCurrentCamera() );
        cullSet.push( this, cv->getNodePath(), nv.getFrameStamp() );
    }
    else
    {
        osg::Group::traverse( nv );
    }
}

//...........................................................................

#undef  LC
#define LC "[DrapeableNode Serializer] "

#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

namespace
{
    REGISTER_OBJECT_WRAPPER(
        DrapeableNode,
        new osgEarth::DrapeableNode,
        osgEarth::DrapeableNode,
        "osg::Object osg::Node osg::Group osgEarth::DrapeableNode")
    {
        ADD_BOOL_SERIALIZER(DrapingEnabled, true);
    }
}
