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
_mapNodeUpdateRequested(true)
{
    // bounding box culling doesn't work on clampable geometry
    // since the GPU will be moving verts. So, disable the default culling
    // for this node so we can out own culling in traverse().
    setCullingActive(false);

    // for the mapnode update:
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}

void
ClampableNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // Lock a reference to the map node:
        osg::ref_ptr<MapNode> mapNode;
        if (_mapNode.lock(mapNode))
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

            // Custom culling. Since clamped geometry can start far outside of the 
            // view frustum, normal culling won't work. Instead, project the
            // bounding sphere upwards (along its up vector) on to the center plane
            // of the view frustum. Then cull it based on the new simulated location.
            osg::RefMatrix* MV = cv->getModelViewMatrix();
            osg::Matrix MVinverse;
            MVinverse.invert(*MV);

            // Actual bounds of geometry:
            osg::BoundingSphere bs = getBound();

            // Find any two points on the bounding sphere's up-vector
            // and transform them into view space:
            osg::Vec3d p0 = bs.center() * (*MV);
            osg::Vec3d p1 =
                mapNode->isGeocentric() ? (bs.center() * 2.0 * (*MV)) :
                                          (bs.center() + osg::Vec3d(0, 0, bs.radius())) * (*MV);

            // Center plane of the view frustum (in view space)
            static osg::Vec3d v0(0, 0, 0);  // point on the plane
            static osg::Vec3d n(0, 1, 0);   // normal vector to the plane

            // Find the intersection of the up vector and the center plane
            // and then transform the result back into world space for culling.
            osg::Vec3d w = p0 - v0;
            osg::Vec3d u = p1 - p0;
            double t = (-n * w) / (n * u);
            bs.center() = (p0 + u*t) * MVinverse;

            if (cv->isCulled(bs) == false)
            {
                // Passed the cull test, so put this node in the clamping cull set.
                ClampingCullSet& cullSet = mapNode->getClampingManager()->get( cv->getCurrentCamera() );
                cullSet.push( this, cv->getNodePath(), nv.getFrameStamp() );
            }
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
