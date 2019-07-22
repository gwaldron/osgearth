/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

            // First check for simple intersection at the geometry's actual position:
            bool visible = false; //(cv->isCulled(bs) == false);
            if (!visible)
            {
                // Failing that, project the geometry to the ellipsoid's surface and
                // expand the radius to account for reasonable elevation changes.
                // On Earth, elevations of +/- 12000m results in a variance of
                // 12000/R = ~0.002. Obviously this differs for other planets but
                // good enough for now
                const double variance = 0.002;  // * R.

                const SpatialReference* mapSRS = mapNode->getMapSRS();
                if (mapSRS->isGeographic())
                {
                    osg::Vec3d p0 = bs.center();
                    p0.normalize();

                    // approximate radius under bs.center:
                    double R = 
                        osg::absolute(p0.z()) * mapSRS->getEllipsoid()->getRadiusPolar() +
                        (1.0 - osg::absolute(p0.z())) * mapSRS->getEllipsoid()->getRadiusEquator();

                    // project to mean surface:
                    bs.center() = p0 * R;

                    // buffer the radius to account for elevation data
                    bs.radius() = bs.radius() + R*variance;
                }

                else // projected
                {
                    double R = osg::maximum(
                        mapSRS->getEllipsoid()->getRadiusPolar(),
                        mapSRS->getEllipsoid()->getRadiusEquator());
                    
                    // project to mean surface:
                    bs.center().z() = 0.0;

                    // buffer the radius to account for elevation data
                    bs.radius() = bs.radius() + R*variance;
                }

                // Test against the virtual bounding sphere
                visible = (cv->isCulled(bs) == false);
            }

            if (visible)
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


bool ClampableNode::isDepthCamera(const osg::Camera* camera)
{
    if (camera->getStateSet() == NULL)
    {
        return false;
    }

    // Check for the existence of the OE_IS_DEPTH_CAMERA define
    return (camera->getStateSet()->getDefineList().find("OE_IS_DEPTH_CAMERA") != camera->getStateSet()->getDefineList().end());
}


//...........................................................................

#undef  LC
#define LC "[ClampableNode Serializer] "


namespace osgEarth { namespace Serializers { namespace ClampableNode
{
    REGISTER_OBJECT_WRAPPER(
        ClampableNode,
        new osgEarth::ClampableNode,
        osgEarth::ClampableNode,
        "osg::Object osg::Node osg::Group osgEarth::ClampableNode")
    {
        //nop
    }
}}}
