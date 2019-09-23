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
#include <osgEarthUtil/NearFarGroup>
#include <osgEarth/FindNode>
#include <osg/Depth>
#include <iomanip>

#define LC "[NearFarClip] "

using namespace osgEarth::Util;


NearFarGroup::NearFarGroup() :
osg::Camera()
{
    this->setRenderOrder( osg::Camera::POST_RENDER, 2 );
    this->setReferenceFrame( osg::Transform::RELATIVE_RF );
    this->setClearMask( 0 );
    this->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    this->setNearFarRatio( 0.00001 );
}

void
NearFarGroup::traverse( osg::NodeVisitor& nv )
{
    bool isCull = nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR;
    osgUtil::CullVisitor* cv;
    if ( isCull )
        cv = static_cast<osgUtil::CullVisitor*>(&nv);

    if ( isCull )
    {
        // bounding sphere of the child graph
        const osg::BoundingSphere& bs = this->getBound();

        // the actual camera at the root of this render stage
        osg::Camera* realCamera = cv->getRenderStage()->getCamera();

        // use the world look vector to calculate the near/far points in world space:
        const osg::Matrixd& viewMatrix = realCamera->getViewMatrix();
        osg::Vec3d lookVectorInWorldCoords = osg::Matrixd::transform3x3(viewMatrix, osg::Vec3d(0.0,0.0,-1.0));
        lookVectorInWorldCoords.normalize();
        osg::Vec3d nearPointInWorldCoords = bs.center() - lookVectorInWorldCoords*bs.radius();
        osg::Vec3d farPointInWorldCoords = bs.center() + lookVectorInWorldCoords*bs.radius();

        // convert those to eye space to get the new near/far values:
        osg::Vec3d nearPointInEyeCoords = nearPointInWorldCoords * viewMatrix;
        osg::Vec3d farPointInEyeCoords = farPointInWorldCoords * viewMatrix;

        double scene_zNear = -nearPointInEyeCoords.z();
        double scene_zFar = -farPointInEyeCoords.z();

        OE_DEBUG
            << std::fixed << std::setprecision(2)
            << "Center = " << bs.center().x() << "," << bs.center().y() << "," << bs.center().z()
            << ", Radius = " << bs.radius()
            << ", Near = " << scene_zNear
            << ", Far = " << scene_zFar
            << std::endl;

        // alter a negative znear by re-calculating it based on the far clip:
        if ( scene_zNear <= 0.0 ) 
            scene_zNear = this->getNearFarRatio() * scene_zFar;

        // calculate a new projection matrix using the calculated near/far clip planes.
        osg::ref_ptr<osg::RefMatrix> proj = new osg::RefMatrix();
        double left, right, top, bottom, zNear, zFar;
        if ( realCamera->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar) )
        {
            double nr = scene_zNear / zNear;
            proj->makeFrustum(left * nr, right * nr, bottom * nr, top * nr, scene_zNear, scene_zFar);
            this->setProjectionMatrix( *proj.get() );      
        }

        // replace whatever projection matrix is atop the stack before culling the subgraph:
        cv->pushProjectionMatrix( proj.get() );
    }

    osg::Camera::traverse(nv);

    if ( isCull )
    {
        // pop our temporary projection matrix now that we're done
        cv->popProjectionMatrix();
    }
}
