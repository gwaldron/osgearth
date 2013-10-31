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
#include <osgEarth/Pickers>
#include <osgEarth/PrimitiveIntersector>

#define LC "[Picker] "

using namespace osgEarth;

Picker::Picker( osgViewer::View* view, osg::Node* root, unsigned travMask, float buffer, Limit limit ) :
_view    ( view ),
_root    ( root ),
_travMask( travMask ),
_buffer  ( buffer ),
_limit   ( limit )
{
    if ( root )
        _path = root->getParentalNodePaths()[0];
}

void
Picker::setLimit(const Picker::Limit& value)
{
    _limit = value;
}

void
Picker::setTraversalMask(unsigned value)
{
    _travMask = value;
}

void
Picker::setBuffer(float value)
{
    _buffer = value;
}

bool
Picker::pick( float x, float y, Hits& results ) const
{
    float local_x, local_y = 0.0;
    const osg::Camera* camera = _view->getCameraContainingPosition(x, y, local_x, local_y);
    if ( !camera )
        camera = _view->getCamera();

    osg::ref_ptr<osgEarth::PrimitiveIntersector> picker;

    double buffer_x = _buffer, buffer_y = _buffer;
    if ( camera->getViewport() )
    {
        double aspectRatio = camera->getViewport()->width()/camera->getViewport()->height();
        buffer_x *= aspectRatio;
        buffer_y /= aspectRatio;
    }

    osg::Matrix windowMatrix;

    if ( _root.valid() )
    {
        osg::Matrix modelMatrix;

        if (camera->getViewport())
        {
            windowMatrix = camera->getViewport()->computeWindowMatrix();
            modelMatrix.preMult( windowMatrix );
        }

        modelMatrix.preMult( camera->getProjectionMatrix() );
        modelMatrix.preMult( camera->getViewMatrix() );

        osg::NodePath prunedNodePath( _path.begin(), _path.end()-1 );
        modelMatrix.preMult( osg::computeWorldToLocal(prunedNodePath) );

        osg::Matrix modelInverse;
        modelInverse.invert(modelMatrix);

        osg::Vec3d startLocal(local_x, local_y, 0.0);
        osg::Vec3d startModel = startLocal * modelInverse;

        osg::Vec3d endLocal(local_x, local_y, 1.0);
        osg::Vec3d endModel = endLocal * modelInverse;

        osg::Vec3d bufferLocal(local_x + buffer_x, local_y + buffer_y, 0.0);
        osg::Vec3d bufferModel = bufferLocal * modelInverse;

        double buffer = osg::maximum((bufferModel - startModel).length(), 5.0);  //TODO: Setting a minimum of 4.0 may need revisited

        OE_DEBUG
            << "local_x:" << local_x << ", local_y:" << local_y
            << ", buffer_x:" << buffer_x << ", buffer_y:" << buffer_y
            << ", bm.x:" << bufferModel.x() << ", bm.y:" << bufferModel.y()
            << ", bm.z:" << bufferModel.z()
            << ", BUFFER: " << buffer
            << std::endl;

        picker = new osgEarth::PrimitiveIntersector(osgUtil::Intersector::MODEL, startModel, endModel, buffer);
    }
    else
    {
        picker = new osgEarth::PrimitiveIntersector(camera->getViewport() ? osgUtil::Intersector::WINDOW : osgUtil::Intersector::PROJECTION, local_x, local_y, _buffer);
    }

    //picker->setIntersectionLimit( (osgUtil::Intersector::IntersectionLimit)_limit );
    osgUtil::IntersectionVisitor iv(picker.get());

    // in MODEL mode, we need to window and proj matrixes in order to support some of the 
    // features in osgEarth (like Annotation::OrthoNode).
    if ( _root.valid() )
    {
        iv.pushWindowMatrix( new osg::RefMatrix(windowMatrix) );
        iv.pushProjectionMatrix( new osg::RefMatrix(camera->getProjectionMatrix()) );
        iv.pushViewMatrix( new osg::RefMatrix(camera->getViewMatrix()) );
    }

    iv.setTraversalMask( _travMask );

    if ( _root.valid() )
        _path.back()->accept(iv);
    else
        const_cast<osg::Camera*>(camera)->accept(iv);

    if (picker->containsIntersections())
    {
        results = picker->getIntersections();
        return true;
    }
    else
    {
        results.clear();
        return false;
    }
}
