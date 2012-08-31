/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgUtil/PolytopeIntersector>
#include <osg/Polytope>

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

bool
Picker::pick( float x, float y, Hits& results ) const
{
    float local_x, local_y = 0.0;
    const osg::Camera* camera = _view->getCameraContainingPosition(x, y, local_x, local_y);
    if ( !camera )
        camera = _view->getCamera();

    osg::ref_ptr<osgUtil::PolytopeIntersector> picker;

    double buffer_x = _buffer, buffer_y = _buffer;
    if ( camera->getViewport() )
    {
        double aspectRatio = camera->getViewport()->width()/camera->getViewport()->height();
        buffer_x *= aspectRatio;
        buffer_y /= aspectRatio;
    }
    
    double zNear = 0.00001;
    double zFar  = 1.0;

    double xMin = local_x - buffer_x;
    double xMax = local_x + buffer_x;
    double yMin = local_y - buffer_y;
    double yMax = local_y + buffer_y;

    osg::Polytope winPT;
    winPT.add(osg::Plane( 1.0, 0.0, 0.0, -xMin));
    winPT.add(osg::Plane(-1.0, 0.0 ,0.0,  xMax));
    winPT.add(osg::Plane( 0.0, 1.0, 0.0, -yMin));
    winPT.add(osg::Plane( 0.0,-1.0, 0.0,  yMax));
    winPT.add(osg::Plane( 0.0, 0.0, 1.0, zNear));

    osg::Matrix windowMatrix;

    if ( _root.valid() )
    {
        osg::Matrix matrix;

        if (camera->getViewport())
        {
            windowMatrix = camera->getViewport()->computeWindowMatrix();
            matrix.preMult( windowMatrix );
            zNear = 0.0;
            zFar = 1.0;
        }

        matrix.preMult( camera->getProjectionMatrix() );
        matrix.preMult( camera->getViewMatrix() );

        osg::NodePath prunedNodePath( _path.begin(), _path.end()-1 );
        matrix.preMult( osg::computeWorldToLocal(prunedNodePath) );

        osg::Polytope transformedPT;
        transformedPT.setAndTransformProvidingInverse( winPT, matrix );
        
        picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::MODEL, transformedPT);
    }

    else
    {
        osgUtil::Intersector::CoordinateFrame cf = camera->getViewport() ? osgUtil::Intersector::WINDOW : osgUtil::Intersector::PROJECTION;
        picker = new osgUtil::PolytopeIntersector(cf, winPT);
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
