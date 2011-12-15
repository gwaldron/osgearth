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
#include <osgEarthUtil/PickingUtils>
#include <osgEarthUtil/DPLineSegmentIntersector>

#define LC "[Picker] "

using namespace osgEarth::Util;

Picker::Picker( osgViewer::View* view, osg::Node* root, unsigned travMask ) :
_view    ( view ),
_root    ( root ),
_travMask( travMask )
{
    if ( root )
        _path.push_back( root );
}

bool
Picker::pick( float x, float y, Hits& results ) const
{
    float local_x, local_y = 0.0;
    const osg::Camera* camera = _view->getCameraContainingPosition(x, y, local_x, local_y);
    if ( !camera )
        camera = _view->getCamera();

    osg::Vec3d startVertex, endVertex;
    osg::ref_ptr<osgUtil::LineSegmentIntersector> picker;

    if ( _root.valid() )
    {
        osg::Matrixd matrix;
        if ( _path.size() > 1 )
        {
            osg::NodePath prunedNodePath( _path.begin(), _path.end()-1 );
            matrix = osg::computeLocalToWorld( prunedNodePath );
        }

        matrix.postMult( camera->getViewMatrix() );
        matrix.postMult( camera->getProjectionMatrix() );

        double zNear = -1.0;
        double zFar = 1.0;
        if (camera->getViewport())
        {
            matrix.postMult(camera->getViewport()->computeWindowMatrix());
            zNear = 0.0;
            zFar = 1.0;
        }

        osg::Matrixd inverse;
        inverse.invert(matrix);

        osg::Vec3d startVertex = osg::Vec3d(local_x,local_y,zNear) * inverse;
        osg::Vec3d endVertex = osg::Vec3d(local_x,local_y,zFar) * inverse;
        
        picker = new DPLineSegmentIntersector(osgUtil::Intersector::MODEL, startVertex, endVertex);
    }

    else
    {
        osgUtil::LineSegmentIntersector::CoordinateFrame cf = camera->getViewport() ? osgUtil::Intersector::WINDOW : osgUtil::Intersector::PROJECTION;
        picker = new osgUtil::LineSegmentIntersector(cf, local_x, local_y);
    }

    osgUtil::IntersectionVisitor iv(picker.get());
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
