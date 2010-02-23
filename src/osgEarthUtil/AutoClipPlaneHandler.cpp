/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthUtil/AutoClipPlaneHandler>

using namespace osgEarthUtil;
using namespace osgEarth;

AutoClipPlaneHandler::AutoClipPlaneHandler( MapNode* node ) :
_node(node),
_frame(0),
_nfrAtRadius( 0.00001 ),
_nfrAtDoubleRadius( 0.0049 ),
_rp( -1 )
{
    //NOP
}

bool 
AutoClipPlaneHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    if ( ea.getEventType() == osgGA::GUIEventAdapter::FRAME && _frame++ > 1 )
    {
        if ( _node.valid() )
        {
            if ( _node->getMap()->getProfile() )
            {
                _rp = _node->getMap()->getProfile()->getSRS()->getEllipsoid()->getRadiusPolar();
                _node = 0L;
            }
        }
        else
        {
            osg::Camera* cam = aa.asView()->getCamera();
            cam->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

            osg::Vec3d eye, center, up;
            cam->getViewMatrixAsLookAt( eye, center, up );

            double d = eye.length();

            if ( d > _rp )
            {
                double fovy, ar, znear, zfar;
                cam->getProjectionMatrixAsPerspective( fovy, ar, znear, zfar );

                // far clip at the horizon:
                zfar = sqrt( d*d - _rp*_rp );

                double nfr = _nfrAtRadius + _nfrAtDoubleRadius * ((d-_rp)/d);
                znear = osg::clampAbove( zfar * nfr, 1.0 );

                cam->setProjectionMatrixAsPerspective( fovy, ar, znear, zfar );

                //osg::notify(osg::NOTICE) << fixed
                //    << "near=" << znear << ", far=" << zfar << std::endl;
            }
        }
    }
    return false;
}

