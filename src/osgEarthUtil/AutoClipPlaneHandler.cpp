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
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarth/FindNode>

using namespace osgEarth::Util;
using namespace osgEarth;

AutoClipPlaneHandler::AutoClipPlaneHandler( const Map* map ) :
_geocentric(false),
_frame(0),
_nfrAtRadius( 0.00001 ),
_nfrAtDoubleRadius( 0.0049 ),
_rp( -1 ),
_autoFarPlaneClipping(true)
{
    //NOP
    if ( map )
    {
        _geocentric = map->isGeocentric();
        if ( _geocentric )
            _rp = map->getProfile()->getSRS()->getEllipsoid()->getRadiusPolar();
    }
}

bool 
AutoClipPlaneHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    if ( ea.getEventType() == osgGA::GUIEventAdapter::FRAME && _frame++ > 1 )
    {
        frame( aa );
    }
    return false;
}

void
AutoClipPlaneHandler::frame( osgGA::GUIActionAdapter& aa )
{
    osg::Camera* cam = aa.asView()->getCamera();

    if ( _rp < 0 )
    {
        osg::ref_ptr<MapNode> tempNode = osgEarth::findTopMostNodeOfType<MapNode>( cam );
        if ( tempNode.valid() && tempNode->getMap()->getProfile() )
        {
            _geocentric = tempNode->getMap()->isGeocentric();
            if ( _geocentric )
                _rp = tempNode->getMap()->getProfile()->getSRS()->getEllipsoid()->getRadiusPolar();
            else
                OE_INFO << "[AutoClipPlaneHandler] disabled for non-geocentric map" << std::endl;

            //_mapNode = tempNode.get();
        }
    }

    if ( _rp > 0 && _geocentric ) // _mapNode.valid() && _geocentric )
    {
        cam->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );

        osg::Vec3d eye, center, up;
        cam->getViewMatrixAsLookAt( eye, center, up );

        double d = eye.length();

        if ( d < _rp )
            d = _rp;

        if ( d > _rp )
        {
            double fovy, ar, znear, zfar, finalZfar;
            cam->getProjectionMatrixAsPerspective( fovy, ar, znear, finalZfar );

            // far clip at the horizon:
            zfar = sqrt( d*d - _rp*_rp );

            if (_autoFarPlaneClipping)
            {
                finalZfar = zfar;
            }

            double nfr = _nfrAtRadius + _nfrAtDoubleRadius * ((d-_rp)/d);
            znear = osg::clampAbove( zfar * nfr, 1.0 );

            cam->setProjectionMatrixAsPerspective( fovy, ar, znear, finalZfar );

            //OE_NOTICE << fixed
            //    << "near=" << znear << ", far=" << zfar << std::endl;
        }
    }
}

