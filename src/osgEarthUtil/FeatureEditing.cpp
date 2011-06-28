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

#include <osgEarthUtil/FeatureEditing>

using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

/****************************************************************/
AddPointHandler::AddPointHandler(Feature* feature, FeatureListSource* source, const osgEarth::SpatialReference* mapSRS):
_feature(feature),
_source( source ),
_mapSRS( mapSRS ),
_mouseDown( false ),
_mouseButton( osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
{
}

void
AddPointHandler::setMouseButton( osgGA::GUIEventAdapter::MouseButtonMask mouseButton)
{
    _mouseButton = mouseButton;
}

osgGA::GUIEventAdapter::MouseButtonMask
AddPointHandler::getMouseButton() const
{
    return _mouseButton;
}

bool
AddPointHandler::addPoint( float x, float y, osgViewer::View* view )
{
    osgUtil::LineSegmentIntersector::Intersections results;
    if ( view->computeIntersections( x, y, results, 0x01 ) )
    {
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
        osg::Vec3d point = first.getWorldIntersectPoint();

        // transform it to map coordinates:
        double lat_rad, lon_rad, dummy;
        _mapSRS->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, dummy );

        double lat_deg = osg::RadiansToDegrees( lat_rad );
        double lon_deg = osg::RadiansToDegrees( lon_rad );

        if (_feature.valid())            
        {
            _feature->getGeometry()->push_back( osg::Vec3d(lon_deg, lat_deg, 0) );
            _source->dirty();
        }
        return true;
    }
    return false;
}

bool
AddPointHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
    if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        if (ea.getButton() == _mouseButton)
        {
            _mouseDown = true;
            return addPoint( ea.getX(), ea.getY(), view );
        }
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
    {
        if (ea.getButton() == _mouseButton)
        {
            _mouseDown = false;
        }
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE || ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
    {
        if (_mouseDown)
        {
            return addPoint( ea.getX(), ea.getY(), view );
        }
    }

    return false;
}

/****************************************************************/