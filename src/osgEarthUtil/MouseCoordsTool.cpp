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

#include <osgEarthUtil/MouseCoordsTool>
#include <osgViewer/View>

using namespace osgEarth;
using namespace osgEarth::Util;

//-----------------------------------------------------------------------

MouseCoordsTool::MouseCoordsTool( MapNode* mapNode ) :
_mapNode( mapNode )
{
    _mapNodePath.push_back( mapNode->getTerrainEngine() );
}

void
MouseCoordsTool::addCallback( MouseCoordsTool::Callback* cb )
{
    _callbacks.push_back( cb );
}

bool
MouseCoordsTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
    if (ea.getEventType() == ea.MOVE || ea.getEventType() == ea.DRAG)
    {
        osgUtil::LineSegmentIntersector::Intersections results;
        if ( view->computeIntersections( ea.getX(), ea.getY(), _mapNodePath, results ) )
        {
            // find the first hit under the mouse:
            osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
            osg::Vec3d world = first.getWorldIntersectPoint();
            osg::Vec3d map;

            // transform it to map coordinates:
            _mapNode->getMap()->worldPointToMapPoint(world, map);

            for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                i->get()->set( map, view, _mapNode );
        }
        else
        {
            for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                i->get()->reset( view, _mapNode );
        }
    }

    return false;
}

//-----------------------------------------------------------------------

MouseCoordsLabelCallback::MouseCoordsLabelCallback( LabelControl* label, Formatter* formatter ) :
_label    ( label ),
_formatter( formatter )
{
    if ( !formatter )
        _formatter = new LatLongFormatter();
}

void
MouseCoordsLabelCallback::set( const osg::Vec3d& mapCoords, osg::View* view, MapNode* mapNode )
{
    if ( _label.valid() )
    {
        _label->setText( _formatter->format(mapCoords, mapNode->getMapSRS()) );
    }
}

void
MouseCoordsLabelCallback::reset( osg::View* view, MapNode* mapNode )
{
    if ( _label.valid() )
    {
        _label->setText( "" );
    }
}

//-----------------------------------------------------------------------
