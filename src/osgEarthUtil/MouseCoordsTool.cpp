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

#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::Util;

//-----------------------------------------------------------------------

MouseCoordsTool::MouseCoordsTool( MapNode* mapNode, LabelControl* label, Formatter* formatter ) :
_mapNode( mapNode )
{
    _mapNodePath.push_back( mapNode->getTerrainEngine() );

    if ( label )
    {
        addCallback( new MouseCoordsLabelCallback(label, formatter) );
    }
}

void
MouseCoordsTool::addCallback( MouseCoordsTool::Callback* cb )
{
    _callbacks.push_back( cb );
}

bool
MouseCoordsTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    if (ea.getEventType() == ea.MOVE || ea.getEventType() == ea.DRAG)
    {
        osg::Vec3d world;
        if ( _mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world) )
        {
            GeoPoint map;
            map.fromWorld( _mapNode->getMapSRS(), world );

            for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                i->get()->set( map, aa.asView(), _mapNode );
        }
        else
        {
            for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                i->get()->reset( aa.asView(), _mapNode );
        }
    }

    return false;
}

//-----------------------------------------------------------------------

MouseCoordsLabelCallback::MouseCoordsLabelCallback( LabelControl* label, Formatter* formatter ) :
_label    ( label ),
_formatter( formatter )
{
    //nop
}

void
MouseCoordsLabelCallback::set( const GeoPoint& mapCoords, osg::View* view, MapNode* mapNode )
{
    if ( _label.valid() )
    {
        osg::Vec3d eye, center, up;
        view->getCamera()->getViewMatrixAsLookAt(eye, center, up);
        osg::Vec3d world;
        mapCoords.toWorld(world);
        double range = (eye-world).length();

        if ( _formatter )
        {
            _label->setText( Stringify()
                <<  _formatter->format( mapCoords )
                << ", " << mapCoords.z() 
                << "; RNG:" << range
                << "  |  "
                << mapCoords.getSRS()->getName() );
        }
        else
        {
            _label->setText( Stringify()
                << std::fixed
                << mapCoords.x()
                << ", " << mapCoords.y()
                << ", " << mapCoords.z()
                << "; RNG:" << range
                << "  |  "
                << mapCoords.getSRS()->getName() );
        }
    }
}

void
MouseCoordsLabelCallback::reset( osg::View* view, MapNode* mapNode )
{
    if ( _label.valid() )
    {
        _label->setText( "" );
        _label->setText(Stringify() << "No data  |  " << mapNode->getMapSRS()->getName() );
    }
}

//-----------------------------------------------------------------------
