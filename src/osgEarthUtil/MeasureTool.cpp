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

#include <osgEarthUtil/MeasureTool>
#include <osgEarth/GeoMath>

#include <osgEarthFeatures/Feature>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;




MeasureToolHandler::MeasureToolHandler( osg::Group* group, osgEarth::MapNode* mapNode ):
_mouseDown(false),
_group(group),
_gotFirstLocation(false),
_lastPointTemporary(false),
_finished(false),
_geoInterpolation( GEOINTERP_GREAT_CIRCLE ),
_mouseButton( osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON),
_isPath( false ),
_mapNode( mapNode ),
_intersectionMask(0xffffffff)
{
    LineString* line = new LineString();
    _feature = new Feature();
    _feature->setGeometry( line );
    _feature->geoInterp() = _geoInterpolation;    

    //Define a style for the line
    Style style;
    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = osg::Vec4f( 1,0,0,1 );
    ls->stroke()->width() = 2.0f;   

    _feature->style() = style;

    _featureNode = new FeatureNode( _mapNode.get(), _feature.get(), false);
    

    //Disable lighting and depth testing for the feature graph
    _featureNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _featureNode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    _group->addChild (_featureNode.get() );
}

MeasureToolHandler::~MeasureToolHandler()
{
    if (_group.valid()) _group->removeChild( _featureNode.get() );
}

void
MeasureToolHandler::setIsPath( bool path ) 
{
    if (_isPath != path)
    {
        _isPath = path;
        _finished = true;
        clear();                    
        _gotFirstLocation = false;
    }
}


GeoInterpolation
MeasureToolHandler::getGeoInterpolation() const
{
    return _geoInterpolation;
}

void
MeasureToolHandler::setGeoInterpolation( GeoInterpolation geoInterpolation )
{
    if (_geoInterpolation != geoInterpolation)
    {
        _geoInterpolation = geoInterpolation;
        _feature->geoInterp() = _geoInterpolation;
        _featureNode->init();
    }
}

void
MeasureToolHandler::setLineStyle( const Style& style )
{
     _feature->style() = style;
     _featureNode->init();
}


bool MeasureToolHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{    
    if ( ea.getHandled() )
    {
        return false;
    }

    
    osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());                
    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == _mouseButton)
    {        
        _mouseDown = true;
        _mouseDownX = ea.getX();
        _mouseDownY = ea.getY();
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == _mouseButton)
    {        
        float eps = 1.0f;
        _mouseDown = false;
        if (osg::equivalent(ea.getX(), _mouseDownX) && osg::equivalent(ea.getY(), _mouseDownY))
        {
            double lon, lat;
            if (getLocationAt(view, ea.getX(), ea.getY(), lon, lat))
            {
                if (!_gotFirstLocation)
                {
                    _finished = false;
                    clear();                    
                    _gotFirstLocation = true;
                    _feature->getGeometry()->push_back( osg::Vec3d( lon, lat, 0 ) );
                }
                else
                {
                    if (_lastPointTemporary)
                    {
                        _feature->getGeometry()->back() = osg::Vec3d( lon, lat, 0 );
                        _lastPointTemporary = false;
                    }
                    else
                    {                     
                        _feature->getGeometry()->push_back( osg::Vec3d( lon, lat, 0 ) );
                    }
                    _featureNode->init();
                    //_gotFirstLocation = false;
                    //_finished = true;
                    if (_finished || !_isPath) {
                        _gotFirstLocation = false;
                    }
                    fireDistanceChanged();
                }
            }
        }
    }  
    else if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK) {        
        if (_gotFirstLocation)
        {
            //_gotFirstLocation = false;
            _finished = true;     
            return true;
        }
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
    {
        if (_gotFirstLocation)
        {
            double lon, lat;
            if (getLocationAt(view, ea.getX(), ea.getY(), lon, lat))
            {
                if (!_lastPointTemporary)
                {
                    _feature->getGeometry()->push_back( osg::Vec3d( lon, lat, 0 ) );                 
                    _lastPointTemporary = true;
                }                        
                else
                {
                    _feature->getGeometry()->back() = osg::Vec3d( lon, lat, 0 );
                }
                _featureNode->init();
                fireDistanceChanged();
            }
        }
    }    
    return false;
}

bool MeasureToolHandler::getLocationAt(osgViewer::View* view, double x, double y, double &lon, double &lat)
{
    osgUtil::LineSegmentIntersector::Intersections results;            
    if ( view->computeIntersections( x, y, results, _intersectionMask ) )
    {
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
        osg::Vec3d point = first.getWorldIntersectPoint();

        double lat_rad, lon_rad, height;       
        _mapNode->getMap()->getProfile()->getSRS()->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, height );
        lat = osg::RadiansToDegrees( lat_rad );
        lon = osg::RadiansToDegrees( lon_rad );
        return true;
    }
    return false;
}

void MeasureToolHandler::clear()
{
    //Clear the locations    
    _feature->getGeometry()->clear();
    //_features->dirty();
    _featureNode->init();
    fireDistanceChanged();
}

void
MeasureToolHandler::setMouseButton(int mouseButton)
{
    _mouseButton = mouseButton;
}

int
MeasureToolHandler::getMouseButton() const
{
    return _mouseButton;
}


void MeasureToolHandler::addEventHandler(MeasureToolEventHandler* handler )
{
    _eventHandlers.push_back( handler );
}


void MeasureToolHandler::fireDistanceChanged()
{
    double distance = 0;
    if (_geoInterpolation == GEOINTERP_GREAT_CIRCLE)
    {
        distance = GeoMath::distance(_feature->getGeometry()->asVector());
    }
    else if (_geoInterpolation == GEOINTERP_RHUMB_LINE) 
    {
        distance = GeoMath::rhumbDistance(_feature->getGeometry()->asVector());
    }
    for (MeasureToolEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i)
    {
        i->get()->onDistanceChanged( this, distance );
    }
}