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

using namespace osgEarth::Util;
using namespace osgEarth::Symbology;
using namespace osgEarth::Features;




MeasureToolHandler::MeasureToolHandler( osg::Group* group, osgEarth::MapNode* mapNode ):
_mouseDown(false),
_group(group),
_gotFirstLocation(false),
_lastPointTemporary(false),
_finished(false),
_mode( MODE_GREATCIRCLE ),
_mouseButton( osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON),
_isPath( false ),
_mapNode( mapNode ),
_intersectionMask(0xffffffff)
{
    //Initialize the feature source, which is just a single feature that we will use to draw the measuring line
    _features = new FeatureListSource();
    LineString* line = new LineString();
    _feature = new Feature(0);
    _feature->setGeometry( line );
    _features->insertFeature( _feature.get() );

    //Define a style for the line
    Style style;
    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = osg::Vec4f( 1,0,0,1 );
    ls->stroke()->width() = 2.0f;   
    StyleSheet styleSheet;
    styleSheet.addStyle( style );


    _factory = new GeomFeatureNodeFactory();
    _factory->_options.resampleMode() = ResampleFilter::RESAMPLE_GREATCIRCLE;
    //2 degrees in meters
    _factory->_options.resampleMaxLength() = 111319.0 * 2.0;


    //Initialize the FeatureModelGraph which will actually display our features
    _featureGraph = new FeatureModelGraph( 
        _features.get(), 
        FeatureModelSourceOptions(), 
        _factory.get(),
        styleSheet,
        new Session( _mapNode->getMap() ) );    

    //Disable lighting and depth testing for the feature graph
    _featureGraph->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _featureGraph->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    _group->addChild (_featureGraph.get() );
}

MeasureToolHandler::~MeasureToolHandler()
{
    if (_group.valid()) _group->removeChild( _featureGraph.get() );
}

MeasureToolHandler::Mode
MeasureToolHandler::getMode() const
{
    return _mode;
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


void
MeasureToolHandler::setMode( Mode mode )
{
    if (_mode != mode) {
        _mode = mode;
        if (mode == MODE_GREATCIRCLE)
        {
            _factory->_options.resampleMode() = ResampleFilter::RESAMPLE_GREATCIRCLE;
        }
        else if (mode == MODE_RHUMB)
        {
               _factory->_options.resampleMode() = ResampleFilter::RESAMPLE_RHUMB;
        }
        _featureGraph->dirty();
    }
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
                    _features->dirty();
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
                _features->dirty();
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
    _features->dirty();
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

static double getGreatCircleDistance( osgEarth::Symbology::Geometry* geometry )
{
    double length = 0;
    
    if (geometry && geometry->size() > 1)
    {
        for (unsigned int i = 0; i < geometry->size()-1; ++i)
        {
            const osg::Vec3d& current = (*geometry)[i];
            const osg::Vec3d& next    = (*geometry)[i+1];
            length += GeoMath::distance(osg::DegreesToRadians(current.y()), osg::DegreesToRadians(current.x()),
            osg::DegreesToRadians(next.y()), osg::DegreesToRadians(next.x()));
        }
    }
    return length;
}

static double
getRhumbDistance( osgEarth::Symbology::Geometry* geometry)
{
    double length = 0;
    if (geometry && geometry->size() > 1)
    {
        for (unsigned int i = 0; i < geometry->size()-1; ++i)
        {
            const osg::Vec3d& current = (*geometry)[i];
            const osg::Vec3d& next    = (*geometry)[i+1];
            length += GeoMath::rhumbDistance(osg::DegreesToRadians(current.y()), osg::DegreesToRadians(current.x()),
                                             osg::DegreesToRadians(next.y()), osg::DegreesToRadians(next.x()));                                             
        }
    }
    return length;
}

void MeasureToolHandler::fireDistanceChanged()
{
    double distance = 0;
    if (_mode == MODE_GREATCIRCLE)
    {
        distance = getGreatCircleDistance( _feature->getGeometry() );
    }
    else if (_mode == MODE_RHUMB) 
    {
        distance = getRhumbDistance( _feature->getGeometry() );
    }
    for (MeasureToolEventHandlerList::const_iterator i = _eventHandlers.begin(); i != _eventHandlers.end(); ++i)
    {
        i->get()->onDistanceChanged( this, distance );
    }
}