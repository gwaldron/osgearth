/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/MeasureTool>
#include <osgEarth/GLUtils>
#include <osgEarth/GeoMath>

#define LC "[MeasureTool] "

using namespace osgEarth;
using namespace osgEarth::Contrib;

//#define SHOW_EXTENT 1


MeasureToolHandler::MeasureToolHandler( osgEarth::MapNode* mapNode ):
_mouseDown         (false),
_gotFirstLocation  (false),
_lastPointTemporary(false),
_finished          (false),
_geoInterpolation  (GEOINTERP_GREAT_CIRCLE),
_mouseButton       (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON),
_isPath            (false),
_intersectionMask  (0xffffffff),
_mouseDownX        (0.0f),
_mouseDownY        (0.0f)
{
    _root = new osg::Group();
    setMapNode( mapNode );
}

MeasureToolHandler::~MeasureToolHandler()
{
    setMapNode( 0L );
}


void
MeasureToolHandler::setMapNode( MapNode* mapNode )
{
    MapNode* oldMapNode = getMapNode();

    if ( oldMapNode != mapNode )
    {
        if (oldMapNode)
        {
            oldMapNode->removeChild(_root.get());
        }

        _mapNode = mapNode;

        if (mapNode)
        {
            mapNode->addChild(_root.get());
        }

        rebuild();
    }
}


void
MeasureToolHandler::rebuild()
{
    if ( _featureNode.valid() )
    {
        _root->removeChild( _featureNode.get() );
        _featureNode = 0L;
    }

    if ( !getMapNode() )
        return;

    if ( getMapNode()->getMapSRS()->isProjected() )
    {
        OE_WARN << LC << "Sorry, MeasureTool does not yet support projected maps" << std::endl;
        return;
    }


    // Define the path feature:
    _feature = new Feature(new LineString(), getMapNode()->getMapSRS());
    _feature->geoInterp() = _geoInterpolation;

    // clamp to the terrain skin as it pages in
    AltitudeSymbol* alt = _feature->getOrCreateStyle().getOrCreate<AltitudeSymbol>();
    alt->clamping() = alt->CLAMP_TO_TERRAIN;
    alt->technique() = alt->TECHNIQUE_GPU;

    // offset to mitigate Z fighting
    RenderSymbol* render = _feature->getOrCreateStyle().getOrCreate<RenderSymbol>();
    render->depthOffset().mutable_value().automatic() = true;

    // define a style for the line
    LineSymbol* ls = _feature->getOrCreateStyle().getOrCreate<LineSymbol>();
    ls->stroke().mutable_value().color() = Color::Yellow;
    ls->stroke().mutable_value().width() = Distance(2.0f, Units::PIXELS);
    ls->tessellation() = 150;

    _featureNode = new FeatureNode( _feature.get() );
    _featureNode->setMapNode(getMapNode());

    GLUtils::setLighting(_featureNode->getOrCreateStateSet(), osg::StateAttribute::OFF);

    _root->addChild (_featureNode.get() );

#ifdef SHOW_EXTENT

    // Define the extent feature:
    _extentFeature = new Feature( new osgEarth::Polygon(), getMapNode()->getMapSRS() );
    _extentFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
    _extentFeature->getOrCreateStyle()->add( alt );
    LineSymbol* extentLine = _extentFeature->style()->getOrCreate<LineSymbol>();
    extentLine->stroke()->color() = Color::Cyan;
    extentLine->stroke()->width() = 2.0f;
    extentLine->tessellation() = 20;

    _extentFeatureNode = new FeatureNode( getMapNode(), _extentFeature.get() );
    
    _root->addChild( _extentFeatureNode.get() );
#endif
}

bool
MeasureToolHandler::getIsPath() const
{
    return _isPath;
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
        _featureNode->dirty();
        fireDistanceChanged();
    }
}

void
MeasureToolHandler::setLineStyle( const Style& style )
{
    _feature->setStyle(style);
    _featureNode->dirty();
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
        if (osg::equivalent(ea.getX(), _mouseDownX, eps) && osg::equivalent(ea.getY(), _mouseDownY, eps))
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
                    _featureNode->dirty();

                    //_gotFirstLocation = false;
                    //_finished = true;
                    if (_finished || !_isPath) {
                        _gotFirstLocation = false;
                    }

#ifdef SHOW_EXTENT
                    const GeoExtent ex( _feature->getSRS(), _feature->getGeometry()->getBounds() );
                    //OE_INFO << "extent = " << ex.toString() << std::endl;
                    Geometry* eg = _extentFeature->getGeometry();
                    osg::Vec3d fc = ex.getCentroid();
                    eg->clear();
                    eg->push_back( ex.west(), ex.south() );
                    if ( ex.width() >= 180.0 )
                        eg->push_back( fc.x(), ex.south() );
                    eg->push_back( ex.east(), ex.south() );
                    eg->push_back( ex.east(), ex.north() );
                    if ( ex.width() >= 180.0 )
                        eg->push_back( fc.x(), ex.north() );
                    eg->push_back( ex.west(), ex.north() );
                    _extentFeatureNode->init();
#endif

                    fireDistanceChanged();
                    aa.requestRedraw();
                }
            }
        }
    }  
    else if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK) {        
        if (_gotFirstLocation)
        {
            //_gotFirstLocation = false;
            _finished = true;    
            aa.requestRedraw(); 
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
                _featureNode->dirty();
                fireDistanceChanged();
                aa.requestRedraw();
            }
        }
    }    
    return false;
}

bool MeasureToolHandler::getLocationAt(osgViewer::View* view, double x, double y, double &lon, double &lat)
{
    osgUtil::LineSegmentIntersector::Intersections results;            
    if ( getMapNode() &&  view->computeIntersections( x, y, results, _intersectionMask ) )
    {
        // find the first hit under the mouse:
        osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
        osg::Vec3d point = first.getWorldIntersectPoint();

        osg::Vec3d lon_lat_h =
            getMapNode()->getMap()->getProfile()->getSRS()->getEllipsoid().geocentricToGeodetic(point);

        lat = lon_lat_h.y();
        lon = lon_lat_h.x();
        return true;
    }
    return false;
}

void MeasureToolHandler::clear()
{
    //Clear the locations    
    _feature->getGeometry()->clear();
    _featureNode->dirty();

#ifdef SHOW_EXTENT
    _extentFeature->getGeometry()->clear();
    _extentFeatureNode->init();
#endif

    fireDistanceChanged();

    _gotFirstLocation = false; 
    _lastPointTemporary = false; 
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
