/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/RectangleNode>
#include <osgEarth/AnnotationRegistry>
#include <osgEarth/GeometryFactory>
#include <osgEarth/MapNode>
#include <osgEarth/GeoMath>

using namespace osgEarth;

RectangleNode::RectangleNode() :
    LocalGeometryNode()
{
    construct();

}

RectangleNode::RectangleNode(const GeoPoint&   position,
                             const Linear&     width,
                             const Linear&     height,
                             const Style&      style) :
    LocalGeometryNode()
{
    construct();

    _width = width;
    _height = height;
    _style = style;

    setPosition(position);

    buildGeometry();
}

void
RectangleNode::construct()
{
    _width.set(1.0, Units::KILOMETERS);
    _height.set(1.0, Units::KILOMETERS);
}

const Linear&
RectangleNode::getWidth() const
{
    return _width;
}

const Linear&
RectangleNode::getHeight() const
{
    return _height;
}

void
RectangleNode::setWidth( const Linear& width )
{
    setSize( width, _height );
}

void
RectangleNode::setHeight( const Linear& height )
{
    setSize( _width, height );
}

void
RectangleNode::setSize( const Linear& width, const Linear& height)
{
    if (_width != width || _height != height)
    {
        _width = width;
        _height = height;
        buildGeometry();
    }
}

const Style&
RectangleNode::getStyle() const
{
    return _style;
}

void
RectangleNode::setStyle( const Style& style )
{
    _style = style;
    buildGeometry();
}


GeoPoint
RectangleNode::getUpperLeft() const
{
    return getCorner( CORNER_UPPER_LEFT );
}

void
RectangleNode::setUpperLeft( const GeoPoint& upperLeft )
{
    GeoPoint center = getPosition();

    //Figure out the new width and height
    double earthRadius = center.getSRS()->getEllipsoid().getRadiusEquator();

    double lat = osg::DegreesToRadians(center.y());
    double lon = osg::DegreesToRadians(center.x());
    double halfWidthMeters  =  _width.as(Units::METERS) / 2.0;
    double halfHeightMeters  = _height.as(Units::METERS) / 2.0;   

    double eastLon, eastLat;
    double westLon, westLat;
    double northLon, northLat;
    double southLon, southLat;

    //Get the current corners
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 90.0 ), halfWidthMeters, eastLat, eastLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( -90.0 ), halfWidthMeters, westLat, westLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 0.0 ),  halfHeightMeters, northLat, northLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 180.0 ), halfHeightMeters, southLat, southLon, earthRadius );

    if (osg::DegreesToRadians(upperLeft.x()) < eastLon && osg::DegreesToRadians(upperLeft.y()) > southLat)
    {
        westLon = osg::DegreesToRadians(upperLeft.x());
        northLat = osg::DegreesToRadians(upperLeft.y());

        double x = ( eastLon + westLon ) / 2.0;
        double y = ( southLat + northLat) / 2.0;
        setPosition(GeoPoint( center.getSRS(), osg::RadiansToDegrees(x), osg::RadiansToDegrees(y)));

        double width =  GeoMath::distance( y, westLon, y, eastLon, earthRadius);
        double height =  GeoMath::distance( southLat, x, northLat, x, earthRadius);
        setWidth(  Linear(width,  Units::METERS ));
        setHeight( Linear(height, Units::METERS ));
    }
}

GeoPoint
RectangleNode::getUpperRight() const
{
    return getCorner( CORNER_UPPER_RIGHT );
}

void
RectangleNode::setUpperRight( const GeoPoint& upperRight )
{
    GeoPoint center = getPosition();

    //Figure out the new width and height
    double earthRadius = center.getSRS()->getEllipsoid().getRadiusEquator();

    double lat = osg::DegreesToRadians(center.y());
    double lon = osg::DegreesToRadians(center.x());
    double halfWidthMeters  =  _width.as(Units::METERS) / 2.0;
    double halfHeightMeters  = _height.as(Units::METERS) / 2.0;   

    double eastLon, eastLat;
    double westLon, westLat;
    double northLon, northLat;
    double southLon, southLat;

    //Get the current corners
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 90.0 ), halfWidthMeters, eastLat, eastLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( -90.0 ), halfWidthMeters, westLat, westLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 0.0 ),  halfHeightMeters, northLat, northLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 180.0 ), halfHeightMeters, southLat, southLon, earthRadius );

    if (osg::DegreesToRadians(upperRight.x()) > westLon && osg::DegreesToRadians(upperRight.y()) > southLat)
    {
        eastLon = osg::DegreesToRadians(upperRight.x());
        northLat = osg::DegreesToRadians(upperRight.y());

        double x = ( eastLon + westLon ) / 2.0;
        double y = ( southLat + northLat) / 2.0;
        setPosition(GeoPoint( center.getSRS(), osg::RadiansToDegrees(x), osg::RadiansToDegrees(y)));

        double width =  GeoMath::distance( y, westLon, y, eastLon, earthRadius);
        double height =  GeoMath::distance( southLat, x, northLat, x, earthRadius);
        setWidth(  Linear(width,  Units::METERS ));
        setHeight( Linear(height, Units::METERS ));
    }
}

GeoPoint
RectangleNode::getLowerLeft() const
{
    return getCorner( CORNER_LOWER_LEFT );
}

void
RectangleNode::setLowerLeft( const GeoPoint& lowerLeft )
{
    GeoPoint center = getPosition();

    //Figure out the new width and height
    double earthRadius = center.getSRS()->getEllipsoid().getRadiusEquator();

    double lat = osg::DegreesToRadians(center.y());
    double lon = osg::DegreesToRadians(center.x());
    double halfWidthMeters  =  _width.as(Units::METERS) / 2.0;
    double halfHeightMeters  = _height.as(Units::METERS) / 2.0;   

    double eastLon, eastLat;
    double westLon, westLat;
    double northLon, northLat;
    double southLon, southLat;

    //Get the current corners
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 90.0 ), halfWidthMeters, eastLat, eastLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( -90.0 ), halfWidthMeters, westLat, westLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 0.0 ),  halfHeightMeters, northLat, northLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 180.0 ), halfHeightMeters, southLat, southLon, earthRadius );

    if (osg::DegreesToRadians(lowerLeft.x()) < eastLon && osg::DegreesToRadians(lowerLeft.y()) < northLat)
    {
        westLon = osg::DegreesToRadians(lowerLeft.x());  
        southLat = osg::DegreesToRadians(lowerLeft.y());

        double x = ( eastLon + westLon ) / 2.0;
        double y = ( southLat + northLat) / 2.0;
        setPosition(GeoPoint( center.getSRS(), osg::RadiansToDegrees(x), osg::RadiansToDegrees(y)));

        double width =  GeoMath::distance( y, westLon, y, eastLon, earthRadius);
        double height =  GeoMath::distance( southLat, x, northLat, x, earthRadius);
        setWidth(  Linear(width,  Units::METERS ));
        setHeight( Linear(height, Units::METERS ));
    }
}

GeoPoint
RectangleNode::getLowerRight() const
{
    return getCorner( CORNER_LOWER_RIGHT );
}

void
RectangleNode::setLowerRight( const GeoPoint& lowerRight )
{
    GeoPoint center = getPosition();

    //Figure out the new width and height
    double earthRadius = center.getSRS()->getEllipsoid().getRadiusEquator();
    
    double lat = osg::DegreesToRadians(center.y());
    double lon = osg::DegreesToRadians(center.x());
    double halfWidthMeters  =  _width.as(Units::METERS) / 2.0;
    double halfHeightMeters  = _height.as(Units::METERS) / 2.0;   

    double eastLon, eastLat;
    double westLon, westLat;
    double northLon, northLat;
    double southLon, southLat;
    
    //Get the current corners
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 90.0 ), halfWidthMeters, eastLat, eastLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( -90.0 ), halfWidthMeters, westLat, westLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 0.0 ),  halfHeightMeters, northLat, northLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 180.0 ), halfHeightMeters, southLat, southLon, earthRadius );
    
    if (osg::DegreesToRadians(lowerRight.x()) > westLon && osg::DegreesToRadians(lowerRight.y()) < northLat) {
        eastLon = osg::DegreesToRadians(lowerRight.x());
        southLat = osg::DegreesToRadians(lowerRight.y());

        double x = ( eastLon + westLon ) / 2.0;
        double y = ( southLat + northLat) / 2.0;
        setPosition(GeoPoint( center.getSRS(), osg::RadiansToDegrees(x), osg::RadiansToDegrees(y)));

        double width =  GeoMath::distance( y, westLon, y, eastLon, earthRadius);
        double height =  GeoMath::distance( southLat, x, northLat, x, earthRadius);
        setWidth(  Linear(width,  Units::METERS ));
        setHeight( Linear(height, Units::METERS ));
    }
}

GeoPoint
RectangleNode::getCorner( Corner corner ) const
{
    GeoPoint center = getPosition();

    double earthRadius = center.getSRS()->getEllipsoid().getRadiusEquator();
    double lat = osg::DegreesToRadians(center.y());
    double lon = osg::DegreesToRadians(center.x());
    double halfWidthMeters  =  _width.as(Units::METERS) / 2.0;
    double halfHeightMeters  = _height.as(Units::METERS) / 2.0;   

    double eastLon, eastLat;
    double westLon, westLat;
    double northLon, northLat;
    double southLon, southLat;
        
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 90.0 ), halfWidthMeters, eastLat, eastLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( -90.0 ), halfWidthMeters, westLat, westLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 0.0 ),  halfHeightMeters, northLat, northLon, earthRadius );
    GeoMath::destination( lat, lon, osg::DegreesToRadians( 180.0 ), halfHeightMeters, southLat, southLon, earthRadius );

    if (corner == CORNER_LOWER_LEFT)
    {
        return GeoPoint(center.getSRS(), osg::RadiansToDegrees(westLon), osg::RadiansToDegrees(southLat), 0, ALTMODE_RELATIVE);
    }
    else if (corner == CORNER_LOWER_RIGHT)
    {
        return GeoPoint(center.getSRS(), osg::RadiansToDegrees(eastLon), osg::RadiansToDegrees(southLat), 0, ALTMODE_RELATIVE);
    }
    else if (corner == CORNER_UPPER_LEFT)
    {
        return GeoPoint(center.getSRS(), osg::RadiansToDegrees(westLon), osg::RadiansToDegrees(northLat), 0, ALTMODE_RELATIVE);
    }
    else if (corner == CORNER_UPPER_RIGHT)
    {
        return GeoPoint(center.getSRS(), osg::RadiansToDegrees(eastLon), osg::RadiansToDegrees(northLat), 0, ALTMODE_RELATIVE);
    }
    return GeoPoint();
}

void
RectangleNode::setCorner( Corner corner, const GeoPoint& location)
{
    if (corner == CORNER_LOWER_LEFT) setLowerLeft( location );
    else if (corner == CORNER_LOWER_RIGHT) setLowerRight( location );
    else if (corner == CORNER_UPPER_LEFT) setUpperLeft( location );
    else if (corner == CORNER_UPPER_RIGHT) setUpperRight( location );
}


void
RectangleNode::buildGeometry()
{ 
    // construct a local-origin circle.
    GeometryFactory factory;    
    osg::ref_ptr<Geometry> geom = factory.createRectangle(osg::Vec3d(0,0,0), _width, _height);
    if (geom.valid())
    {
        setGeometry(geom.get());
    }
}



//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( rectangle, osgEarth::RectangleNode );


RectangleNode::RectangleNode(const Config&         conf,
                             const osgDB::Options* dbOptions) :
    LocalGeometryNode(conf, dbOptions)
{
    construct();

    conf.get( "width", _width );
    conf.get( "height", _height );
    conf.get( "style",  _style );

    buildGeometry();
}

Config
RectangleNode::getConfig() const
{
    Config conf = LocalGeometryNode::getConfig();
    conf.key() = "rectangle";

    conf.set( "width",  _width );
    conf.set( "height", _height );
    conf.set( "style",  _style );

    return conf;
}
