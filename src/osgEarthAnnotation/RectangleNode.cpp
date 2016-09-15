/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarthAnnotation/RectangleNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarthSymbology/ExtrusionSymbol>
#include <osgEarth/MapNode>
#include <osgEarth/DrapeableNode>
#include <osg/MatrixTransform>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


RectangleNode::RectangleNode(MapNode*          mapNode,
                             const GeoPoint&   position,
                             const Linear&     width,
                             const Linear&     height,
                             const Style&      style) :
GeoPositionNode    ( mapNode, position ),
_width       ( width ),
_height      ( height ),
_style       ( style )
{
    rebuild();
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
        rebuild();
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
    rebuild();
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
    double earthRadius = center.getSRS()->getEllipsoid()->getRadiusEquator();

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
    double earthRadius = center.getSRS()->getEllipsoid()->getRadiusEquator();

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
    double earthRadius = center.getSRS()->getEllipsoid()->getRadiusEquator();

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
    double earthRadius = center.getSRS()->getEllipsoid()->getRadiusEquator();
    
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

    double earthRadius = center.getSRS()->getEllipsoid()->getRadiusEquator();
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
RectangleNode::rebuild()
{    
    osgEarth::clearChildren( getPositionAttitudeTransform() );

    // construct a local-origin circle.
    GeometryFactory factory;    
    Geometry* geom = factory.createRectangle(osg::Vec3d(0,0,0), _width, _height);
    if ( geom )
    {
        GeometryCompiler compiler;
        osg::ref_ptr<osg::Node> node = compiler.compile( geom, _style, FilterContext(0L) );
        if ( node )
        {
            node = AnnotationUtils::installOverlayParent( node.get(), _style );
            getPositionAttitudeTransform()->addChild( node.get() );
        }

        applyRenderSymbology( _style );
        setLightingIfNotSet( false );
    }
}



//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( rectangle, osgEarth::Annotation::RectangleNode );


RectangleNode::RectangleNode(MapNode*              mapNode,
                             const Config&         conf,
                             const osgDB::Options* dbOptions) :
GeoPositionNode( mapNode, conf )
{
    conf.getObjIfSet( "width", _width );
    conf.getObjIfSet( "height", _height );
    conf.getObjIfSet( "style",  _style );

    rebuild();
}

Config
RectangleNode::getConfig() const
{
    Config conf = GeoPositionNode::getConfig();
    conf.key() = "rectangle";

    conf.addObj( "width",  _width );
    conf.addObj( "height", _height );
    conf.addObj( "style",  _style );

    return conf;
}
