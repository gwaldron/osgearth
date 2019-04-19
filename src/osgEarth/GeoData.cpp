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

#include <osgEarth/GeoData>
#include <osgEarth/GeoMath>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Registry>
#include <osgEarth/Terrain>


#include <gdal_priv.h>
#include <gdalwarper.h>


using namespace osgEarth;

#undef  LC
#define LC "[GeoPoint] "

GeoPoint GeoPoint::INVALID;


GeoPoint::GeoPoint(const SpatialReference* srs,
                   double x,
                   double y ) :
_srs    ( srs ),
_p      ( x, y, 0.0 ),
_altMode( ALTMODE_RELATIVE )
{
    //nop
}

GeoPoint::GeoPoint(const SpatialReference* srs,
                   double x,
                   double y,
                   double z,
                   const AltitudeMode& altMode) :
_srs    ( srs ),
_p      ( x, y, z ),
_altMode( altMode )
{
    //nop
}

GeoPoint::GeoPoint(const SpatialReference* srs,
                   double x,
                   double y,
                   double z) :
_srs    ( srs ),
_p      ( x, y, z ),
_altMode( ALTMODE_ABSOLUTE )
{
    //nop
}

GeoPoint::GeoPoint(const SpatialReference* srs,
                   const osg::Vec3d&       xyz,
                   const AltitudeMode&     altMode) :
_srs(srs),
_p  (xyz),
_altMode( altMode )
{
    //nop
}

GeoPoint::GeoPoint(const SpatialReference* srs,
                   const osg::Vec3d&       xyz) :
_srs(srs),
_p  (xyz),
_altMode( ALTMODE_ABSOLUTE )
{
    //nop
}

GeoPoint::GeoPoint(const SpatialReference* srs,
                   const GeoPoint&         rhs)
{
     rhs.transform(srs, *this);
}

GeoPoint::GeoPoint(const GeoPoint& rhs) :
_srs    ( rhs._srs.get() ),
_p      ( rhs._p ),
_altMode( rhs._altMode )
{
    //nop
}

GeoPoint::GeoPoint() :
_srs    ( 0L ),
_altMode( ALTMODE_ABSOLUTE )
{
    //nop
}

GeoPoint::GeoPoint(const Config& conf, const SpatialReference* srs) :
_srs    ( srs ),
_altMode( ALTMODE_ABSOLUTE )
{
    conf.get( "x", _p.x() );
    conf.get( "y", _p.y() );
    conf.get( "z", _p.z() );
    conf.get( "alt", _p.z() );
    conf.get( "hat", _p.z() ); // height above terrain (relative)

    if ( !_srs.valid() )
        _srs = SpatialReference::create( conf.value("srs"), conf.value("vdatum") );

    if ( conf.hasValue("lat") && (!_srs.valid() || _srs->isGeographic()) )
    {
        conf.get( "lat", _p.y() );
        if ( !_srs.valid() ) 
            _srs = SpatialReference::create("wgs84");
    }
    if ( conf.hasValue("long") && (!_srs.valid() || _srs->isGeographic()) )
    {
        conf.get("long", _p.x());
        if ( !_srs.valid() ) 
            _srs = SpatialReference::create("wgs84");
    }

    if ( conf.hasValue("mode") )
    {
        conf.get( "mode", "relative",            _altMode, ALTMODE_RELATIVE );
        conf.get( "mode", "relative_to_terrain", _altMode, ALTMODE_RELATIVE );
        conf.get( "mode", "absolute",            _altMode, ALTMODE_ABSOLUTE );
    }
    else
    {
        if (conf.hasValue("hat"))
            _altMode = ALTMODE_RELATIVE;
        else if ( conf.hasValue("alt") || conf.hasValue("z") )
            _altMode = ALTMODE_ABSOLUTE;
        else
            _altMode = ALTMODE_RELATIVE;
    }
}

Config
GeoPoint::getConfig() const
{
    Config conf;
    if ( _srs.valid() && _srs->isGeographic() )
    {
        conf.set( "lat", _p.y() );
        conf.set( "long", _p.x() );
        if ( _p.z() != 0.0 )
        {
            if ( _altMode == ALTMODE_ABSOLUTE )
                conf.set( "alt", _p.z() );
            else
                conf.set( "hat", _p.z() );
        }
    }
    else
    {
        conf.set( "x", _p.x() );
        conf.set( "y", _p.y() );
        if ( _altMode == ALTMODE_ABSOLUTE )
            conf.set( "z", _p.z() );
        else
            conf.set( "hat", _p.z() );
    }

    if ( _srs.valid() )
    {
        conf.set("srs", _srs->getHorizInitString());
        if ( _srs->getVerticalDatum() )
            conf.set("vdatum", _srs->getVertInitString());
    }

    return conf;
}

void
GeoPoint::set(const SpatialReference* srs,
              const osg::Vec3d&       xyz,
              const AltitudeMode&     altMode)
{
    _srs = srs;
    _p   = xyz;
    _altMode = altMode;
}

void
GeoPoint::set(const SpatialReference* srs,
              double                  x,
              double                  y,
              double                  z,
              const AltitudeMode&     altMode)
{
    _srs = srs;
    _p.set(x, y, z);
    _altMode = altMode;
}

bool 
GeoPoint::operator == (const GeoPoint& rhs) const
{
    return
        isValid() && rhs.isValid() &&
        _p        == rhs._p        &&
        _altMode  == rhs._altMode  &&
        ((_altMode == ALTMODE_ABSOLUTE && _srs->isEquivalentTo(rhs._srs.get())) ||
         (_altMode == ALTMODE_RELATIVE && _srs->isHorizEquivalentTo(rhs._srs.get())));
}

GeoPoint
GeoPoint::transform(const SpatialReference* outSRS) const
{
    if ( isValid() && outSRS )
    {
        osg::Vec3d out;
        if ( _altMode == ALTMODE_ABSOLUTE )
        {
            if ( _srs->transform(_p, outSRS, out) )
                return GeoPoint(outSRS, out, ALTMODE_ABSOLUTE);
        }
        else // if ( _altMode == ALTMODE_RELATIVE )
        {
            if ( _srs->transform2D(_p.x(), _p.y(), outSRS, out.x(), out.y()) )
            {
                out.z() = _p.z();
                return GeoPoint(outSRS, out, ALTMODE_RELATIVE);
            }
        }
    }
    return GeoPoint::INVALID;
}

bool
GeoPoint::transformInPlace(const SpatialReference* srs) 
{
    if ( isValid() && srs )
    {
        osg::Vec3d out;
        if ( _altMode == ALTMODE_ABSOLUTE )
        {
            if ( _srs->transform(_p, srs, out) )
            {
                set(srs, out, ALTMODE_ABSOLUTE);
                return true;
            }
        }
        else // if ( _altMode == ALTMODE_RELATIVE )
        {
            if ( _srs->transform2D(_p.x(), _p.y(), srs, out.x(), out.y()) )
            {
                out.z() = _p.z();
                set(srs, out, ALTMODE_RELATIVE);
                return true;
            }
        }
    }
    return false;
}

bool
GeoPoint::transformZ(const AltitudeMode& altMode, const TerrainResolver* terrain ) 
{
    double z;
    if ( transformZ(altMode, terrain, z) )
    {
        _p.z() = z;
        _altMode = altMode;
        return true;
    }
    return false;
}

bool
GeoPoint::transformZ(const AltitudeMode& altMode, const TerrainResolver* terrain, double& out_z ) const
{
    if ( !isValid() )
        return false;
    
    // already in the target mode? just return z.
    if ( _altMode == altMode ) 
    {
        out_z = z();
        return true;
    }

    if ( !terrain )
        return false;

    // convert to geographic if necessary and sample the MSL height under the point.
    double out_hamsl;
    if ( !terrain->getHeight(_srs.get(), x(), y(), &out_hamsl) )
    {
        return false;
    }

    // convert the Z value as appropriate.
    if ( altMode == ALTMODE_RELATIVE )
    {
        out_z = z() - out_hamsl;
    }
    else // if ( altMode == ALTMODE_ABSOLUTE )
    {
        out_z = z() + out_hamsl;
    }
    return true;
}

bool
GeoPoint::makeGeographic()
{
    if ( !isValid() ) return false;
    if ( !_srs->isGeographic() )
        return _srs->transform( _p, _srs->getGeographicSRS(), _p);
    return true;
}

bool
GeoPoint::transform(const SpatialReference* outSRS, GeoPoint& output) const
{
    output = transform(outSRS);
    return output.isValid();
}

bool
GeoPoint::toWorld( osg::Vec3d& out_world ) const
{
    if ( !isValid() )
    {
        OE_WARN << LC << "Called toWorld() on an invalid point" << std::endl;
        return false;
    }
    if ( _altMode != ALTMODE_ABSOLUTE )
    {
        OE_WARN << LC << "ILLEGAL: called GeoPoint::toWorld with AltitudeMode = RELATIVE_TO_TERRAIN" << std::endl;
        return false;
    }
    return _srs->transformToWorld( _p, out_world );
}

bool
GeoPoint::toWorld( osg::Vec3d& out_world, const TerrainResolver* terrain ) const
{
    if ( !isValid() )
    {
        OE_WARN << LC << "Called toWorld() on an invalid point" << std::endl;
        return false;
    }
    if ( _altMode == ALTMODE_ABSOLUTE )
    {
        return _srs->transformToWorld( _p, out_world );
    }
    else if ( terrain != 0L )
    {
        GeoPoint absPoint = *this;
        if (!absPoint.makeAbsolute( terrain ))
        {
            return false;            
        }
        return absPoint.toWorld( out_world );
    }
    else
    {
        OE_WARN << LC << "ILLEGAL: called GeoPoint::toWorld with AltitudeMode = RELATIVE_TO_TERRAIN" << std::endl;
        return false;
    }
}


bool
GeoPoint::fromWorld(const SpatialReference* srs, const osg::Vec3d& world)
{
    if ( srs )
    {
        osg::Vec3d p;
        if ( srs->transformFromWorld(world, p) )
        {
            set( srs, p, ALTMODE_ABSOLUTE );
            return true;
        }
    }
    return false;
}

bool
GeoPoint::createLocalToWorld( osg::Matrixd& out_l2w ) const
{
    if ( !isValid() ) return false;
    bool result = _srs->createLocalToWorld( _p, out_l2w );
    if ( _altMode != ALTMODE_ABSOLUTE )
    {
        OE_DEBUG << LC << "ILLEGAL: called GeoPoint::createLocalToWorld with AltitudeMode = RELATIVE_TO_TERRAIN" << std::endl;
        return false;
    }
    return result;
}

bool
GeoPoint::createWorldToLocal( osg::Matrixd& out_w2l ) const
{
    if ( !isValid() ) return false;
    bool result = _srs->createWorldToLocal( _p, out_w2l );
    if ( _altMode != ALTMODE_ABSOLUTE )
    {
        OE_DEBUG << LC << "ILLEGAL: called GeoPoint::createWorldToLocal with AltitudeMode = RELATIVE_TO_TERRAIN" << std::endl;
        return false;
    }
    return result;
}

bool
GeoPoint::createWorldUpVector( osg::Vec3d& out_up ) const
{
    if ( !isValid() ) return false;

    if ( _srs->isProjected() )
    {
        out_up.set(0, 0, 1);
        return true;
    }
    else
    {
        osg::Vec3d ecef;
        if ( this->toWorld( ecef ) )
        {
            out_up = _srs->getEllipsoid()->computeLocalUpVector( ecef.x(), ecef.y(), ecef.z() );
            return true;
        }
    }
    return false;
}

GeoPoint
GeoPoint::interpolate(const GeoPoint& rhs, double t) const
{
    if (t == 0.0)
        return *this;
    else if (t == 1.0)
        return rhs;

    GeoPoint result;

    // Geometric slerp in unit sphere space
    // https://en.wikipedia.org/wiki/Slerp#Geometric_Slerp

    GeoPoint p1 = transform(getSRS()->getGeodeticSRS());
    GeoPoint p2 = rhs.transform(p1.getSRS());

    double deltaZ = p2.z()-p1.z();

    // Convert each point to unit sphere world space:
    osg::Vec3d unitToEllip(
        getSRS()->getEllipsoid()->getRadiusEquator(),
        getSRS()->getEllipsoid()->getRadiusEquator(),
        getSRS()->getEllipsoid()->getRadiusPolar());

    osg::Vec3d ellipToUnit = osg::componentDivide(
        osg::Vec3d(1,1,1), unitToEllip);
        
    osg::Vec3d w1;
    p1.toWorld(w1);
    w1 = osg::componentMultiply(w1, ellipToUnit);
    w1.normalize();

    osg::Vec3d w2;
    p2.toWorld(w2);
    w2 = osg::componentMultiply(w2, ellipToUnit);
    w2.normalize();

    // perform geometric slerp:
    double dp = w1*w2;
    if (dp == 1.0)
        return *this;

    double angle = acos(dp);

    double s = sin(angle);
    if (s == 0.0)
        return *this;

    double c1 = sin((1.0-t)*angle)/s;
    double c2 = sin(t*angle)/s;

    osg::Vec3d n = w1*c1 + w2*c2;

    // convert back to world space and apply altitude lerp
    n = osg::componentMultiply(n, unitToEllip);
    result.fromWorld(getSRS(), n);
    result.z() = p1.z() + t*deltaZ;

    result.transformInPlace(getSRS());
    return result;
}

double
GeoPoint::distanceTo(const GeoPoint& rhs) const
{
    // https://en.wikipedia.org/wiki/Geographical_distance#Ellipsoidal-surface_formulae

    // Convert both points to geographic
    GeoPoint p1 = transform( getSRS()->getGeographicSRS() );
    GeoPoint p2 = rhs.transform( p1.getSRS() );

    double Re = getSRS()->getEllipsoid()->getRadiusEquator();
    double Rp = getSRS()->getEllipsoid()->getRadiusPolar();
    double F  = (Re-Rp)/Re; // flattening

    double
        lat1 = osg::DegreesToRadians(p1.y()),
        lon1 = osg::DegreesToRadians(p1.x()),
        lat2 = osg::DegreesToRadians(p2.y()),
        lon2 = osg::DegreesToRadians(p2.x());

    double B1 = atan( (1.0-F)*tan(lat1) );
    double B2 = atan( (1.0-F)*tan(lat2) );

    double P = (B1+B2)/2.0;
    double Q = (B2-B1)/2.0;

    double G = acos(sin(B1)*sin(B2) + cos(B1)*cos(B2)*cos(fabs(lon2-lon1)));

    double 
        sinG = sin(G), 
        sinP = sin(P), sinQ = sin(Q), 
        cosP = cos(P), cosQ = cos(Q), 
        sinG2 = sin(G/2.0), cosG2 = cos(G/2.0);

    double X = (G-sinG)*((sinP*sinP*cosQ*cosQ)/(cosG2*cosG2));
    double Y = (G+sinG)*((cosP*cosP*sinQ*sinQ)/(sinG2*sinG2));

    double dist = Re*(G-(F/2.0)*(X+Y));

    // NaN could mean start/end points are the same
    return osg::isNaN(dist)? 0.0 : dist;
}

std::string
GeoPoint::toString() const
{
    std::stringstream buf;
    buf << "x=" << x() << ", y=" << y() << ", z=" << z() << "; m=" <<
        (_altMode == ALTMODE_ABSOLUTE ? "abs" : "rel");
    return buf.str();
}


//------------------------------------------------------------------------

#undef  LC
#define LC "[GeoCircle] "

GeoCircle GeoCircle::INVALID = GeoCircle();


GeoCircle::GeoCircle() :
_center( GeoPoint::INVALID ),
_radius( -1.0 )
{
    //nop
}


GeoCircle::GeoCircle(const GeoCircle& rhs) :
_center( rhs._center ),
_radius( rhs._radius )
{
    //nop
}


GeoCircle::GeoCircle(const GeoPoint& center,
                     double          radius ) :
_center( center ),
_radius( radius )
{
    //nop
}


bool
GeoCircle::operator == ( const GeoCircle& rhs ) const
{
    return 
        _center == rhs._center && 
        osg::equivalent(_radius, rhs._radius);
}


GeoCircle
GeoCircle::transform( const SpatialReference* srs ) const
{
    return GeoCircle(
        getCenter().transform( srs ),
        getRadius() );
}


bool
GeoCircle::transform( const SpatialReference* srs, GeoCircle& output ) const
{
    output._radius = _radius;
    return getCenter().transform( srs, output._center );
}


bool 
GeoCircle::intersects( const GeoCircle& rhs ) const
{
    if ( !isValid() || !rhs.isValid() )
        return false;

    if ( !getSRS()->isHorizEquivalentTo( rhs.getSRS() ) )
    {
        return intersects( rhs.transform(getSRS()) );
    }
    else
    {
        if ( getSRS()->isProjected() )
        {
            osg::Vec2d vec = osg::Vec2d(_center.x(), _center.y()) - osg::Vec2d(rhs.getCenter().x(), rhs.getCenter().y());
            return vec.length2() <= (_radius + rhs.getRadius())*(_radius + rhs.getRadius());
        }
        else // if ( isGeographic() )
        {
            osg::Vec3d p0( _center.x(), _center.y(), 0.0 );
            osg::Vec3d p1( rhs.getCenter().x(), rhs.getCenter().y(), 0.0 );
            return GeoMath::distance( p0, p1, getSRS() ) <= (_radius + rhs.getRadius());
        }
    }
}


//------------------------------------------------------------------------

#undef  LC
#define LC "[GeoExtent] "

namespace {
    bool is_valid(double n) {
        return
            osg::isNaN(n) == false &&
            n != DBL_MAX &&
            n != -DBL_MAX;
    }
}

GeoExtent GeoExtent::INVALID = GeoExtent();


GeoExtent::GeoExtent():
_srs(0L),
_west(0.0),
_width(-1.0),
_south(0.0),
_height(-1.0)
{
    //NOP - invalid
}

GeoExtent::GeoExtent(const SpatialReference* srs) :
_srs(srs),
_west(0.0),
_width(-1.0),
_south(0.0),
_height(-1.0)
{
    //NOP - invalid
}

GeoExtent::GeoExtent(const SpatialReference* srs,
                     double west, double south, double east, double north ) :
_srs( srs ),
_west(0.0),
_width(-1.0),
_south(0.0),
_height(-1.0)
{
    set(west, south, east, north);
}


GeoExtent::GeoExtent(const SpatialReference* srs, const Bounds& bounds) :
_srs(srs),
_west(0.0),
_width(-1.0),
_south(0.0),
_height(-1.0)
{
    set(bounds.xMin(), bounds.yMin(), bounds.xMax(), bounds.yMax());
}

GeoExtent::GeoExtent(const GeoExtent& rhs) :
_srs(rhs._srs),
_west(rhs._west),
_width(rhs._width),
_south(rhs._south),
_height(rhs._height)
{
    //NOP
}

bool
GeoExtent::isGeographic() const
{
    return _srs.valid() && _srs->isGeographic();
}

void
GeoExtent::set(double west, double south, double east, double north)
{
    // Validate input.
    if (!is_valid(west) ||
        !is_valid(south) ||
        !is_valid(east) ||
        !is_valid(north) ||
        south > north)
    {
        _west = _south = 0.0;
        _width = _height = -1.0;
        return;
    }

    // In this method, east is always to the east of west!
    // If it appears not to be, that means the extent crosses the antimeridian.
    west = normalizeX(west);
    double width = 0.0;
    double height = 0.0;

    if (isGeographic())
    {
        // ensure east >= west in a geographic frame.
        while (east < west)
            east += 360.0;
    }
    width = osg::maximum(0.0, east - west);

    height = osg::maximum(0.0, north-south);

    setOriginAndSize(west, south, width, height);
}

void
GeoExtent::setOriginAndSize(double west, double south, double width, double height)
{
    _west = west;
    _south = south;
    _width = width;
    _height = height;
    clamp();
}

bool
GeoExtent::getCentroid(GeoPoint& out) const
{
    out = GeoPoint(_srs.get(), getCentroid(), ALTMODE_ABSOLUTE);
    return true;
}

bool
GeoExtent::operator == ( const GeoExtent& rhs ) const
{
    if ( !isValid() && !rhs.isValid() )
        return true;

    if ( !isValid() || !rhs.isValid() )
        return false;

    return
        west()  == rhs.west()  &&
        east()  == rhs.east()  &&
        south() == rhs.south() &&
        north() == rhs.north() &&
        _srs->isEquivalentTo( rhs._srs.get() );
}

bool
GeoExtent::operator != ( const GeoExtent& rhs ) const
{
    return !( *this == rhs );
}

bool
GeoExtent::isValid() const
{
    return _srs.valid() && _width >= 0.0 && _height >= 0.0;
}

bool
GeoExtent::getCentroid(double& out_x, double& out_y) const
{
    if (isInvalid()) return false;

    out_x = normalizeX(west() + 0.5*width());
    out_y = south() + 0.5*height();
    return true;
}

bool
GeoExtent::crossesAntimeridian() const
{
    return _srs.valid() && _srs->isGeographic() && east() < west(); //west()+width() > 180.0;
}

bool
GeoExtent::splitAcrossAntimeridian(GeoExtent& out_west, GeoExtent& out_east) const
{
    if ( crossesAntimeridian() )
    {
        double width_new;

        out_west = *this;
        width_new = 180.0 - west();
        out_west.setOriginAndSize(180.0 - width_new, south(), width_new, height());

        out_east = *this;
        width_new = east() - (-180.0);
        out_east.setOriginAndSize(-180.0, south(), width_new, height());
    }
    else if ( !_srs->isGeographic() )
    {
        //note: may not actually work.
        GeoExtent latlong_extent = transform( _srs->getGeographicSRS() );
        GeoExtent w, e;
        if ( latlong_extent.splitAcrossAntimeridian( w, e ) )
        {
            out_west = w.transform( _srs.get() );
            out_east = e.transform( _srs.get() );
        }
    }

    return out_west.isValid() && out_east.isValid();
}

GeoExtent
GeoExtent::transform(const SpatialReference* to_srs) const 
{       
    //TODO: this may not work across the antimeridian - unit test required
    if ( isValid() && to_srs )
    {
        // do not normalize the X values here.
        double xmin = west(), ymin = south();
        double xmax = west() + width(), ymax = south() + height();
        
        if ( _srs->transformExtentToMBR(to_srs, xmin, ymin, xmax, ymax) )
        {
            return GeoExtent( to_srs, xmin, ymin, xmax, ymax );
        }
    }
    return GeoExtent::INVALID;
}


bool
GeoExtent::transform( const SpatialReference* srs, GeoExtent& output ) const
{
    output = transform(srs);
    return output.isValid();
}

void
GeoExtent::getBounds(double &xmin, double &ymin, double &xmax, double &ymax) const
{
    xmin = west();
    ymin = south();
    xmax = west() + width();
    ymax = south() + height();
}

Bounds
GeoExtent::bounds() const
{
    double west, east, south, north;
    getBounds(west, south, east, north);
    return Bounds( west, south, east, north );
}

bool
GeoExtent::contains(double x, double y, const SpatialReference* srs) const
{
    if (isInvalid() || !is_valid(x) || !is_valid(y))
        return false;

    osg::Vec3d xy( x, y, 0 );
    osg::Vec3d local = xy;

    // See if we need to xform the input:
    if (srs && srs->isHorizEquivalentTo(_srs.get()) == false)
    {
        // If the transform fails, bail out with error
        if (srs->transform(xy, _srs.get(), local) == false)
        {
            return false;
        }
    }

    // Quantize the Y coordinate to account for tiny rounding errors:
    if (osg::equivalent(south(), local.y()))
        local.y() = south();
    if (osg::equivalent(north(), local.y()))
        local.y() = north();

    // Test the Y coordinate:
    if (local.y() < south() || local.y() > north())
        return false;

    // Bring the X coordinate into normal range:
    local.x() = normalizeX(local.x());
    
    // Quantize the X coordinate to account for tiny rounding errors:
    if (osg::equivalent(west(), local.x()))
        local.x() = west();
    if (osg::equivalent(east(), local.x()))
        local.x() = east();

    // account for the antimeridian wrap-around:
    double a0 = west(), a1 = west() + width();
    double b0 = east() - width(), b1 = east();
    return (a0 <= local.x() && local.x() <= a1) || (b0 <= local.x() && local.x() <= b1);
}

bool
GeoExtent::contains(const GeoPoint& rhs) const
{
    return contains( rhs.x(), rhs.y(), rhs.getSRS() );
}

bool
GeoExtent::contains(const Bounds& rhs) const
{
    return
        isValid() &&
        rhs.isValid() &&
        contains( rhs.xMin(), rhs.yMin() ) &&
        contains( rhs.xMax(), rhs.yMax() ) &&
        contains( rhs.center() );
}

bool
GeoExtent::contains(const GeoExtent& rhs) const
{
    return
        isValid() &&
        rhs.isValid() &&
        contains( rhs.west(), rhs.south(), rhs.getSRS() ) &&
        contains( rhs.east(), rhs.north(), rhs.getSRS() ) &&
        contains( rhs.getCentroid(), rhs.getSRS() );   // this accounts for the antimeridian
}

#undef  OVERLAPS
#define OVERLAPS(A, B, C, D) (!(B <= C || A >= D))

bool
GeoExtent::intersects(const GeoExtent& rhs, bool checkSRS) const
{
    if ( !isValid() || !rhs.isValid() )
        return false;

    // Transform the incoming extent if necessary:
    if ( checkSRS && !_srs->isHorizEquivalentTo(rhs.getSRS()) )
    {
        if (_srs->isContiguous())
        {
            GeoExtent rhsExt = rhs.transform(getSRS());
            return this->intersects( rhsExt, false );
        }
        else
        {
            // non-contiguous projection? convert to a contiguous one:
            GeoExtent thisGeo = transform(getSRS()->getGeographicSRS());
            GeoExtent rhsGeo = rhs.transform(getSRS()->getGeographicSRS());
            return thisGeo.intersects(rhsGeo, false);
        }
    }

    // Trivial reject: y-dimension does not overlap:
    bool y_excl = south() >= rhs.north() || north() <= rhs.south();
    if (y_excl)
        return false;

    // Trivial reject: x-dimension does not overlap in projected SRS:
    if (!_srs->isGeographic())
    {
        bool x_excl = west() >= rhs.east() || east() <= rhs.west();
        return x_excl == false;
    }

    // By now we know that Y overlaps and we are in a geographic SRS
    // and therefore must consider the antimeridian wrap-around in X.
    // a and b are "this"; c and d are "rhs":
    double a0 = east() - width(), a1 = east();
    double b0 = west(), b1 = west() + width();
    double c0 = rhs.east() - rhs.width(), c1 = rhs.east();
    double d0 = rhs.west(), d1 = rhs.west() + rhs.width();
    return
        OVERLAPS(a0, a1, c0, c1) ||
        OVERLAPS(a0, a1, d0, d1) ||
        OVERLAPS(b0, b1, c0, c1) ||
        OVERLAPS(b0, b1, d0, d1);
}

GeoCircle
GeoExtent::computeBoundingGeoCircle() const
{
    GeoCircle circle;

    if ( !isValid() )
    {
        circle.setRadius( -1.0 );
    }
    else 
    {
        double x, y;
        getCentroid( x, y );

        if ( getSRS()->isProjected() )
        {
            double ext = osg::maximum( width(), height() );
            circle.setRadius( 0.5*ext * 1.414121356237 ); /*sqrt(2)*/
        }
        else // isGeographic
        {
            osg::Vec3d center, sw, se, ne, nw;

            GeoPoint(getSRS(), x, y, 0, ALTMODE_ABSOLUTE).toWorld(center);
            GeoPoint(getSRS(), west(), south(), 0, ALTMODE_ABSOLUTE).toWorld(sw);
            GeoPoint(getSRS(), east(), south(), 0, ALTMODE_ABSOLUTE).toWorld(se);
            GeoPoint(getSRS(), east(), north(), 0, ALTMODE_ABSOLUTE).toWorld(ne);
            GeoPoint(getSRS(), west(), north(), 0, ALTMODE_ABSOLUTE).toWorld(nw);
            
            double radius2 = (center-sw).length2();
            radius2 = osg::maximum(radius2, (center-se).length2());
            radius2 = osg::maximum(radius2, (center-ne).length2());
            radius2 = osg::maximum(radius2, (center-sw).length2());

            circle.setRadius( sqrt(radius2) );
        }

        circle.setCenter( GeoPoint(getSRS(), x, y, 0.0, ALTMODE_ABSOLUTE) );
    }

    return circle;
}

void
GeoExtent::expandToInclude(double x, double y)
{
    if (!is_valid(x) || !is_valid(y))
        return;

    // First, bring the X coordinate into the local frame.
    x = normalizeX(x);

    // Invalid? Set to a point.
    if (isInvalid())
    {
        set(x, y, x, y);
        return;
    }

    // Check each coordinate separately:
    double cx, cy;
    getCentroid(cx, cy);
    bool containsX = contains(x, cy);
    bool containsY = contains(cx, y);

    // Expand along the Y axis:
    if (!containsY)
    {
        if (y < south())
        {
            _height += (_south-y);
            _south = y;
        }
        else if (y > north())
        {
            _height = y - south();
        }
    }

    if (!containsX)
    {
        if (isGeographic())
        {
            if (x > west())
            {
                double w0 = x - west(); // non-wrap-around width
                double w1 = (180.0 - x) + (west() - (-180.0) + _width); // wrap-around width
                if (w0 <= w1)
                {
                    _width = w0;
                }
                else
                {
                    _west = x;
                    _width = w1;
                }
            }
            else // (x < west())
            {
                double w0 = _width + (west() - x); // non-wrap-around
                double w1 = (x - (-180.0)) + (180.0 - west()); // wrap-around
                if (w0 < w1)
                {
                    _west = x;
                    _width = w0;
                }
                else
                {
                    _width = w1;
                }
            }
        }
        else
        {
            // projected mode is the same approach as Y
            if (x < west())
            {
                _width += _west - x;
                _west = x;
            }
            else if (x > east())
            {
                _width = x - west();
            }
        }
    }

    if (!containsX || !containsY)
    {
        clamp();
    }
}

bool
GeoExtent::expandToInclude(const GeoExtent& rhs)
{
    if (rhs.isInvalid() || !_srs.valid())
        return false;

    if ( !rhs.getSRS()->isHorizEquivalentTo( _srs.get() ) )
    {
        return expandToInclude( rhs.transform(_srs.get()) );
    }

    else
    {        
        if (area() <= 0.0)
        {
            *this = rhs;
            return true;
        }

        double h = osg::maximum(north(), rhs.north()) - osg::minimum(south(), rhs.south());
        if (rhs.south() < south())
        {
            _south = rhs.south();
        }
        _height = h;
        
        // non-wrap-around new width:
        double w0 = osg::maximum(xMax(), rhs.xMax()) - osg::minimum(xMin(), rhs.xMin());

        if (isGeographic())
        {
            // wrap-around width:
            double w1 = west() > rhs.east()? (180-west())+(rhs.east()-(-180)) : (180-rhs.west()) + (east()-(-180));

            // pick the smaller one:
            if (w0 < w1)
            {
                if (w0 > _width)
                {
                    _width = w0;
                    _west = osg::minimum(west(), rhs.west());
                }
            }
            else
            {
                if (w1 > _width)
                {
                    _width = w1;
                    if (west() <= rhs.east())
                        _west = rhs.west();
                }
            }
        }
        else
        {
            // projected mode is the same approach as Y
            _west = osg::minimum(west(), rhs.west());
            _width = w0;
        }

    }

    return true;
}

namespace
{
    void sort4(double* n, bool* b)
    {
        if (n[0] > n[1]) std::swap(n[0], n[1]), std::swap(b[0], b[1]);
        if (n[2] > n[3]) std::swap(n[2], n[3]), std::swap(b[2], b[3]);
        if (n[0] > n[2]) std::swap(n[0], n[2]), std::swap(b[0], b[2]);
        if (n[1] > n[3]) std::swap(n[1], n[3]), std::swap(b[1], b[3]);
        if (n[1] > n[2]) std::swap(n[1], n[2]), std::swap(b[1], b[2]);
    }
}

GeoExtent
GeoExtent::intersectionSameSRS(const GeoExtent& rhs) const
{
    if ( isInvalid() || rhs.isInvalid() || !_srs->isHorizEquivalentTo( rhs.getSRS() ) )
        return GeoExtent::INVALID;

    if ( !intersects(rhs) )
    {
        OE_DEBUG << "Extents " << toString() << " and " << rhs.toString() << " do not intersect."
            << std::endl;
        return GeoExtent::INVALID;
    }

    GeoExtent result( *this );

    if (isGeographic())
    {
        if (width() == 360.0)
        {
            result._west = rhs._west;
            result._width = rhs._width;
        }
        else if (rhs.width() == 360.0)
        {
            result._west = _west;
            result._width = _width;
        }
        else
        {
            // Sort the four X coordinates, remembering whether each one is west or east edge:
            double x[4];
            bool iswest[4];
            x[0] = west(), x[1] = east(), x[2] = rhs.west(), x[3] = rhs.east();
            iswest[0] = true, iswest[1] = false, iswest[2] = true, iswest[3] = false;
            sort4(x, iswest);

            // find the western-most west coord:
            int iw = -1;
            for (int i=0; i<4 && iw<0; ++i)
            {
                if (iswest[i])
                    iw = i;
            }

            // iterate from there, finding the LAST west coord and stopping on the 
            // FIRST east coord found.
            int q = iw+4;
            int ie = -1;
            for (int i = iw; i < q && ie < 0; ++i)
            {
                int j = i;
                if (j >= 4) j-=4;
                if (iswest[j])
                    iw = j; // found a better west coord; remember it.
                else
                    ie = j; // found the western-most east coord; done.
            }

            result._west = x[iw];
            if (ie >= iw)
                result._width = x[ie] - x[iw];
            else
                result._width = (180.0 - x[iw]) + (x[ie] - (-180.0)); // crosses the antimeridian
        }
    }
    else
    {
        // projected mode is simple
        result._west = osg::maximum(west(), rhs.west());
        double eastTemp = osg::minimum(east(), rhs.east());
        result._width = eastTemp - result._west;
    }

    result._south = osg::maximum(south(), rhs.south());
    double northTemp = osg::minimum(north(), rhs.north());
    result._height = northTemp - result._south;

    result.clamp();

    OE_DEBUG << "Intersection of " << this->toString() << " and " << rhs.toString() << " is: " 
        << result.toString()
        << std::endl;

    return result;
}

void
GeoExtent::scale(double x_scale, double y_scale)
{
    if (isInvalid() || !is_valid(x_scale) || !is_valid(y_scale))
        return;

    double cx = _west + 0.5*_width;

    double cy = _south + 0.5*_height;
     
    setOriginAndSize(
        normalizeX(cx - 0.5*_width*x_scale),
        cy - 0.5*_height*y_scale,
        _width * x_scale,
        _height * y_scale);
}

void
GeoExtent::expand(double x, double y)
{
    if (isInvalid() || !is_valid(x) || !is_valid(y))
        return;

    setOriginAndSize(
        normalizeX(_west - 0.5*x),
        _south - 0.5*y,
        _width + x,
        _height + y);
}

void
GeoExtent::clamp()
{
    if (osg::equivalent(_west, floor(_west)))
        _west = floor(_west);
    else if (osg::equivalent(_west, ceil(_west)))
        _west = ceil(_west);

    if (osg::equivalent(_south, floor(_south)))
        _south = floor(_south);
    else if (osg::equivalent(_south, ceil(_south)))
        _south = ceil(_south);

    if (osg::equivalent(_width, floor(_width)))
        _width = floor(_width);
    else if (osg::equivalent(_width, ceil(_width)))
        _width = ceil(_width);

    if (osg::equivalent(_height, floor(_height)))
        _height = floor(_height);
    else if (osg::equivalent(_height, ceil(_height)))
        _height = ceil(_height);

    if (isGeographic())
    {
        _width = osg::clampBetween(_width, 0.0, 360.0);
        _height = osg::clampBetween(_height, 0.0, 180.0);

        if (south() < -90.0)
        {
            _height -= (-90.0)-_south;
            _south = -90.0;
        }
        else if (north() > 90.0)
        {
            _height -= (north()-90.0);            
        }
    }
}

double
GeoExtent::area() const
{
    return isValid() ? width() * height() : 0.0;
}

double
GeoExtent::normalizeX(double x) const
{
    if (isValid() && is_valid(x) && _srs->isGeographic())
    {
        if (fabs(x) <= 180.0)
        {
            return x;
        }

        if (x < 0.0 || x >= 360.0)
        {
            x = fmod(x, 360.0);
            if (x < 0.0)
                x += 360.0;
        }
        
        if (x > 180.0)
        {
            x -= 360.0;
        }
    }
    return x;
}

std::string
GeoExtent::toString() const
{
    std::stringstream buf;
    if ( !isValid() )
        buf << "INVALID";
    else
        buf << std::setprecision(12) << "SW=" << west() << "," << south() << " NE=" << east() << "," << north();

    if (_srs.valid() == true)
    {
        buf << ", SRS=" << _srs->getName();
    }
    else
    {
        buf << ", SRS=NULL";
    }

    std::string bufStr;
    bufStr = buf.str();
    return bufStr;
}

bool
GeoExtent::createPolytope(osg::Polytope& tope) const
{
    if ( ! this->isValid() )
        return false;

    if ( getSRS()->isProjected() )
    {
        // add planes for the four sides of the extent, Normals point inwards.
        double halfWidth  = 0.5*width();
        double halfHeight = 0.5*height();
        tope.add( osg::Plane(osg::Vec3d( 1, 0,0), osg::Vec3d(-halfWidth,0,0)) );
        tope.add( osg::Plane(osg::Vec3d(-1, 0,0), osg::Vec3d( halfWidth,0,0)) );
        tope.add( osg::Plane(osg::Vec3d( 0, 1,0), osg::Vec3d(0, -halfHeight,0)) );
        tope.add( osg::Plane(osg::Vec3d( 0,-1,0), osg::Vec3d(0,  halfHeight,0)) );
    }

    else
    {
        // for each extent, create a plane passing through the planet's centroid.

        // convert 4 corners to world space (ECEF)
        osg::Vec3d center(0.0, 0.0, 0.0);
        osg::Vec3d sw, se, nw, ne;
        GeoPoint(getSRS(), west(), south(), 0.0, ALTMODE_ABSOLUTE).toWorld( sw );
        GeoPoint(getSRS(), east(), south(), 0.0, ALTMODE_ABSOLUTE).toWorld( se );
        GeoPoint(getSRS(), east(), north(), 0.0, ALTMODE_ABSOLUTE).toWorld( ne );
        GeoPoint(getSRS(), west(), north(), 0.0, ALTMODE_ABSOLUTE).toWorld( nw );

        // bounding planes in ECEF space:
        tope.add( osg::Plane(center, nw, sw) ); // west
        tope.add( osg::Plane(center, sw, se) ); // south
        tope.add( osg::Plane(center, se, ne) ); // east
        tope.add( osg::Plane(center, ne, nw) ); // north
    }

    return true;
}

osg::BoundingSphered
GeoExtent::createWorldBoundingSphere(double minElev, double maxElev) const
{
    osg::BoundingSphered bs;

    if (getSRS()->isProjected())
    {
        osg::Vec3d w;
        GeoPoint(getSRS(), xMin(), yMin(), minElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMax(), yMax(), maxElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
    }

    else // geocentric
    {
        osg::Vec3d w;
        GeoPoint(getSRS(), xMin(), yMin(), minElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMax(), yMin(), minElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMax(), yMax(), minElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMin(), yMax(), minElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMin(), yMin(), maxElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMax(), yMin(), maxElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMax(), yMax(), maxElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
        GeoPoint(getSRS(), xMin(), yMax(), maxElev, ALTMODE_ABSOLUTE).toWorld(w); bs.expandBy(w);
    }

    return bs;
}

bool
GeoExtent::createScaleBias(const GeoExtent& rhs, osg::Matrixf& output) const
{    
    double scalex = width() / rhs.width();
    double scaley = height() / rhs.height();
    double biasx  = (west()-rhs.west()) / rhs.width();
    double biasy  = (south()-rhs.south()) / rhs.height();

    output(0,0) = scalex;
    output(1,1) = scaley;
    output(3,0) = biasx;
    output(3,1) = biasy;

    return true;
}

bool
GeoExtent::createScaleBias(const GeoExtent& rhs, osg::Matrixd& output) const
{    
    double scalex = width() / rhs.width();
    double scaley = height() / rhs.height();
    double biasx  = (west()-rhs.west()) / rhs.width();
    double biasy  = (south()-rhs.south()) / rhs.height();

    output(0,0) = scalex;
    output(1,1) = scaley;
    output(3,0) = biasx;
    output(3,1) = biasy;

    return true;
}

/***************************************************************************/

DataExtent::DataExtent(const GeoExtent& extent, unsigned minLevel, unsigned maxLevel, const std::string &description) :
GeoExtent(extent)
{
    _minLevel = minLevel;
    _maxLevel = maxLevel;
    _description = description;
}

DataExtent::DataExtent(const GeoExtent& extent, const std::string &description) :
GeoExtent(extent),
_minLevel( 0 ),
_maxLevel( 0 )
{
    _description = description;
}

DataExtent::DataExtent(const GeoExtent& extent, unsigned minLevel,  unsigned maxLevel) :
GeoExtent(extent)
{
    _minLevel = minLevel;
    _maxLevel = maxLevel;
}

DataExtent::DataExtent(const GeoExtent& extent, unsigned minLevel) :
GeoExtent(extent),
_maxLevel( 25 )
{
    _minLevel = minLevel;
}

DataExtent::DataExtent(const GeoExtent& extent, unsigned minLevel, const std::string &description) :
GeoExtent(extent),
_maxLevel( 0 )
{
    _minLevel = minLevel;
    _description = description;
}

DataExtent::DataExtent(const GeoExtent& extent ) :
GeoExtent(extent),
_minLevel( 0 ),
_maxLevel( 25 )
{
    //nop
}


/***************************************************************************/

#undef  LC
#define LC "[GeoImage] "

// static
GeoImage GeoImage::INVALID( 0L, GeoExtent::INVALID );

GeoImage::GeoImage() :
_image ( 0L ),
_extent( GeoExtent::INVALID )
{
    //nop
}

GeoImage::GeoImage(osg::Image* image, const GeoExtent& extent) :
_image(image),
_extent(extent)
{
    if ( _image.valid() && extent.isInvalid() )
    {
        OE_WARN << LC << "ILLEGAL: created a GeoImage with a valid image and an invalid extent" << std::endl;
    }
}

bool
GeoImage::valid() const 
{
    return _image.valid() && _extent.isValid();
}

osg::Image*
GeoImage::getImage() const
{
    return _image.get();
}

const SpatialReference*
GeoImage::getSRS() const
{
    return _extent.getSRS();
}

const GeoExtent&
GeoImage::getExtent() const
{
    return _extent;
}

double
GeoImage::getUnitsPerPixel() const
{
    double uppw = _extent.width() / (double)_image->s();
    double upph = _extent.height() / (double)_image->t();
    return (uppw + upph) / 2.0;
}

GeoImage
GeoImage::crop( const GeoExtent& extent, bool exact, unsigned int width, unsigned int height, bool useBilinearInterpolation) const
{
    if ( !valid() )
        return *this;

    //Check for equivalence
    if ( extent.getSRS()->isEquivalentTo( getSRS() ) )
    {
        //If we want an exact crop or they want to specify the output size of the image, use GDAL
        if (exact || width != 0 || height != 0 )
        {
            OE_DEBUG << "[osgEarth::GeoImage::crop] Performing exact crop" << std::endl;

            //Suggest an output image size
            if (width == 0 || height == 0)
            {
                double xRes = getExtent().width() / (double)_image->s(); //(getExtent().xMax() - getExtent().xMin()) / (double)_image->s();
                double yRes = getExtent().height() / (double)_image->t(); //(getExtent().yMax() - getExtent().yMin()) / (double)_image->t();

                width =  osg::maximum(1u, (unsigned int)(extent.width() / xRes));
                height = osg::maximum(1u, (unsigned int)(extent.height() / yRes));
                //width =  osg::maximum(1u, (unsigned int)((extent.xMax() - extent.xMin()) / xRes));
                //height = osg::maximum(1u, (unsigned int)((extent.yMax() - extent.yMin()) / yRes));

                OE_DEBUG << "[osgEarth::GeoImage::crop] Computed output image size " << width << "x" << height << std::endl;
            }

            //Note:  Passing in the current SRS simply forces GDAL to not do any warping
            return reproject( getSRS(), &extent, width, height, useBilinearInterpolation );
        }
        else
        {
            OE_DEBUG << "[osgEarth::GeoImage::crop] Performing non-exact crop " << std::endl;
            //If an exact crop is not desired, we can use the faster image cropping code that does no resampling.
            double destXMin = extent.xMin();
            double destYMin = extent.yMin();
            double destXMax = extent.xMax();
            double destYMax = extent.yMax();

            osg::Image* new_image = ImageUtils::cropImage(
                _image.get(),
                _extent.xMin(), _extent.yMin(), _extent.xMax(), _extent.yMax(),
                destXMin, destYMin, destXMax, destYMax );

            //The destination extents may be different than the input extents due to not being able to crop along pixel boundaries.
            return new_image?
                GeoImage( new_image, GeoExtent( getSRS(), destXMin, destYMin, destXMax, destYMax ) ) :
                GeoImage::INVALID;
        }
    }
    else
    {
        //TODO: just reproject the image before cropping
        OE_NOTICE << "[osgEarth::GeoImage::crop] Cropping extent does not have equivalent SpatialReference" << std::endl;
        return GeoImage::INVALID;
    }
}

GeoImage
GeoImage::addTransparentBorder(bool leftBorder, bool rightBorder, bool bottomBorder, bool topBorder)
{
    unsigned int buffer = 1;

    unsigned int newS = _image->s();
    if (leftBorder) newS += buffer;
    if (rightBorder) newS += buffer;

    unsigned int newT = _image->t();
    if (topBorder)    newT += buffer;
    if (bottomBorder) newT += buffer;

    osg::Image* newImage = new osg::Image;
    newImage->allocateImage(newS, newT, _image->r(), _image->getPixelFormat(), _image->getDataType(), _image->getPacking());
    newImage->setInternalTextureFormat(_image->getInternalTextureFormat());
    memset(newImage->data(), 0, newImage->getImageSizeInBytes());
    unsigned startC = leftBorder ? buffer : 0;
    unsigned startR = bottomBorder ? buffer : 0;
    ImageUtils::copyAsSubImage(_image.get(), newImage, startC, startR );

    //double upp = getUnitsPerPixel();
    double uppw = _extent.width() / (double)_image->s();
	double upph = _extent.height() / (double)_image->t();

    double xmin = leftBorder ? _extent.xMin() - buffer * uppw : _extent.xMin();
    double ymin = bottomBorder ? _extent.yMin() - buffer * upph : _extent.yMin();
    double xmax = rightBorder ? _extent.xMax() + buffer * uppw : _extent.xMax();
    double ymax = topBorder ? _extent.yMax() + buffer * upph : _extent.yMax();

    return GeoImage(newImage, GeoExtent(getSRS(), xmin, ymin, xmax, ymax));
}

namespace
{
    osg::Image*
    createImageFromDataset(GDALDataset* ds)
    {
        // called internally -- GDAL lock not required

        int numBands = ds->GetRasterCount();
        if ( numBands < 1 )
            return 0L;

        GLenum dataType;
        int    sampleSize;
        GLint  internalFormat;

        switch(ds->GetRasterBand(1)->GetRasterDataType())
        {
        case GDT_Byte:
            dataType = GL_UNSIGNED_BYTE;
            sampleSize = 1;
            internalFormat = GL_LUMINANCE8;
            break;
        case GDT_UInt16:
        case GDT_Int16:
            dataType = GL_UNSIGNED_SHORT;
            sampleSize = 2;
            internalFormat = GL_LUMINANCE16;
            break;
        default:
            dataType = GL_FLOAT;
            sampleSize = 4;
            internalFormat = GL_LUMINANCE32F_ARB;
        }

        GLenum pixelFormat =
            numBands == 1 ? GL_LUMINANCE :
            numBands == 2 ? GL_LUMINANCE_ALPHA :
            numBands == 3 ? GL_RGB :
                            GL_RGBA;

        int pixelBytes = sampleSize * numBands;

        //Allocate the image
        osg::Image *image = new osg::Image;
        image->allocateImage(ds->GetRasterXSize(), ds->GetRasterYSize(), 1, pixelFormat, dataType);

        CPLErr err = ds->RasterIO(
            GF_Read, 
            0, 0, 
            image->s(), image->t(), 
            (void*)image->data(), 
            image->s(), image->t(), 
            ds->GetRasterBand(1)->GetRasterDataType(),
            numBands,
            NULL,
            pixelBytes,
            pixelBytes * image->s(),
            1);
        if ( err != CE_None )
        {
            OE_WARN << LC << "RasterIO failed.\n";
        }

        ds->FlushCache();

        image->flipVertical();

        return image;
    }

    GDALDataset*
    createMemDS(int width, int height, int numBands, GDALDataType dataType, double minX, double minY, double maxX, double maxY, const std::string &projection)
    {
        //Get the MEM driver
        GDALDriver* memDriver = (GDALDriver*)GDALGetDriverByName("MEM");
        if (!memDriver)
        {
            OE_NOTICE << "[osgEarth::GeoData] Could not get MEM driver" << std::endl;
        }

        //Create the in memory dataset.
        GDALDataset* ds = memDriver->Create("", width, height, numBands, dataType, 0);

        //Initialize the color interpretation
        if ( numBands == 1 )
        {
            ds->GetRasterBand(1)->SetColorInterpretation(GCI_GrayIndex);
        }
        else
        {
            ds->GetRasterBand(1)->SetColorInterpretation(GCI_RedBand);
            ds->GetRasterBand(2)->SetColorInterpretation(GCI_GreenBand);
            ds->GetRasterBand(3)->SetColorInterpretation(GCI_BlueBand);

            if ( numBands == 4 )
            {
                ds->GetRasterBand(4)->SetColorInterpretation(GCI_AlphaBand);
            }
        }

        //Initialize the geotransform
        double geotransform[6];
        double x_units_per_pixel = (maxX - minX) / (double)width;
        double y_units_per_pixel = (maxY - minY) / (double)height;
        geotransform[0] = minX;
        geotransform[1] = x_units_per_pixel;
        geotransform[2] = 0;
        geotransform[3] = maxY;
        geotransform[4] = 0;
        geotransform[5] = -y_units_per_pixel;
        ds->SetGeoTransform(geotransform);
        ds->SetProjection(projection.c_str());

        return ds;
    }

    GDALDataset*
    createDataSetFromImage(const osg::Image* image, double minX, double minY, double maxX, double maxY, const std::string &projection)
    {
        //Clone the incoming image
        osg::ref_ptr<osg::Image> clonedImage = new osg::Image(*image);

        //Flip the image
        clonedImage->flipVertical();

        GDALDataType gdalDataType =
            image->getDataType() == GL_UNSIGNED_BYTE  ? GDT_Byte :
            image->getDataType() == GL_UNSIGNED_SHORT ? GDT_UInt16 :
            image->getDataType() == GL_FLOAT          ? GDT_Float32 :
                                                        GDT_Byte;

        int numBands =
            image->getPixelFormat() == GL_RGBA      ? 4 :
            image->getPixelFormat() == GL_RGB       ? 3 :
            image->getPixelFormat() == GL_LUMINANCE ? 1 : 0;


        if ( numBands == 0 )
        {
            OE_WARN << LC << "Failure in createDataSetFromImage: unsupported pixel format\n";
            return 0L;
        }

        int pixelBytes =
            gdalDataType == GDT_Byte   ?   numBands :
            gdalDataType == GDT_UInt16 ? 2*numBands :
                                         4*numBands;
        
        GDALDataset* srcDS = createMemDS(image->s(), image->t(), numBands, gdalDataType, minX, minY, maxX, maxY, projection);

        if ( srcDS )
        {
            CPLErr err = srcDS->RasterIO(
                GF_Write, 
                0, 0,
                clonedImage->s(), clonedImage->t(),
                (void*)clonedImage->data(),
                clonedImage->s(),
                clonedImage->t(),
                gdalDataType,
                numBands,
                NULL,
                pixelBytes,
                pixelBytes * image->s(),
                1);
            if ( err != CE_None )
            {
                OE_WARN << LC << "RasterIO failed.\n";
            }

            srcDS->FlushCache();
        }

        return srcDS;
    }

    osg::Image*
    reprojectImage(osg::Image* srcImage, const std::string srcWKT, double srcMinX, double srcMinY, double srcMaxX, double srcMaxY,
                   const std::string destWKT, double destMinX, double destMinY, double destMaxX, double destMaxY,
                   int width = 0, int height = 0, bool useBilinearInterpolation = true)
    {
        GDAL_SCOPED_LOCK;
        osg::Timer_t start = osg::Timer::instance()->tick();

        //Create a dataset from the source image
        GDALDataset* srcDS = createDataSetFromImage(srcImage, srcMinX, srcMinY, srcMaxX, srcMaxY, srcWKT);

        OE_DEBUG << LC << "Source image is " << srcImage->s() << "x" << srcImage->t() << " in " << srcWKT << std::endl;


        if (width == 0 || height == 0)
        {
            double outgeotransform[6];
            double extents[4];
            void* transformer = GDALCreateGenImgProjTransformer(srcDS, srcWKT.c_str(), NULL, destWKT.c_str(), 1, 0, 0);
            GDALSuggestedWarpOutput2(srcDS,
                GDALGenImgProjTransform, transformer,
                outgeotransform,
                &width,
                &height,
                extents,
                0);
            GDALDestroyGenImgProjTransformer(transformer);
        }
	    OE_DEBUG << "Creating warped output of " << width <<"x" << height << " in " << destWKT << std::endl;

        int numBands = srcDS->GetRasterCount();
        GDALDataType dataType = srcDS->GetRasterBand(1)->GetRasterDataType();
               
        GDALDataset* destDS = createMemDS(width, height, numBands, dataType, destMinX, destMinY, destMaxX, destMaxY, destWKT);

        if (useBilinearInterpolation == true)
        {
            GDALReprojectImage(srcDS, NULL,
                               destDS, NULL,
                               GRA_Bilinear,
                               0,0,0,0,0);
        }
        else
        {
            GDALReprojectImage(srcDS, NULL,
                               destDS, NULL,
                               GRA_NearestNeighbour,
                               0,0,0,0,0);
        }

        osg::Image* result = createImageFromDataset(destDS);
        
        delete srcDS;
        delete destDS;  

        osg::Timer_t end = osg::Timer::instance()->tick();

        OE_DEBUG << "Reprojected image in " << osg::Timer::instance()->delta_m(start,end) << std::endl;

        return result;
    }    


    osg::Image* manualReproject(
        const osg::Image* image, 
        const GeoExtent&  src_extent, 
        const GeoExtent&  dest_extent,
        bool              interpolate,
        unsigned int      width = 0, 
        unsigned int      height = 0)
    {
        //TODO:  Compute the optimal destination size
        if (width == 0 || height == 0)
        {
            //If no width and height are specified, just use the minimum dimension for the image
            width = osg::minimum(image->s(), image->t());
            height = osg::minimum(image->s(), image->t());
        }

        osg::Image *result = new osg::Image();
        //result->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
        result->allocateImage(width, height, 1, image->getPixelFormat(), image->getDataType()); //GL_UNSIGNED_BYTE);
        result->setInternalTextureFormat(image->getInternalTextureFormat());
        ImageUtils::markAsUnNormalized(result, ImageUtils::isUnNormalized(image));

        //Initialize the image to be completely transparent/black
        memset(result->data(), 0, result->getImageSizeInBytes());

        //ImageUtils::PixelReader ra(result);
        ImageUtils::PixelWriter writer(result);
        const double dx = dest_extent.width() / (double)width;
        const double dy = dest_extent.height() / (double)height;

        // offset the sample points by 1/2 a pixel so we are sampling "pixel center".
        // (This is especially useful in the UnifiedCubeProfile since it nullifes the chances for
        // edge ambiguity.)

        unsigned int numPixels = width * height;

        // Start by creating a sample grid over the destination
        // extent. These will be the source coordinates. Then, reproject
        // the sample grid into the source coordinate system.
        double *srcPointsX = new double[numPixels * 2];
        double *srcPointsY = srcPointsX + numPixels;

        dest_extent.getSRS()->transformExtentPoints(
            src_extent.getSRS(),
            dest_extent.xMin() + .5 * dx, dest_extent.yMin() + .5 * dy,
            dest_extent.xMax() - .5 * dx, dest_extent.yMax() - .5 * dy,
            srcPointsX, srcPointsY, width, height);

        // Next, go through the source-SRS sample grid, read the color at each point from the source image,
        // and write it to the corresponding pixel in the destination image.
        int pixel = 0;
        ImageUtils::PixelReader ia(image);
        double xfac = (image->s() - 1) / src_extent.width();
        double yfac = (image->t() - 1) / src_extent.height();
        for (unsigned int c = 0; c < width; ++c)
        {
            for (unsigned int r = 0; r < height; ++r)
            {   
                double src_x = srcPointsX[pixel];
                double src_y = srcPointsY[pixel];

                if ( src_x < src_extent.xMin() || src_x > src_extent.xMax() || src_y < src_extent.yMin() || src_y > src_extent.yMax() )
                {
                    //If the sample point is outside of the bound of the source extent, increment the pixel and keep looping through.
                    //OE_WARN << LC << "ERROR: sample point out of bounds: " << src_x << ", " << src_y << std::endl;
                    pixel++;
                    continue;
                }

                float px = (src_x - src_extent.xMin()) * xfac;
                float py = (src_y - src_extent.yMin()) * yfac;

                int px_i = osg::clampBetween( (int)osg::round(px), 0, image->s()-1 );
                int py_i = osg::clampBetween( (int)osg::round(py), 0, image->t()-1 );

                osg::Vec4 color(0,0,0,0);

                // TODO: consider this again later. Causes blockiness.
                if ( !interpolate ) //! isSrcContiguous ) // non-contiguous space- use nearest neighbot
                {
                    color = ia(px_i, py_i);
                }

                else // contiguous space - use bilinear sampling
                {
                    int rowMin = osg::maximum((int)floor(py), 0);
                    int rowMax = osg::maximum(osg::minimum((int)ceil(py), (int)(image->t()-1)), 0);
                    int colMin = osg::maximum((int)floor(px), 0);
                    int colMax = osg::maximum(osg::minimum((int)ceil(px), (int)(image->s()-1)), 0);

                    if (rowMin > rowMax) rowMin = rowMax;
                    if (colMin > colMax) colMin = colMax;

                    osg::Vec4 urColor = ia(colMax, rowMax);
                    osg::Vec4 llColor = ia(colMin, rowMin);
                    osg::Vec4 ulColor = ia(colMin, rowMax);
                    osg::Vec4 lrColor = ia(colMax, rowMin);

                    /*Average Interpolation*/
                    /*double x_rem = px - (int)px;
                    double y_rem = py - (int)py;

                    double w00 = (1.0 - y_rem) * (1.0 - x_rem);
                    double w01 = (1.0 - y_rem) * x_rem;
                    double w10 = y_rem * (1.0 - x_rem);
                    double w11 = y_rem * x_rem;
                    double wsum = w00 + w01 + w10 + w11;
                    wsum = 1.0/wsum;

                    color.r() = (w00 * llColor.r() + w01 * lrColor.r() + w10 * ulColor.r() + w11 * urColor.r()) * wsum;
                    color.g() = (w00 * llColor.g() + w01 * lrColor.g() + w10 * ulColor.g() + w11 * urColor.g()) * wsum;
                    color.b() = (w00 * llColor.b() + w01 * lrColor.b() + w10 * ulColor.b() + w11 * urColor.b()) * wsum;
                    color.a() = (w00 * llColor.a() + w01 * lrColor.a() + w10 * ulColor.a() + w11 * urColor.a()) * wsum;*/

                    /*Nearest Neighbor Interpolation*/
                    /*if (px_i >= 0 && px_i < image->s() &&
                    py_i >= 0 && py_i < image->t())
                    {
                    //OE_NOTICE << "[osgEarth::GeoData] Sampling pixel " << px << "," << py << std::endl;
                    color = ImageUtils::getColor(image, px_i, py_i);
                    }
                    else
                    {
                    OE_NOTICE << "[osgEarth::GeoData] Pixel out of range " << px_i << "," << py_i << "  image is " << image->s() << "x" << image->t() << std::endl;
                    }*/

                    /*Bilinear interpolation*/
                    //Check for exact value
                    if ((colMax == colMin) && (rowMax == rowMin))
                    {
                        //OE_NOTICE << "[osgEarth::GeoData] Exact value" << std::endl;
                        color = ia(px_i, py_i);
                    }
                    else if (colMax == colMin)
                    {
                        //OE_NOTICE << "[osgEarth::GeoData] Vertically" << std::endl;
                        //Linear interpolate vertically
                        for (unsigned int i = 0; i < 4; ++i)
                        {
                            color[i] = ((float)rowMax - py) * llColor[i] + (py - (float)rowMin) * ulColor[i];
                        }
                    }
                    else if (rowMax == rowMin)
                    {
                        //OE_NOTICE << "[osgEarth::GeoData] Horizontally" << std::endl;
                        //Linear interpolate horizontally
                        for (unsigned int i = 0; i < 4; ++i)
                        {
                            color[i] = ((float)colMax - px) * llColor[i] + (px - (float)colMin) * lrColor[i];
                        }
                    }
                    else
                    {
                        //OE_NOTICE << "[osgEarth::GeoData] Bilinear" << std::endl;
                        //Bilinear interpolate
                        float col1 = colMax - px, col2 = px - colMin;
                        float row1 = rowMax - py, row2 = py - rowMin;
                        for (unsigned int i = 0; i < 4; ++i)
                        {
                            float r1 = col1 * llColor[i] + col2 * lrColor[i];
                            float r2 = col1 * ulColor[i] + col2 * urColor[i];

                            //OE_INFO << "r1, r2 = " << r1 << " , " << r2 << std::endl;
                            color[i] = row1 * r1 + row2 * r2;
                        }
                    }
                }

                writer(color, c, r);
                pixel++;
            }
        }

        delete[] srcPointsX;

        return result;
    }
}

GeoImage
GeoImage::reproject(const SpatialReference* to_srs, const GeoExtent* to_extent, unsigned int width, unsigned int height, bool useBilinearInterpolation) const
{  
    GeoExtent destExtent;
    if (to_extent)
    {
        destExtent = *to_extent;
    }
    else
    {
        destExtent = getExtent().transform(to_srs);    
    }

    osg::Image* resultImage = 0L;

    bool isNormalized = ImageUtils::isNormalized(getImage());
    
    if ( getSRS()->isUserDefined()      || 
        to_srs->isUserDefined()         ||
        getSRS()->isSphericalMercator() ||
        to_srs->isSphericalMercator()   ||
        !isNormalized )
    {
        // if either of the SRS is a custom projection, we have to do a manual reprojection since
        // GDAL will not recognize the SRS.
        resultImage = manualReproject(getImage(), getExtent(), destExtent, useBilinearInterpolation && isNormalized, width, height);
    }
    else
    {
        // otherwise use GDAL.
        resultImage = reprojectImage(
            getImage(),
            getSRS()->getWKT(),
            getExtent().xMin(), getExtent().yMin(), getExtent().xMax(), getExtent().yMax(),
            to_srs->getWKT(),
            destExtent.xMin(), destExtent.yMin(), destExtent.xMax(), destExtent.yMax(),
            width, height, useBilinearInterpolation);
    }   
    return GeoImage(resultImage, destExtent);
}

void
GeoImage::applyAlphaMask(const GeoExtent& maskingExtent)
{
    if ( !valid() )
        return;

    GeoExtent maskingExtentLocal = maskingExtent.transform(_extent.getSRS());

    // if the image is completely contains by the mask, no work to do.
    if ( maskingExtentLocal.contains(getExtent()))
        return;

    // TODO: find a more performant way about this 
    ImageUtils::PixelReader read (_image.get());
    ImageUtils::PixelWriter write(_image.get());

    double sInterval = _extent.width()/(double)_image->s();
    double tInterval = _extent.height()/(double)_image->t();

    for( int t=0; t<_image->t(); ++t )
    {
        double y = _extent.south() + tInterval*(double)t;

        for( int s=0; s<_image->s(); ++s )
        {
            double x = _extent.west() + sInterval*(double)s;

            for( int r=0; r<_image->r(); ++r )
            {
                if ( !maskingExtentLocal.contains(x, y) )
                {
                    osg::Vec4f pixel = read(s,t,r);
                    write(osg::Vec4f(pixel.r(), pixel.g(), pixel.b(), 0.0f), s, t, r);
                }
            }
        }
    }
}

osg::Image*
GeoImage::takeImage()
{
    return _image.release();
}

/***************************************************************************/

#define DEFAULT_NORMAL osg::Vec3(0,0,1)
#define DEFAULT_CURVATURE 0.0f

NormalMap::NormalMap(unsigned s, unsigned t) :
osg::Image(),
_write(0L),
_read(0L)
{
    const osg::Vec3 defaultNormal(DEFAULT_NORMAL);
    const float defaultCurvature(DEFAULT_CURVATURE);

    if ( s > 0 && t > 0 )
    {
        allocateImage(s, t, 1, GL_RGBA, GL_UNSIGNED_BYTE, 1);

        _write = new ImageUtils::PixelWriter(this);
        _read = new ImageUtils::PixelReader(this);

        for (unsigned y=0; y<t; ++y)
            for (unsigned x=0; x<s; ++x)
                set(x, y, defaultNormal, defaultCurvature);
    }
}

NormalMap::~NormalMap()
{
    if (_read) delete _read;
    if (_write) delete _write;
}

void
NormalMap::set(unsigned s, unsigned t, const osg::Vec3& normal, float curvature)
{
    if (!_write) return;

    osg::Vec4f encoding(
        0.5f*(normal.x()+1.0f),
        0.5f*(normal.y()+1.0f),
        0.5f*(normal.z()+1.0f),
        0.5f*(curvature+1.0f));

    (*_write)(encoding, s, t);
}

osg::Vec3
NormalMap::getNormal(unsigned s, unsigned t) const
{
    if (!_read) return osg::Vec3(0,0,1);

    osg::Vec4 encoding = (*_read)(s, t);
    return osg::Vec3(
        encoding.x()*2.0 - 1.0,
        encoding.y()*2.0 - 1.0,
        encoding.z()*2.0 - 1.0);
}

osg::Vec3
NormalMap::getNormalByUV(double u, double v) const
{
    if (!_read) return osg::Vec3(0,0,1);

    double c = u * (double)(s()-1);
    double r = v * (double)(t()-1);

    unsigned rowMin = osg::maximum((int)floor(r), 0);
    unsigned rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(t() - 1)), 0);
    unsigned colMin = osg::maximum((int)floor(c), 0);
    unsigned colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(s() - 1)), 0);
    
    if (rowMin > rowMax) rowMin = rowMax;
    if (colMin > colMax) colMin = colMax;

    osg::Vec3 ur = getNormal(colMax, rowMax);
    osg::Vec3 ll = getNormal(colMin, rowMin);
    osg::Vec3 ul = getNormal(colMin, rowMax);
    osg::Vec3 lr = getNormal(colMax, rowMin);

    osg::Vec3 result;

    // Bilinear:
    //if (interpolation == INTERP_BILINEAR)
    {
        //Check for exact value
        if ((colMax == colMin) && (rowMax == rowMin))
        {
            // exact
            result = getNormal(colMin, rowMin);
        }
        else if (colMax == colMin)
        {
            //Linear interpolate vertically
            result = ll*((double)rowMax - r) + ul*(r - (double)rowMin);
        }
        else if (rowMax == rowMin)
        {
            //Linear interpolate horizontally
            result = ll*((double)colMax - c) + lr*(c - (double)colMin);
        }
        else
        {
            //Bilinear interpolate
            osg::Vec3 n1 = ll*((double)colMax - c) + lr*(c - (double)colMin);
            osg::Vec3 n2 = ul*((double)colMax - c) + ur*(c - (double)colMin);
            result = n1*((double)rowMax - r) + n2*(r - (double)rowMin);
        }
    }

    result.normalize();
    return result;
}

float
NormalMap::getCurvature(unsigned s, unsigned t) const
{
    if (!_read) return 0.0f;
    return (*_read)(s, t).a() * 2.0f - 1.0f;
}

/***************************************************************************/

#undef  LC
#define LC "[GeoHeightField] "

// static
GeoHeightField GeoHeightField::INVALID( 0L, GeoExtent::INVALID );

GeoHeightField::GeoHeightField() :
_heightField( 0L ),
_extent     ( GeoExtent::INVALID ),
_minHeight  ( 0.0f ),
_maxHeight  ( 0.0f )
{
    //nop
}

GeoHeightField::GeoHeightField(osg::HeightField* heightField,
                               const GeoExtent&  extent) :
_heightField( heightField ),
_extent     ( extent ),
_minHeight( FLT_MAX ),
_maxHeight( -FLT_MAX )
{
    init();
}

GeoHeightField::GeoHeightField(osg::HeightField* heightField,
                               NormalMap*        normalMap,
                               const GeoExtent&  extent) :
_heightField( heightField ),
_normalMap  ( normalMap ),
_extent     ( extent ),
_minHeight  ( FLT_MAX ),
_maxHeight  ( -FLT_MAX )
{
    init();
}

void
GeoHeightField::init()
{
    if ( _heightField.valid() && _extent.isInvalid() )
    {
        OE_WARN << LC << "Created with a valid heightfield AND INVALID extent" << std::endl;
    }

    else if ( _heightField.valid() )
    {
        double minx, miny, maxx, maxy;
        _extent.getBounds(minx, miny, maxx, maxy);

        _heightField->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
        _heightField->setXInterval( (maxx - minx)/(double)(_heightField->getNumColumns()-1) );
        _heightField->setYInterval( (maxy - miny)/(double)(_heightField->getNumRows()-1) );
        _heightField->setBorderWidth( 0 );

        const osg::HeightField::HeightList& heights = _heightField->getHeightList();
        for( unsigned i=0; i<heights.size(); ++i )
        {
            float h = heights[i];
            if ( h > _maxHeight ) _maxHeight = h;
            if ( h < _minHeight ) _minHeight = h;
        }
    }
}

bool
GeoHeightField::valid() const
{
    return _heightField.valid() && _extent.isValid();
}

float
GeoHeightField::getElevation(double x, double y) const
{
    return HeightFieldUtils::getHeightAtLocation(
        _heightField.get(),
        x, y,
        _extent.xMin(), _extent.yMin(),
        _heightField->getXInterval(), _heightField->getYInterval(),
        INTERP_BILINEAR);
}

osg::Vec3
GeoHeightField::getNormal(double x, double y) const
{
    return !_normalMap.valid() ? osg::Vec3(0,0,1) :
        _normalMap->getNormalByUV((x - _extent.xMin()) / _extent.width(), (y - _extent.yMin()) / _extent.height());
}

bool
GeoHeightField::getElevation(const SpatialReference* inputSRS, 
                             double                  x, 
                             double                  y, 
                             ElevationInterpolation  interp,
                             const SpatialReference* outputSRS,
                             float&                  out_elevation) const
{
    osg::Vec3d xy(x, y, 0);
    osg::Vec3d local = xy;
    const SpatialReference* extentSRS = _extent.getSRS();


    // first xform the input point into our local SRS:
    if (inputSRS != extentSRS)
    {
        if (inputSRS && !inputSRS->transform(xy, extentSRS, local))
            return false;
    }

    // check that the point falls within the heightfield bounds:
    if ( _extent.contains(local.x(), local.y()) )
    {
        double xInterval = _extent.width()  / (double)(_heightField->getNumColumns()-1);
        double yInterval = _extent.height() / (double)(_heightField->getNumRows()-1);

        // sample the heightfield at the input coordinates:
        // (note: since it's sampling the HF, it will return an MSL height if applicable)
        out_elevation = HeightFieldUtils::getHeightAtLocation(
            _heightField.get(), 
            local.x(), local.y(),
            _extent.xMin(), _extent.yMin(), 
            xInterval, yInterval, 
            interp);

        // if the vertical datums don't match, do a conversion:
        if (out_elevation != NO_DATA_VALUE && 
            outputSRS && 
            !extentSRS->isVertEquivalentTo(outputSRS) )
        {
            // if the caller provided a custom output SRS, perform the appropriate
            // Z transformation. This requires a lat/long point:

            osg::Vec3d geolocal(local);
            if ( !extentSRS->isGeographic() )
            {
                extentSRS->transform(geolocal, extentSRS->getGeographicSRS(), geolocal);
            }

            VerticalDatum::transform(
                extentSRS->getVerticalDatum(),
                outputSRS->getVerticalDatum(),
                geolocal.y(), geolocal.x(), out_elevation);
        }

        return true;
    }
    else
    {
        out_elevation = 0.0f;
        return false;
    }
}

bool
GeoHeightField::getElevationAndNormal(const SpatialReference* inputSRS,
                                      double                  x,
                                      double                  y,
                                      ElevationInterpolation  interp,
                                      const SpatialReference* outputSRS,
                                      float&                  out_elevation,
                                      osg::Vec3f&             out_normal) const
{
    osg::Vec3d xy(x, y, 0);
    osg::Vec3d local = xy;
    const SpatialReference* extentSRS = _extent.getSRS();


    // first xform the input point into our local SRS:
    if ( inputSRS && !inputSRS->transform(xy, extentSRS, local) )
        return false;

    // check that the point falls within the heightfield bounds:
    if ( _extent.contains(local.x(), local.y()) )
    {
        double xInterval = _extent.width()  / (double)(_heightField->getNumColumns()-1);
        double yInterval = _extent.height() / (double)(_heightField->getNumRows()-1);

        // sample the heightfield at the input coordinates:
        // (note: since it's sampling the HF, it will return an MSL height if applicable)
        out_elevation = HeightFieldUtils::getHeightAtLocation(
            _heightField.get(), 
            local.x(), local.y(),
            _extent.xMin(), _extent.yMin(), 
            xInterval, yInterval, 
            interp);

        // if the vertical datums don't match, do a conversion:
        if (out_elevation != NO_DATA_VALUE && 
            outputSRS && 
            !extentSRS->isVertEquivalentTo(outputSRS) )
        {
            // if the caller provided a custom output SRS, perform the appropriate
            // Z transformation. This requires a lat/long point:

            osg::Vec3d geolocal(local);
            if ( !extentSRS->isGeographic() )
            {
                extentSRS->transform(geolocal, extentSRS->getGeographicSRS(), geolocal);
            }

            VerticalDatum::transform(
                extentSRS->getVerticalDatum(),
                outputSRS->getVerticalDatum(),
                geolocal.y(), geolocal.x(), out_elevation);
        }

        // If we have a normal map, use it; if not, attempt to generate a normal
        // by sampling the heightfield.
        if (_normalMap.valid())
        {
            //OE_INFO << "Normal Map Exists\n";
            double nx = osg::clampBetween((local.x() - _extent.xMin()) / _extent.width(), 0.0, 1.0);
            double ny = osg::clampBetween((local.y() - _extent.yMin()) / _extent.height(), 0.0, 1.0);
            out_normal = _normalMap->getNormalByUV(nx, ny);
        }
        else
        {
            HeightFieldNeighborhood hood;
            hood.setNeighbor(0, 0, _heightField.get());

            // calculate the normal at the same location:
            out_normal = HeightFieldUtils::getNormalAtLocation(
                hood,
                local.x(), local.y(),
                _extent.xMin(), _extent.yMin(), 
                xInterval, yInterval);
        }

        return true;
    }
    else
    {
        out_elevation = 0.0f;
        out_normal.set(0.0f, 0.0f, 1.0f);
        return false;
    }
}

GeoHeightField
GeoHeightField::createSubSample( const GeoExtent& destEx, unsigned int width, unsigned int height, ElevationInterpolation interpolation) const
{
    double div = destEx.width()/_extent.width();
    if ( div >= 1.0f )
        return GeoHeightField::INVALID;

    double dx = destEx.width()/(double)(width-1);
    double dy = destEx.height()/(double)(height-1);    

    osg::HeightField* dest = new osg::HeightField();
    dest->allocate( width, height );
    dest->setXInterval( dx );
    dest->setYInterval( dy );

    double x, y;
    int col, row;

    double x0 = (destEx.xMin()-_extent.xMin())/_extent.width();
    double y0 = (destEx.yMin()-_extent.yMin())/_extent.height();

    double xstep = div / (double)(width-1);
    double ystep = div / (double)(height-1);
    
    for( x = x0, col = 0; col < (int)width; x += xstep, col++ )
    {
        for( y = y0, row = 0; row < (int)height; y += ystep, row++ )
        {
            float height = HeightFieldUtils::getHeightAtNormalizedLocation(
                _heightField.get(), x, y, interpolation );
            dest->setHeight( col, row, height );
        }
    }

    return GeoHeightField( dest, destEx ); // Q: is the VDATUM accounted for?
}

const GeoExtent&
GeoHeightField::getExtent() const
{
    return _extent;
}

const osg::HeightField*
GeoHeightField::getHeightField() const
{
    return _heightField.get();
}

const NormalMap*
GeoHeightField::getNormalMap() const
{
    return _normalMap.get();
}

double
GeoHeightField::getXInterval() const
{
    return _extent.width()  / (double)(_heightField->getNumColumns()-1);        
}

double
GeoHeightField::getYInterval() const
{
    return _extent.height() / (double)(_heightField->getNumRows()-1);
}


