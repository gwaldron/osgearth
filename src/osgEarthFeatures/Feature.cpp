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
#include <osgEarthFeatures/Feature>
#include <osgEarth/StringUtils>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[Feature] "

//----------------------------------------------------------------------------

FeatureProfile::FeatureProfile( const GeoExtent& extent ) :
_extent    ( extent ),
_firstLevel( 0 ),
_maxLevel  ( -1 ),
_tiled     ( false )
{
    //nop
}

bool
FeatureProfile::getTiled() const
{
    return _tiled;
}

void
FeatureProfile::setTiled(bool tiled)
{
    _tiled = true;
}

int
FeatureProfile::getFirstLevel() const
{
    return _firstLevel;
}

void
FeatureProfile::setFirstLevel(int firstLevel )
{
    _firstLevel = firstLevel;
}

int
FeatureProfile::getMaxLevel() const
{
    return _maxLevel;
}

void
FeatureProfile::setMaxLevel(int maxLevel)
{
    _maxLevel = maxLevel;
}

const osgEarth::Profile* 
FeatureProfile::getProfile() const
{
    return _profile.get();
}

void
FeatureProfile::setProfile( const osgEarth::Profile* profile )
{
    _profile = profile;
}

//----------------------------------------------------------------------------

std::string
AttributeValue::getString() const
{
    switch( first ) {
        case ATTRTYPE_STRING: return second.stringValue;
        case ATTRTYPE_DOUBLE: return osgEarth::toString(second.doubleValue);
        case ATTRTYPE_INT:    return osgEarth::toString(second.intValue);
        case ATTRTYPE_BOOL:   return osgEarth::toString(second.boolValue);
        case ATTRTYPE_UNSPECIFIED: break;
    }
    return EMPTY_STRING;
}

double
AttributeValue::getDouble( double defaultValue ) const 
{
    switch( first ) {
        case ATTRTYPE_STRING: return osgEarth::as<double>(second.stringValue, defaultValue);
        case ATTRTYPE_DOUBLE: return second.doubleValue;
        case ATTRTYPE_INT:    return (double)second.intValue;
        case ATTRTYPE_BOOL:   return second.boolValue? 1.0 : 0.0;
        case ATTRTYPE_UNSPECIFIED: break;
    }
    return defaultValue;
}

int
AttributeValue::getInt( int defaultValue ) const 
{
    switch( first ) {
        case ATTRTYPE_STRING: return osgEarth::as<int>(second.stringValue, defaultValue);
        case ATTRTYPE_DOUBLE: return (int)second.doubleValue;
        case ATTRTYPE_INT:    return second.intValue;
        case ATTRTYPE_BOOL:   return second.boolValue? 1 : 0;
        case ATTRTYPE_UNSPECIFIED: break;
    }
    return defaultValue;
}

bool
AttributeValue::getBool( bool defaultValue ) const 
{
    switch( first ) {
        case ATTRTYPE_STRING: return osgEarth::as<bool>(second.stringValue, defaultValue);
        case ATTRTYPE_DOUBLE: return second.doubleValue != 0.0;
        case ATTRTYPE_INT:    return second.intValue != 0;
        case ATTRTYPE_BOOL:   return second.boolValue;
        case ATTRTYPE_UNSPECIFIED: break;
    }
    return defaultValue;
}

//----------------------------------------------------------------------------

Feature::Feature( FeatureID fid ) :
_fid( fid ),
_srs( 0L ),
_cachedBoundingPolytopeValid( false )
{
    //NOP
}

Feature::Feature( Geometry* geom, const SpatialReference* srs, const Style& style, FeatureID fid ) :
_geom ( geom ),
_srs  ( srs ),
_fid  ( fid )
{
    if ( !style.empty() )
        _style = style;

    dirty();
}

Feature::Feature( const Feature& rhs, const osg::CopyOp& copyOp ) :
_fid      ( rhs._fid ),
_attrs    ( rhs._attrs ),
_style    ( rhs._style ),
_geoInterp( rhs._geoInterp ),
_srs      ( rhs._srs.get() )
{
    if ( rhs._geom.valid() )
        _geom = rhs._geom->clone();

    dirty();
}

FeatureID
Feature::getFID() const 
{
    return _fid;
}

void
Feature::setSRS( const SpatialReference* srs )
{
    _srs = srs;
    dirty();
}

void
Feature::setGeometry( Geometry* geom )
{
    _geom = geom;
    dirty();
}

void
Feature::dirty()
{
    _cachedExtent = GeoExtent::INVALID;
    _cachedGeocentricBound._radius = -1.0; // invalidate
    _cachedBoundingPolytopeValid = false;
}

void
Feature::set( const std::string& name, const std::string& value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_STRING;
    a.second.stringValue = value;
}

void
Feature::set( const std::string& name, double value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_DOUBLE;
    a.second.doubleValue = value;
}

void
Feature::set( const std::string& name, int value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_INT;
    a.second.intValue = value;
}

void
Feature::set( const std::string& name, bool value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_BOOL;
    a.second.boolValue = value;
}

bool
Feature::hasAttr( const std::string& name ) const
{
    return _attrs.find(toLower(name)) != _attrs.end();
}

std::string
Feature::getString( const std::string& name ) const
{
    AttributeTable::const_iterator i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getString() : EMPTY_STRING;
}

double
Feature::getDouble( const std::string& name, double defaultValue ) const 
{
    AttributeTable::const_iterator i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getDouble(defaultValue) : defaultValue;
}

int
Feature::getInt( const std::string& name, int defaultValue ) const 
{
    AttributeTable::const_iterator i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getInt(defaultValue) : defaultValue;
}

bool
Feature::getBool( const std::string& name, bool defaultValue ) const 
{
    AttributeTable::const_iterator i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getBool(defaultValue) : defaultValue;
}

double
Feature::eval( NumericExpression& expr, FilterContext const* context ) const
{
    const NumericExpression::Variables& vars = expr.variables();
    for( NumericExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i )
    {
      double val = 0.0;
      AttributeTable::const_iterator ai = _attrs.find(toLower(i->first));
      if (ai != _attrs.end())
      {
        val = ai->second.getDouble(0.0);
      }
      else if (context)
      {
        //No attr found, look for script
        ScriptEngine* engine = context->getSession()->getScriptEngine();
        if (engine)
        {
          ScriptResult result = engine->run(i->first, this, context);
          if (result.success())
            val = result.asDouble();
          else
              OE_WARN << LC << "Script error:" << result.message() << std::endl;
        }
      }

      expr.set( *i, val); //osgEarth::as<double>(getAttr(i->first),0.0) );
    }

    return expr.eval();
}

const std::string&
Feature::eval( StringExpression& expr, FilterContext const* context ) const
{
    const StringExpression::Variables& vars = expr.variables();
    for( StringExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i )
    {
      std::string val = "";
      AttributeTable::const_iterator ai = _attrs.find(toLower(i->first));
      if (ai != _attrs.end())
      {
        val = ai->second.getString();
      }
      else if (context)
      {
        //No attr found, look for script
        ScriptEngine* engine = context->getSession()->getScriptEngine();
        if (engine)
        {
          ScriptResult result = engine->run(i->first, this, context);
          if (result.success())
            val = result.asString();
          else
              OE_WARN << LC << "Script error:" << result.message() << std::endl;
        }
      }

      if (!val.empty())
        expr.set( *i, val );
    }

    return expr.eval();
}

#if 0
#define SIGN_OF(x) double(int(x > 0.0) - int(x < 0.0))

//TODO:
// This almost works but not quite. There are 2 more things required to make it work
// (I think).
//
// First, GeoExtent.expandToInclude has no knowledge of connectivity, so
// it can expand the extent in the wrong direction if the feature goes around the globe.
// So it needs another function that helps preserve connectivity by taking the "previous"
// point along with the new point, calculating the extent of those 2 points, and unioning
// that with the existing extent.
//
// Second, this needs support for polygons, whose extent includes the interior of the poly.
// For this we can calculate the geocentric bbox of the ECEF feature points, and use the 
// min/max Z of that box to determine the min/max latitude of the polygon extent.

const GeoExtent&
Feature::getExtent() const
{
    if ( !_cachedExtent.isValid() )
    {
        if ( getGeometry() && getSRS() )
        {
            if ( getSRS()->isGeographic() )
            {
                GeoExtent e( getSRS() );

                if ( _geoInterp.value() == GEOINTERP_GREAT_CIRCLE )
                {
                    const osg::EllipsoidModel* em = getSRS()->getEllipsoid();

                    // find the GC "cutting plane" with the greatest inclination.
                    ConstSegmentIterator i( getGeometry() );
                    while( i.hasMore() )
                    {
                        Segment s = i.next();

                        double minLat, maxLat;

                        GeoMath::greatCircleMinMaxLatitude( 
                            osg::DegreesToRadians(s.first.y()), osg::DegreesToRadians(s.first.x()),
                            osg::DegreesToRadians(s.second.y()), osg::DegreesToRadians(s.second.x()),
                            minLat, maxLat);

                        minLat = osg::RadiansToDegrees( minLat );
                        maxLat = osg::RadiansToDegrees( maxLat );

                        e.expandToInclude( s.first.x(), minLat );
                        e.expandToInclude( s.second.x(), maxLat );
                        //e.expandToInclude( e.getCentroid().x(), std::min(minLat, e.south()) );
                        //e.expandToInclude( e.getCentroid().x(), std::max(maxLat, e.north()) );
                    }
                }
                else // RHUMB_LINE
                {
                    ConstGeometryIterator i( getGeometry(), true );
                    while( i.hasMore() )
                    {
                        const Geometry* g = i.next();
                        for( Geometry::const_iterator v = g->begin(); v != g->end(); ++v )
                            e.expandToInclude( v->x(), v->y() );
                    }
                }

                const_cast<Feature*>(this)->_cachedExtent = e;
            }
            else
            {
                const_cast<Feature*>(this)->_cachedExtent = GeoExtent(getSRS(), getGeometry()->getBounds());
            }
        }
    }
    return _cachedExtent;
}
#endif


const osg::BoundingSphere&
Feature::getGeocentricBound() const
{
    if ( !_cachedGeocentricBound.valid() )
    {
        if ( getSRS() && getGeometry() )
        {
            ConstGeometryIterator i( getGeometry(), false); 
            while( i.hasMore() )
            {
                const Geometry* g = i.next();
                for( Geometry::const_iterator p = g->begin(); p != g->end(); ++p )
                {
                    osg::Vec3d ecef;
                    getSRS()->transformToECEF( *p, ecef );
                    const_cast<Feature*>(this)->_cachedGeocentricBound.expandBy( ecef );
                }
            }
            if ( _cachedGeocentricBound.valid() && _cachedGeocentricBound.radius() == 0.0 )
            {
                const_cast<Feature*>(this)->_cachedGeocentricBound.radius() = 1.0;
            }
        }
    }
    return _cachedGeocentricBound;
}

const osg::Polytope&
Feature::getWorldBoundingPolytope() const
{
    if ( !_cachedBoundingPolytopeValid )
    {
        const osg::BoundingSphere& bs = getGeocentricBound();
        if ( bs.valid() )
        {
            const osg::EllipsoidModel* e = getSRS()->getEllipsoid();

            osg::Polytope& p = const_cast<osg::Polytope&>(_cachedBoundingPolytope);
            p.clear();

            // add planes for the four sides of the BS (in local space). Normals point inwards.
            p.add( osg::Plane(osg::Vec3d( 1, 0,0), osg::Vec3d(-bs.radius(),0,0)) );
            p.add( osg::Plane(osg::Vec3d(-1, 0,0), osg::Vec3d( bs.radius(),0,0)) );
            p.add( osg::Plane(osg::Vec3d( 0, 1,0), osg::Vec3d(0, -bs.radius(),0)) );
            p.add( osg::Plane(osg::Vec3d( 0,-1,0), osg::Vec3d(0,  bs.radius(),0)) );

            // for a projected feature, we're done. For a geocentric one, transform the polytope
            // into world (ECEF) space.
            if ( getSRS()->isGeographic() )
            {
                // add a bottom cap, unless the bounds are sufficiently large.
                double minRad = std::min(e->getRadiusPolar(), e->getRadiusEquator());
                double maxRad = std::max(e->getRadiusPolar(), e->getRadiusEquator());
                double zeroOffset = bs.center().length();
                if ( zeroOffset > minRad * 0.1 )
                {
                    p.add( osg::Plane(osg::Vec3d(0,0,1), osg::Vec3d(0,0,-maxRad+zeroOffset)) );
                }

                // transform the clipping planes ito ECEF space
                osg::Matrix local2world;
                getSRS()->getEllipsoid()->computeLocalToWorldTransformFromXYZ( 
                    bs.center().x(), bs.center().y(), bs.center().z(),
                    local2world );
                p.transform( local2world );
            }

            const_cast<Feature*>(this)->_cachedBoundingPolytopeValid = true;
        }
    }
    return _cachedBoundingPolytope;
}
