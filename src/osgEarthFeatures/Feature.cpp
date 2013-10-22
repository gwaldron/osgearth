/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarth/JsonUtils>
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
_srs( 0L )
//_cachedBoundingPolytopeValid( false )
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
    //_cachedGeocentricBound._radius = -1.0; // invalidate
    //_cachedBoundingPolytopeValid = false;
}

void
Feature::set( const std::string& name, const std::string& value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_STRING;
    a.second.stringValue = value;
    a.second.set = true;
}

void
Feature::set( const std::string& name, double value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_DOUBLE;
    a.second.doubleValue = value;
    a.second.set = true;
}

void
Feature::set( const std::string& name, int value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_INT;
    a.second.intValue = value;
    a.second.set = true;
}

void
Feature::set( const std::string& name, bool value )
{
    AttributeValue& a = _attrs[name];
    a.first = ATTRTYPE_BOOL;
    a.second.boolValue = value;
    a.second.set = true;
}

void
Feature::setNull( const std::string& name)
{
    AttributeValue& a = _attrs[name];    
    a.second.set = false;
}

void
Feature::setNull( const std::string& name, AttributeType type)
{
    AttributeValue& a = _attrs[name];
    a.first = type;    
    a.second.set = false;
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

bool
Feature::isSet( const std::string& name) const
{
    AttributeTable::const_iterator i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.second.set : false;
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

      expr.set( *i, val );
    }

    return expr.eval();
}


bool
Feature::getWorldBound(const SpatialReference* srs,
                       osg::BoundingSphered&   out_bound) const
{
    if ( srs && getSRS() && getGeometry() )
    {
        out_bound.init();

        ConstGeometryIterator i( getGeometry(), false); 
        while( i.hasMore() )
        {
            const Geometry* g = i.next();
            for( Geometry::const_iterator p = g->begin(); p != g->end(); ++p )
            {
                GeoPoint point( getSRS(), *p, ALTMODE_ABSOLUTE );
                GeoPoint srs_point;
                if ( point.transform( srs, srs_point ) )
                {
                    osg::Vec3d world;
                    srs_point.toWorld(world);
                    out_bound.expandBy( world );
                }
            }
        }
        if ( out_bound.valid() && out_bound.radius() == 0.0 )
        {
            out_bound.radius() = 1.0;
        }
        return true;
    }
    return false;
}


bool
Feature::getWorldBoundingPolytope(const SpatialReference* srs,
                                  osg::Polytope&          out_polytope) const
{
    osg::BoundingSphered bs;
    if ( getWorldBound(srs, bs) && bs.valid() )
    {
        out_polytope.clear();

        // add planes for the four sides of the BS. Normals point inwards.
        out_polytope.add( osg::Plane(osg::Vec3d( 1, 0,0), osg::Vec3d(-bs.radius(),0,0)) );
        out_polytope.add( osg::Plane(osg::Vec3d(-1, 0,0), osg::Vec3d( bs.radius(),0,0)) );
        out_polytope.add( osg::Plane(osg::Vec3d( 0, 1,0), osg::Vec3d(0, -bs.radius(),0)) );
        out_polytope.add( osg::Plane(osg::Vec3d( 0,-1,0), osg::Vec3d(0,  bs.radius(),0)) );

        // for a projected feature, we're done. For a geocentric one, transform the polytope
        // into world (ECEF) space.
        if ( srs->isGeographic() && !srs->isPlateCarre() )
        {
            const osg::EllipsoidModel* e = srs->getEllipsoid();

            // add a bottom cap, unless the bounds are sufficiently large.
            double minRad = std::min(e->getRadiusPolar(), e->getRadiusEquator());
            double maxRad = std::max(e->getRadiusPolar(), e->getRadiusEquator());
            double zeroOffset = bs.center().length();
            if ( zeroOffset > minRad * 0.1 )
            {
                out_polytope.add( osg::Plane(osg::Vec3d(0,0,1), osg::Vec3d(0,0,-maxRad+zeroOffset)) );
            }
        }

        // transform the clipping planes ito ECEF space
        GeoPoint refPoint;
        refPoint.fromWorld( srs, bs.center() );

        osg::Matrix local2world;
        refPoint.createLocalToWorld( local2world );

        out_polytope.transform( local2world );

        return true;
    }
    return false;
}


std::string
Feature::getGeoJSON()
{
    std::string geometry = GeometryUtils::geometryToGeoJSON( getGeometry() );

    Json::Value root(Json::objectValue);
    root["type"] = "Feature";
    root["id"] = (unsigned int)getFID(); //TODO:  Update JSON to use unsigned longs
    
    Json::Reader reader;
    Json::Value geometryValue( Json::objectValue );
    if ( reader.parse( geometry, geometryValue ) )
    {
        root["geometry"] = geometryValue;
    }

    //Write out all the properties         
    Json::Value props(Json::objectValue);    
    if (getAttrs().size() > 0)
    {

        for (AttributeTable::const_iterator itr = getAttrs().begin(); itr != getAttrs().end(); ++itr)
        {
            if (itr->second.first == ATTRTYPE_INT)
            {
                if (itr->second.second.set)
                {
                    props[itr->first] = itr->second.getInt();
                }
                else
                {
                    props[itr->first] = Json::nullValue;
                }
            }
            else if (itr->second.first == ATTRTYPE_DOUBLE)
            {
                if (itr->second.second.set)
                {
                    props[itr->first] = itr->second.getDouble();
                }
                else
                {
                    props[itr->first] = Json::nullValue;
                }
            }
            else if (itr->second.first == ATTRTYPE_BOOL)
            {
                if (itr->second.second.set)
                {
                    props[itr->first] = itr->second.getBool();
                }
                else
                {
                    props[itr->first] = Json::nullValue;
                }
            }
            else
            {
                if (itr->second.second.set)
                {
                    props[itr->first] = itr->second.getString();
                }
                else
                {
                    props[itr->first] = Json::nullValue;
                }
            }            
        }
    } 

    root["properties"] = props;
    return Json::FastWriter().write( root );
    //return Json::StyledWriter().write( root );
}

std::string Feature::featuresToGeoJSON( FeatureList& features)
{
    std::stringstream buf;

    buf << "{\"type\": \"FeatureCollection\", \"features\": [";

    FeatureList::iterator last = features.end();
    last--;

    for (FeatureList::iterator i = features.begin(); i != features.end(); i++)
    {
        buf << i->get()->getGeoJSON();
        if (i != last)
        {
            buf << ",";
        }
    }

    buf << "]}";

    return buf.str();

}

void Feature::transform( const SpatialReference* srs )
{
    if (!getGeometry())
    {
        return;
    }

    if (getSRS()->isEquivalentTo( srs )) return;

    // iterate over the feature geometry.
    GeometryIterator iter( getGeometry() );
    while( iter.hasMore() )
    {
        Geometry* geom = iter.next();
        getSRS()->transform( geom->asVector(), srs );
    }
    setSRS( srs );
}
