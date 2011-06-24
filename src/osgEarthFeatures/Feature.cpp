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
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

static
std::string EMPTY_STRING;

//----------------------------------------------------------------------------

FeatureProfile::FeatureProfile( const GeoExtent& extent ) :
_extent( extent ),
_firstLevel(0),
_maxLevel(-1),
_tiled(false)
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
    }
    return defaultValue;
}

//----------------------------------------------------------------------------

Feature::Feature( FeatureID fid ) :
_fid( fid )
{
    //NOP
}

Feature::Feature( const Feature& rhs, const osg::CopyOp& copyOp ) :
_fid( rhs._fid ),
_attrs( rhs._attrs ),
_style( rhs._style )
{
    if ( rhs._geom.valid() )
        _geom = dynamic_cast<Geometry*>( copyOp( rhs._geom.get() ) );
}

FeatureID
Feature::getFID() const 
{
    return _fid;
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
Feature::eval( NumericExpression& expr ) const
{
    const NumericExpression::Variables& vars = expr.variables();
    for( NumericExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i )
        expr.set( *i, getDouble(i->first, 0.0)); //osgEarth::as<double>(getAttr(i->first),0.0) );
    return expr.eval();
}

const std::string&
Feature::eval( StringExpression& expr ) const
{
    const StringExpression::Variables& vars = expr.variables();
    for( StringExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i )
        expr.set( *i, getString(i->first) ); //getAttr(i->first) );
    return expr.eval();
}
