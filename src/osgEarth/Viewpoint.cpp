/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/Viewpoint>
#include <sstream>

using namespace osgEarth;


Viewpoint::Viewpoint() :
_is_valid( false ),
_heading_deg(0.0),
_pitch_deg(0.0),
_range(1.0)
{
    //NOP
}

Viewpoint::Viewpoint(const osg::Vec3d& focal_point,
                     double heading_deg,
                     double pitch_deg, 
                     double range,
                     const osgEarth::SpatialReference* srs ) :
_focal_point( focal_point ),
_heading_deg( heading_deg ),
_pitch_deg( pitch_deg ),
_range( range ),
_srs( srs ),
_is_valid( true )
{
    //NOP
}
Viewpoint::Viewpoint(double x, double y, double z,
                     double heading_deg,
                     double pitch_deg, 
                     double range,
                     const osgEarth::SpatialReference* srs ) :
_focal_point( x, y, z ),
_heading_deg( heading_deg ),
_pitch_deg( pitch_deg ),
_range( range ),
_srs( srs ),
_is_valid( true )
{
    //NOP
}

Viewpoint::Viewpoint(const std::string& name,
                     const osg::Vec3d& focal_point,
                     double heading_deg,
                     double pitch_deg, 
                     double range,
                     const osgEarth::SpatialReference* srs ) :
_name( name ),
_focal_point( focal_point ),
_heading_deg( heading_deg ),
_pitch_deg( pitch_deg ),
_range( range ),
_srs( srs ),
_is_valid( true )
{
    //NOP
}

Viewpoint::Viewpoint(const std::string& name,
                     double x, double y, double z,
                     double heading_deg,
                     double pitch_deg, 
                     double range,
                     const osgEarth::SpatialReference* srs ) :
_name( name ),
_focal_point( x, y, z ),
_heading_deg( heading_deg ),
_pitch_deg( pitch_deg ),
_range( range ),
_srs( srs ),
_is_valid( true )
{
    //NOP
}

Viewpoint::Viewpoint( const Viewpoint& rhs ) :
_name( rhs._name ),
_focal_point( rhs._focal_point ),
_heading_deg( rhs._heading_deg ),
_pitch_deg( rhs._pitch_deg ),
_range( rhs._range ),
_srs( rhs._srs.get() ),
_is_valid( rhs._is_valid )
{
    //NOP
}

Viewpoint::Viewpoint( const Config& conf )
{
    _name = conf.value("name");

    if ( conf.hasValue("x") )
    {
        _focal_point.set(
            conf.value<double>("x", 0.0),
            conf.value<double>("y", 0.0),
            conf.value<double>("z", 0.0) );
    }
    else if ( conf.hasValue("lat") )
    {
        _focal_point.set(
            conf.value<double>("long", 0.0),
            conf.value<double>("lat", 0.0),
            conf.value<double>("height", 0.0) );

        if ( !conf.hasValue("srs") )
            _srs = SpatialReference::create("wgs84");
    }

    _heading_deg = conf.value<double>("heading", 0.0);
    _pitch_deg   = conf.value<double>("pitch",   0.0);
    _range       = conf.value<double>("range",   0.0);
    _is_valid    = _range > 0.0;

    const std::string horiz = conf.value("srs");
    const std::string vert  = conf.value("vdatum");

    if ( !horiz.empty() )
    {
        _srs = SpatialReference::create(horiz, vert);
    }
}

#define CONF_STR Stringify() << std::fixed << std::setprecision(4)

Config
Viewpoint::getConfig() const
{
    Config conf( "viewpoint" );

    if ( _is_valid )
    {
        if ( !_name.empty() )
            conf.set("name", _name);

        if ( getSRS() && getSRS()->isGeographic() )
        {
            conf.set("lat",    _focal_point.y());
            conf.set("long",   _focal_point.x());
            conf.set("height", _focal_point.z());
        }
        else
        {
            conf.set("x", _focal_point.x());
            conf.set("y", _focal_point.y());
            conf.set("z", _focal_point.z());
        }

        conf.set("heading", _heading_deg);
        conf.set("pitch",   _pitch_deg);
        conf.set("range",   _range);

        if ( _srs.valid() )
        {
            conf.set("srs", _srs->getHorizInitString());
            if ( _srs->getVerticalDatum() )
                conf.set("vdatum", _srs->getVertInitString());
        }
    }

    return conf;
}

bool
Viewpoint::isValid() const {
    return _is_valid;
}

const std::string&
Viewpoint::getName() const {
    return _name;
}

void
Viewpoint::setName( const std::string& name ) {
    _name = name;
}

const osg::Vec3d&
Viewpoint::getFocalPoint() const {
    return _focal_point;
}

void
Viewpoint::setFocalPoint( const osg::Vec3d& value ) {
    _focal_point = value;
}

double
Viewpoint::x() const {
    return _focal_point.x();
}

double&
Viewpoint::x() {
    return _focal_point.x();
}

double
Viewpoint::y() const {
    return _focal_point.y();
}

double&
Viewpoint::y() {
    return _focal_point.y();
}

double
Viewpoint::z() const {
    return _focal_point.z();
}

double&
Viewpoint::z() {
    return _focal_point.z();
}

double
Viewpoint::getHeading() const {
    return _heading_deg;
}

void
Viewpoint::setHeading( double value ) {
    _heading_deg = value;
}

double
Viewpoint::getPitch() const {
    return _pitch_deg;
}

void
Viewpoint::setPitch( double value ) {
    _pitch_deg = value;
}

double
Viewpoint::getRange() const {
    return _range;
}

void
Viewpoint::setRange( double value ) {
    _range = value;
}

const SpatialReference*
Viewpoint::getSRS() const {
    return _srs.get();
}

std::string
Viewpoint::toString() const
{
    return Stringify()
        << "x=" << _focal_point.x()
        << ", y=" << _focal_point.y()
        << ", z=" << _focal_point.z()
        << ", h=" << _heading_deg
        << ", p=" << _pitch_deg
        << ", d=" << _range;
}
