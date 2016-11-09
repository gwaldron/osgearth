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
_heading( Angle(       0.0, Units::DEGREES) ),
_pitch  ( Angle(     -30.0, Units::DEGREES) ),
_range  ( Distance(10000.0, Units::METERS)  )
{
    //NOP
}

Viewpoint::Viewpoint( const Viewpoint& rhs ) :
_name     ( rhs._name ),
_point    ( rhs._point ),
_heading  ( rhs._heading ),
_pitch    ( rhs._pitch ),
_range    ( rhs._range ),
_posOffset( rhs._posOffset),
_node     ( rhs._node.get() )
{
    //NOP
}

Viewpoint::Viewpoint(const char* name, double lon, double lat, double z, double h, double p, double range)
{
    if (name) _name = name;
    _point->set( SpatialReference::get("wgs84"), lon, lat, z, ALTMODE_ABSOLUTE );
    _heading->set( h, Units::DEGREES );
    _pitch->set( p, Units::DEGREES );
    _range->set( range, Units::METERS );
}

Viewpoint::Viewpoint(const Config& conf)
{
    conf.getIfSet( "name",    _name );
    conf.getIfSet( "heading", _heading );
    conf.getIfSet( "pitch",   _pitch );
    conf.getIfSet( "range",   _range );

    // piecewise point.
    std::string horiz = conf.value("srs");
    if ( horiz.empty() )
        horiz = "wgs84";

    const std::string vert = conf.value("vdatum");

    // try to parse an SRS, defaulting to WGS84 if not able to do so
    osg::ref_ptr<const SpatialReference> srs = SpatialReference::create(horiz, vert);

    // try x/y/z variant:
    if ( conf.hasValue("x") )
    {
        _point = GeoPoint(
            srs.get(),
            conf.value<double>("x", 0.0),
            conf.value<double>("y", 0.0),
            conf.value<double>("z", 0.0),
            ALTMODE_ABSOLUTE );
    }
    else if ( conf.hasValue("lat") )
    {
        _point = GeoPoint(
            srs.get(),
            conf.value<double>("long",   0.0),
            conf.value<double>("lat",    0.0),
            conf.value<double>("height", 0.0),
            ALTMODE_ABSOLUTE );
    }

    double xOffset = conf.value("x_offset", 0.0);
    double yOffset = conf.value("y_offset", 0.0);
    double zOffset = conf.value("z_offset", 0.0);
    if ( xOffset != 0.0 || yOffset != 0.0 || zOffset != 0.0 )
    {
        _posOffset->set(xOffset, yOffset, zOffset);
    }
}

#define CONF_STR Stringify() << std::fixed << std::setprecision(4)

Config
Viewpoint::getConfig() const
{
    Config conf( "viewpoint" );

    conf.addIfSet( "name",    _name );
    conf.addIfSet( "heading", _heading );
    conf.addIfSet( "pitch",   _pitch );
    conf.addIfSet( "range",   _range );
    
    if ( _point.isSet() )
    {
        if ( _point->getSRS()->isGeographic() )
        {
            conf.set("long",   _point->x());
            conf.set("lat",    _point->y());
            conf.set("height", _point->z());
        }
        else
        {
            conf.set("x", _point->x());
            conf.set("y", _point->y());
            conf.set("z", _point->z());
        }

        conf.set("srs", _point->getSRS()->getHorizInitString());

        if ( _point->getSRS()->getVerticalDatum() )
            conf.set("vdatum", _point->getSRS()->getVertInitString());
    }

    if ( _posOffset.isSet() )
    {
        conf.set("x_offset", _posOffset->x());
        conf.set("y_offset", _posOffset->y());
        conf.set("z_offset", _posOffset->z());
    }

    return conf;
}

bool
Viewpoint::isValid() const
{
    return
        (_point.isSet() && _point->isValid()) ||
        (_node.valid());
}

std::string
Viewpoint::toString() const
{
    if ( _point.isSet() )
    {
        return Stringify()
            << "x="   << _point->x()
            << ", y=" << _point->y()
            << ", z=" << _point->z()
            << ", h=" << _heading->to(Units::DEGREES).asParseableString()
            << ", p=" << _pitch->to(Units::DEGREES).asParseableString()
            << ", d=" << _range->asParseableString()
            << ", xo=" << _posOffset->x()
            << ", yo=" << _posOffset->y()
            << ", zo=" << _posOffset->z();
    }
    else
    {
        return Stringify()
            << "attached to node; "
            << ", h=" << _heading->to(Units::DEGREES).asParseableString()
            << ", p=" << _pitch->to(Units::DEGREES).asParseableString()
            << ", d=" << _range->asParseableString()
            << ", xo=" << _posOffset->x()
            << ", yo=" << _posOffset->y()
            << ", zo=" << _posOffset->z();
    }
}
