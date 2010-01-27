/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthUtil/Viewpoint>

using namespace osgEarthUtil;
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
