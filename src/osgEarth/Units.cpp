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

#include <osgEarth/Units>
#include <osg/Math>

using namespace osgEarth;

Units::Units( const std::string& name, const std::string& abbr, const Units::Type& type, double toBase ) :
_name( name ),
_abbr( abbr ),
_type( type ),
_toBase( toBase )
{
    //nop
}

const Units Units::CENTIMETERS       ( "centimeters",    "cm", Units::TYPE_LINEAR, 0.01 );
const Units Units::FEET_INTERNATIONAL( "feet (int'l)",   "ft", Units::TYPE_LINEAR, 0.3048 );
const Units Units::FEET_US           ( "feet (us)",      "ft", Units::TYPE_LINEAR, 12.0/39.37);
const Units Units::KILOMETERS        ( "kilometers",     "km", Units::TYPE_LINEAR, 1000.0 );
const Units Units::METERS            ( "meters",         "m",  Units::TYPE_LINEAR, 1.0 );
const Units Units::MILES             ( "miles",          "mi", Units::TYPE_LINEAR, 1609.334 );
const Units Units::MILLIMETERS       ( "millimeters",    "mm", Units::TYPE_LINEAR, 0.001 );
const Units Units::NAUTICAL_MILES    ( "nautical miles", "nm", Units::TYPE_LINEAR, 1852.0 );

const Units Units::DEGREES           ( "degrees",        "deg", Units::TYPE_ANGULAR, osg::PI/180.0 );
const Units Units::RADIANS           ( "radians",        "rad", Units::TYPE_ANGULAR, 1.0 );

