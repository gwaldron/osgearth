/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osg/Math>

using namespace osgEarth;

Units::Units( const std::string& name, const std::string& abbr, const Units::Type& type, double toBase ) :
_name  ( name ),
_abbr  ( abbr ),
_type  ( type ),
_toBase( toBase )
{
    //nop
}

Units::Units( const std::string& name, const std::string& abbr, const Units& distance, const Units& time ) :
_name    ( name ),
_abbr    ( abbr ),
_type    ( TYPE_SPEED ),
_distance( &distance ),
_time    ( &time )
{
    //nop
}

bool
Units::parse( const std::string& name, Units& output )
{
    const Units* u = osgEarth::Registry::instance()->getUnits( name );
    if ( u ) 
    {
        output = *u;
        return true;
    }
    return false;
}

const Units Units::CENTIMETERS       ( "centimeters",    "cm",  Units::TYPE_LINEAR, 0.01 ); 
OSGEARTH_REGISTER_UNITS( CENTIMETERS, &Units::CENTIMETERS );

const Units Units::FEET              ( "feet",           "ft",  Units::TYPE_LINEAR, 0.3048 );
OSGEARTH_REGISTER_UNITS( FEET, &Units::FEET );

const Units Units::FEET_US_SURVEY    ( "feet(us)",       "ft",  Units::TYPE_LINEAR, 12.0/39.37 );
OSGEARTH_REGISTER_UNITS( FEET_US_SURVEY, &Units::FEET_US_SURVEY );

const Units Units::KILOMETERS        ( "kilometers",     "km",  Units::TYPE_LINEAR, 1000.0 );
OSGEARTH_REGISTER_UNITS( KILOMETERS, &Units::KILOMETERS );

const Units Units::METERS            ( "meters",         "m",   Units::TYPE_LINEAR, 1.0 );
OSGEARTH_REGISTER_UNITS( METERS, &Units::METERS );

const Units Units::MILES             ( "miles",          "mi",  Units::TYPE_LINEAR, 1609.334 );
OSGEARTH_REGISTER_UNITS( MILES, &Units::MILES );

const Units Units::MILLIMETERS       ( "millimeters",    "mm",  Units::TYPE_LINEAR, 0.001 );
OSGEARTH_REGISTER_UNITS( MILLIMETERS, &Units::MILLIMETERS );

const Units Units::YARDS             ( "yards",          "yd",  Units::TYPE_LINEAR, 0.9144 );
OSGEARTH_REGISTER_UNITS( YARDS, &Units::YARDS );

const Units Units::NAUTICAL_MILES    ( "nautical miles", "nm",  Units::TYPE_LINEAR, 1852.0 );
OSGEARTH_REGISTER_UNITS( NAUTICAL_MILES, &Units::NAUTICAL_MILES );

const Units Units::DATA_MILES        ( "data miles",     "dm",  Units::TYPE_LINEAR, 1828.8 );
OSGEARTH_REGISTER_UNITS( DATA_MILES, &Units::DATA_MILES );

const Units Units::INCHES            ( "inches",         "in",  Units::TYPE_LINEAR, 0.0254 );
OSGEARTH_REGISTER_UNITS( INCHES, &Units::INCHES );

const Units Units::FATHOMS           ( "fathoms",        "fm",  Units::TYPE_LINEAR, 1.8288 );
OSGEARTH_REGISTER_UNITS( FATHOMS, &Units::FATHOMS );

const Units Units::KILOFEET          ( "kilofeet",       "kf",  Units::TYPE_LINEAR, 304.8 );
OSGEARTH_REGISTER_UNITS( KILOFEET, &Units::KILOFEET );

const Units Units::KILOYARDS         ( "kiloyards",      "kyd", Units::TYPE_LINEAR, 914.4 );
OSGEARTH_REGISTER_UNITS( KILOYARDS, &Units::KILOYARDS );


const Units Units::DEGREES           ( "degrees",        "\xb0",Units::TYPE_ANGULAR, 0.017453292519943295 );
OSGEARTH_REGISTER_UNITS( DEGREES, &Units::DEGREES );

const Units Units::RADIANS           ( "radians",        "rad", Units::TYPE_ANGULAR, 1.0 );
OSGEARTH_REGISTER_UNITS( RADIANS, &Units::RADIANS );

const Units Units::BAM               ( "BAM",            "bam", Units::TYPE_ANGULAR, 6.283185307179586476925286766559 );
OSGEARTH_REGISTER_UNITS( BAM, &Units::BAM );

const Units Units::NATO_MILS         ( "mils",           "mil", Units::TYPE_ANGULAR, 9.8174770424681038701957605727484e-4 );
OSGEARTH_REGISTER_UNITS( NATO_MILS, &Units::NATO_MILS );


const Units Units::DAYS              ( "days",           "d",   Units::TYPE_TEMPORAL, 1.0/86400.0 );
OSGEARTH_REGISTER_UNITS( DAYS, &Units::DAYS );

const Units Units::HOURS             ( "hours",          "hr",  Units::TYPE_TEMPORAL, 1.0/3600.0 );
OSGEARTH_REGISTER_UNITS( HOURS, &Units::HOURS );

const Units Units::MICROSECONDS      ( "microseconds",   "us",  Units::TYPE_TEMPORAL, 1000000.0 );
OSGEARTH_REGISTER_UNITS( MICROSECONDS, &Units::MICROSECONDS );

const Units Units::MILLISECONDS      ( "milliseconds",   "ms",  Units::TYPE_TEMPORAL, 1000.0 );
OSGEARTH_REGISTER_UNITS( MILLISECONDS, &Units::MILLISECONDS );

const Units Units::MINUTES           ( "minutes",        "min", Units::TYPE_TEMPORAL, 1.0/60.0 );
OSGEARTH_REGISTER_UNITS( MINUTES, &Units::MINUTES );

const Units Units::SECONDS           ( "seconds",        "s",   Units::TYPE_TEMPORAL, 1.0 );
OSGEARTH_REGISTER_UNITS( SECONDS, &Units::SECONDS );

const Units Units::WEEKS             ( "weeks",          "wk",  Units::TYPE_TEMPORAL, 1.0/604800.0 );
OSGEARTH_REGISTER_UNITS( WEEKS, &Units::WEEKS );


const Units Units::FEET_PER_SECOND      ( "feet per second",         "ft/s", Units::FEET,           Units::SECONDS );
OSGEARTH_REGISTER_UNITS( FEET_PER_SECOND, &Units::FEET_PER_SECOND );

const Units Units::YARDS_PER_SECOND     ( "yards per second",        "yd/s", Units::YARDS,          Units::SECONDS );
OSGEARTH_REGISTER_UNITS( YARDS_PER_SECOND, &Units::YARDS_PER_SECOND );

const Units Units::METERS_PER_SECOND    ( "meters per second",       "m/s",  Units::METERS,         Units::SECONDS );
OSGEARTH_REGISTER_UNITS( METERS_PER_SECOND, &Units::METERS_PER_SECOND );

const Units Units::KILOMETERS_PER_SECOND( "kilometers per second",   "km/s", Units::KILOMETERS,     Units::SECONDS );
OSGEARTH_REGISTER_UNITS( KILOMETERS_PER_SECOND, &Units::KILOMETERS_PER_SECOND );

const Units Units::KILOMETERS_PER_HOUR  ( "kilometers per hour",     "kmh",  Units::KILOMETERS,     Units::SECONDS );
OSGEARTH_REGISTER_UNITS( KILOMETERS_PER_HOUR, &Units::KILOMETERS_PER_HOUR );

const Units Units::MILES_PER_HOUR       ( "miles per hour",          "mph",  Units::MILES,          Units::HOURS );
OSGEARTH_REGISTER_UNITS( MILES_PER_HOUR, &Units::MILES_PER_HOUR );

const Units Units::DATA_MILES_PER_HOUR  ( "data miles per hour",     "dm/h", Units::DATA_MILES,     Units::HOURS );
OSGEARTH_REGISTER_UNITS( DATA_MILES_PER_HOUR, &Units::DATA_MILES_PER_HOUR );

const Units Units::KNOTS                ( "nautical miles per hour", "kts",  Units::NAUTICAL_MILES, Units::HOURS );
OSGEARTH_REGISTER_UNITS( KNOTS, &Units::KNOTS );

