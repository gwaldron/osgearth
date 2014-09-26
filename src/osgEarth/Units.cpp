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

#include <osgEarth/Units>
#include <osgEarth/Registry>
#include <osg/Math>
#include <algorithm>
#include <cctype>

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    template<typename T>
    bool
    parseValueAndUnits(const std::string& input, 
                       T&                 out_value, 
                       Units&             out_units,
                       const Units&       defaultUnits )
    {
        if ( input.empty() )
            return false;

        std::string valueStr, unitsStr;

        std::string::const_iterator i = std::find_if( input.begin(), input.end(), ::isalpha );
        if ( i == input.end() )
        {
            // to units found; use default
            out_units = defaultUnits;
            out_value = as<T>(input, (T)0.0);
            return true;
        }

        else
        {
            valueStr = std::string( input.begin(), i );
            unitsStr = std::string( i, input.end() );

            if ( !valueStr.empty() )
            {
                out_value = as<T>(valueStr, (T)0);
            }

            if ( !unitsStr.empty() )
            {
                Units units;
                if ( Units::parse(unitsStr, units) )
                    out_units = units;
            }
            else
            {
                out_units = defaultUnits;
            }

            return !valueStr.empty() && !unitsStr.empty();
        }
    }
}

//------------------------------------------------------------------------



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

bool
Units::parse( const std::string& input, float& out_value, Units& out_units, const Units& defaultUnits )
{
    return parseValueAndUnits(input, out_value, out_units, defaultUnits);
}

bool
Units::parse( const std::string& input, double& out_value, Units& out_units, const Units& defaultUnits )
{
    return parseValueAndUnits(input, out_value, out_units, defaultUnits);
}

bool
Units::parse( const std::string& input, int& out_value, Units& out_units, const Units& defaultUnits )
{
    return parseValueAndUnits(input, out_value, out_units, defaultUnits);
}

void
Units::registerAll(Registry* r)
{
    r->registerUnits( &Units::CENTIMETERS );
    r->registerUnits( &Units::FEET );
    r->registerUnits( &Units::FEET_US_SURVEY );
    r->registerUnits( &Units::KILOMETERS );
    r->registerUnits( &Units::METERS );
    r->registerUnits( &Units::MILES );
    r->registerUnits( &Units::MILLIMETERS );
    r->registerUnits( &Units::YARDS );
    r->registerUnits( &Units::NAUTICAL_MILES );
    r->registerUnits( &Units::DATA_MILES );
    r->registerUnits( &Units::INCHES );
    r->registerUnits( &Units::FATHOMS );
    r->registerUnits( &Units::KILOFEET );
    r->registerUnits( &Units::KILOYARDS );

    r->registerUnits( &Units::DEGREES );
    r->registerUnits( &Units::RADIANS );
    r->registerUnits( &Units::BAM );
    r->registerUnits( &Units::NATO_MILS );

    r->registerUnits( &Units::DAYS );
    r->registerUnits( &Units::HOURS );
    r->registerUnits( &Units::MICROSECONDS );
    r->registerUnits( &Units::MILLISECONDS );
    r->registerUnits( &Units::MINUTES );
    r->registerUnits( &Units::SECONDS );
    r->registerUnits( &Units::WEEKS );

    r->registerUnits( &Units::FEET_PER_SECOND );
    r->registerUnits( &Units::YARDS_PER_SECOND );
    r->registerUnits( &Units::METERS_PER_SECOND );
    r->registerUnits( &Units::KILOMETERS_PER_SECOND );
    r->registerUnits( &Units::KILOMETERS_PER_HOUR );
    r->registerUnits( &Units::MILES_PER_HOUR );
    r->registerUnits( &Units::DATA_MILES_PER_HOUR );
    r->registerUnits( &Units::KNOTS );

    r->registerUnits( &Units::PIXELS );
}


// Factor converts unit into METERS:
const Units Units::CENTIMETERS       ( "centimeters",    "cm",  Units::TYPE_LINEAR, 0.01 ); 
const Units Units::FEET              ( "feet",           "ft",  Units::TYPE_LINEAR, 0.3048 );
const Units Units::FEET_US_SURVEY    ( "feet(us)",       "ft",  Units::TYPE_LINEAR, 12.0/39.37 );
const Units Units::KILOMETERS        ( "kilometers",     "km",  Units::TYPE_LINEAR, 1000.0 );
const Units Units::METERS            ( "meters",         "m",   Units::TYPE_LINEAR, 1.0 );
const Units Units::MILES             ( "miles",          "mi",  Units::TYPE_LINEAR, 1609.334 );
const Units Units::MILLIMETERS       ( "millimeters",    "mm",  Units::TYPE_LINEAR, 0.001 );
const Units Units::YARDS             ( "yards",          "yd",  Units::TYPE_LINEAR, 0.9144 );
const Units Units::NAUTICAL_MILES    ( "nautical miles", "nm",  Units::TYPE_LINEAR, 1852.0 );
const Units Units::DATA_MILES        ( "data miles",     "dm",  Units::TYPE_LINEAR, 1828.8 );
const Units Units::INCHES            ( "inches",         "in",  Units::TYPE_LINEAR, 0.0254 );
const Units Units::FATHOMS           ( "fathoms",        "fm",  Units::TYPE_LINEAR, 1.8288 );
const Units Units::KILOFEET          ( "kilofeet",       "kf",  Units::TYPE_LINEAR, 304.8 );
const Units Units::KILOYARDS         ( "kiloyards",      "kyd", Units::TYPE_LINEAR, 914.4 );

// Factor converts unit into RADIANS:
const Units Units::DEGREES           ( "degrees",        "\xb0",Units::TYPE_ANGULAR, 0.017453292519943295 );
const Units Units::RADIANS           ( "radians",        "rad", Units::TYPE_ANGULAR, 1.0 );
const Units Units::BAM               ( "BAM",            "bam", Units::TYPE_ANGULAR, 6.283185307179586476925286766559 );
const Units Units::NATO_MILS         ( "mils",           "mil", Units::TYPE_ANGULAR, 9.8174770424681038701957605727484e-4 );

// Factor convert unit into SECONDS:
const Units Units::DAYS              ( "days",           "d",   Units::TYPE_TEMPORAL, 86400.0 );
const Units Units::HOURS             ( "hours",          "hr",  Units::TYPE_TEMPORAL, 3600.0 );
const Units Units::MICROSECONDS      ( "microseconds",   "us",  Units::TYPE_TEMPORAL, 0.000001 );
const Units Units::MILLISECONDS      ( "milliseconds",   "ms",  Units::TYPE_TEMPORAL, 0.001 );
const Units Units::MINUTES           ( "minutes",        "min", Units::TYPE_TEMPORAL, 60.0 );
const Units Units::SECONDS           ( "seconds",        "s",   Units::TYPE_TEMPORAL, 1.0 );
const Units Units::WEEKS             ( "weeks",          "wk",  Units::TYPE_TEMPORAL, 604800.0 );

const Units Units::FEET_PER_SECOND      ( "feet per second",         "ft/s", Units::FEET,           Units::SECONDS );
const Units Units::YARDS_PER_SECOND     ( "yards per second",        "yd/s", Units::YARDS,          Units::SECONDS );
const Units Units::METERS_PER_SECOND    ( "meters per second",       "m/s",  Units::METERS,         Units::SECONDS );
const Units Units::KILOMETERS_PER_SECOND( "kilometers per second",   "km/s", Units::KILOMETERS,     Units::SECONDS );
const Units Units::KILOMETERS_PER_HOUR  ( "kilometers per hour",     "kmh",  Units::KILOMETERS,     Units::HOURS );
const Units Units::MILES_PER_HOUR       ( "miles per hour",          "mph",  Units::MILES,          Units::HOURS );
const Units Units::DATA_MILES_PER_HOUR  ( "data miles per hour",     "dm/h", Units::DATA_MILES,     Units::HOURS );
const Units Units::KNOTS                ( "nautical miles per hour", "kts",  Units::NAUTICAL_MILES, Units::HOURS );

const Units Units::PIXELS               ( "pixels", "px", Units::TYPE_SCREEN_SIZE, 1.0 );
