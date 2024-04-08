/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include "Units"
#include "Registry"

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    template<typename T>
    bool parseValueAndUnits(const std::string& input,
        T& out_value,
        UnitsType& out_units,
        const UnitsType& defaultUnits)
    {
        if (input.empty())
            return false;

        std::string valueStr, unitsStr;

        std::string::const_iterator start = input.begin();
        
        // deal with scientific notation by moving the units search point
        // past the "e+/-" if it exists:
        std::string::size_type pos = input.find_first_of("eE");
        if (pos != std::string::npos && 
            input.length() > (pos+2) &&
            (input[pos+1] == '-' || input[pos+1] == '+'))
        {
            start = input.begin() + pos + 2;
        }

        std::string::const_iterator i = std::find_if( start, input.end(), ::isalpha );
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
                UnitsType units;
                if ( Units::parse(unitsStr, units) )
                    out_units = units;
                else if (unitsStr.back() != 's' && Units::parse(unitsStr+'s', units))
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

bool
Units::parse(const std::string& name, UnitsType& output)
{
    const UnitsType u = osgEarth::Registry::instance()->getUnits(name);
    if (u.valid())
    {
        output = u;
        return true;
    }
    return false;
}

bool
Units::parse(const std::string& input, float& out_value, UnitsType& out_units, const UnitsType& defaultUnits)
{
    return parseValueAndUnits(input, out_value, out_units, defaultUnits);
}

bool
Units::parse(const std::string& input, double& out_value, UnitsType& out_units, const UnitsType& defaultUnits)
{
    return parseValueAndUnits(input, out_value, out_units, defaultUnits);
}

bool
Units::parse(const std::string& input, int& out_value, UnitsType& out_units, const UnitsType& defaultUnits)
{
    return parseValueAndUnits(input, out_value, out_units, defaultUnits);
}

void
Units::registerAll(Registry* r)
{
    r->registerUnits(Units::CENTIMETERS);
    r->registerUnits(Units::FEET);
    r->registerUnits(Units::FEET_US_SURVEY);
    r->registerUnits(Units::KILOMETERS);
    r->registerUnits(Units::METERS);
    r->registerUnits(Units::MILES);
    r->registerUnits(Units::MILLIMETERS);
    r->registerUnits(Units::YARDS);
    r->registerUnits(Units::NAUTICAL_MILES);
    r->registerUnits(Units::DATA_MILES);
    r->registerUnits(Units::INCHES);
    r->registerUnits(Units::FATHOMS);
    r->registerUnits(Units::KILOFEET);
    r->registerUnits(Units::KILOYARDS);

    r->registerUnits(Units::DEGREES);
    r->registerUnits(Units::RADIANS);
    r->registerUnits(Units::BAM);
    r->registerUnits(Units::NATO_MILS);
    r->registerUnits(Units::DECIMAL_HOURS);

    r->registerUnits(Units::DAYS);
    r->registerUnits(Units::HOURS);
    r->registerUnits(Units::MICROSECONDS);
    r->registerUnits(Units::MILLISECONDS);
    r->registerUnits(Units::MINUTES);
    r->registerUnits(Units::SECONDS);
    r->registerUnits(Units::WEEKS);

    r->registerUnits(Units::FEET_PER_SECOND);
    r->registerUnits(Units::YARDS_PER_SECOND);
    r->registerUnits(Units::METERS_PER_SECOND);
    r->registerUnits(Units::KILOMETERS_PER_SECOND);
    r->registerUnits(Units::KILOMETERS_PER_HOUR);
    r->registerUnits(Units::MILES_PER_HOUR);
    r->registerUnits(Units::DATA_MILES_PER_HOUR);
    r->registerUnits(Units::KNOTS);

    r->registerUnits(Units::PIXELS);
}

int
Units::unitTest()
{
    double value;
    UnitsType units;

    // test parsing scientific notation
    {
        Units::parse( "123e-003m", value, units, Units::MILES);
        if ( value != 123e-003 || units != Units::METERS )
            return 101;

        Units::parse( "123e+003m", value, units, Units::MILES );
        if ( value != 123e+003 || units != Units::METERS )
            return 102;

        Units::parse( "123E-003m", value, units, Units::MILES );
        if ( value != 123E-003 || units != Units::METERS )
            return 103;

        Units::parse( "123E+003m", value, units, Units::MILES );
        if ( value != 123E+003 || units != Units::METERS )
            return 104;
    }

    // normal parsing
    {
        Units::parse( "123m", value, units, Units::MILES );
        if ( value != 123 || units != Units::METERS )
            return 201;
        
        Units::parse( "123km", value, units, Units::MILES );
        if ( value != 123 || units != Units::KILOMETERS )
            return 202;
        
        Units::parse( "1.2rad", value, units, Units::DEGREES );
        if ( value != 1.2 || units != Units::RADIANS )
            return 203;
    }

    // add tests as needed

    return 0;
}
