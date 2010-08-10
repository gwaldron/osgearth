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

#include <osgEarth/VerticalSpatialReference>
#include <osgEarth/EGM>
#include <osgEarth/StringUtils>
#include <osgEarth/GeoData>

using namespace osgEarth;

#define LC "[osgEarth::VSRS] "

// --------------------------------------------------------------------------

//TODO: replace this with some kind of registry.
VerticalSpatialReference*
VerticalSpatialReference::create( const std::string& initString )
{
    std::string s = toLower( initString );

    if ( s == EGM::EGM96_NAME )
        return new VerticalSpatialReference( EGM::EGM96_NAME, initString, EGM::createEGM96Geoid() );
    else if ( s == "meters" || s == "metres" || s == "meter" || s == "metre" )
        return new VerticalSpatialReference( Units::METERS );
    else if ( startsWith( s, "feet" ) || startsWith( s, "foot" ) )
        return new VerticalSpatialReference( Units::FEET_US );

    return 0L;
}

const VerticalSpatialReference*
VerticalSpatialReference::DEFAULT_VSRS = new VerticalSpatialReference( Units::METERS );

// --------------------------------------------------------------------------

VerticalSpatialReference::VerticalSpatialReference(const std::string& name,
                                                   const std::string& initString,
                                                   const Geoid* geoid ) :
_name( name ),
_initString( initString ),
_geoid( geoid ),
_units( Units::METERS )
{
    if ( _geoid.valid() )
        _units = _geoid->getUnits();
}

VerticalSpatialReference::VerticalSpatialReference( const Units& units ) :
_name( units.getName() ),
_initString( units.getName() ),
_units( units )
{
    //nop
}

bool
VerticalSpatialReference::canTransform( const VerticalSpatialReference* toVSRS ) const
{
    return toVSRS && !isEquivalentTo( toVSRS );
}

bool
VerticalSpatialReference::canTransform(const VerticalSpatialReference* fromVSRS,
                                       const VerticalSpatialReference* toVSRS )
{
    return fromVSRS && fromVSRS->canTransform( toVSRS );
}

bool 
VerticalSpatialReference::transform(const VerticalSpatialReference* to_srs, 
                                    double lat_deg, double lon_deg, double in_z,
                                    double& out_z ) const
{
    bool success = false;

    if ( _geoid.valid() && _geoid->isValid() )
    {
        float offset = _geoid->getOffset( lat_deg, lon_deg, INTERP_BILINEAR );
        Units::convert( getUnits(), to_srs->getUnits(), in_z + offset, out_z );
        success = true;
    }
    else
    {
        // Units-only VSRS. just convert, ignoring the lat/long.
        Units::convert( getUnits(), to_srs->getUnits(), in_z, out_z );
        success = true;
    }

    return success;
}

bool 
VerticalSpatialReference::isEquivalentTo( const VerticalSpatialReference* rhs ) const
{
    if ( this == rhs )
        return true;

    if ( _units != rhs->_units )
        return false;

    if ( _geoid.valid() != rhs->_geoid.valid() )
        return false;
    
    if ( _geoid.valid() && !_geoid->isEquivalentTo( *rhs->_geoid.get() ) )
        return false;

    //TODO - add comparisons as necessary

    return true;
}
