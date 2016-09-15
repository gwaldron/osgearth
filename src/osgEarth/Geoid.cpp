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

#include <osgEarth/Geoid>
#include <osgEarth/HeightFieldUtils>

#define LC "[Geoid] "

using namespace osgEarth;


Geoid::Geoid() :
_units( Units::METERS ),
_valid( false )
{
    //nop
}

void
Geoid::setName( const std::string& name )
{
    _name = name;
    validate();
}

void
Geoid::setHeightField( osg::HeightField* hf )
{
    _hf = hf;
    _bounds = Bounds(
        _hf->getOrigin().x(),
        _hf->getOrigin().y(),
        _hf->getOrigin().x() + _hf->getXInterval() * double(_hf->getNumColumns()-1),
        _hf->getOrigin().y() + _hf->getYInterval() * double(_hf->getNumRows()-1) );
    validate();
}

void
Geoid::setUnits( const Units& units ) 
{
    _units = units;
    validate();
}

void
Geoid::validate()
{
    _valid = false;
    if ( !_hf.valid() )
    {
        //OE_WARN << LC << "ILLEGAL GEOID: no heightfield" << std::endl;
    }
    else if ( !_bounds.valid() )
    {
        OE_WARN << LC << "ILLEGAL GEOID: heightfield must be geodetic" << std::endl;
    }
    else
    {
        _valid = true;
    }
}

float 
Geoid::getHeight(double lat_deg, double lon_deg, const ElevationInterpolation& interp ) const
{
    float result = 0.0f;

    if ( _valid && _bounds.contains(lon_deg, lat_deg) )
    {
        double nlon = (lon_deg-_bounds.xMin())/_bounds.width();
        double nlat = (lat_deg-_bounds.yMin())/_bounds.height();
        result = HeightFieldUtils::getHeightAtNormalizedLocation( _hf.get(), nlon, nlat, interp );
    }

    return result;
}

bool
Geoid::isEquivalentTo( const Geoid& rhs ) const
{
    // weak..
    return
        _valid                      &&
        _name == rhs._name          &&
        _hf.get() == rhs._hf.get()  &&
        _units == rhs._units;
}
