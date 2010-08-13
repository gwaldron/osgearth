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

    GeoidRegistry::const_iterator i = (*_geoidRegistry).find( s );
    if ( i != (*_geoidRegistry).end() )
    {
        const Geoid* geoid = i->second.get();
        return new VerticalSpatialReference( geoid->getName(), initString, geoid );
    }
    else if ( s == "meters" || s == "metres" || s == "meter" || s == "metre" )
        return new VerticalSpatialReference( Units::METERS );
    else if ( startsWith( s, "feet" ) || startsWith( s, "foot" ) )
        return new VerticalSpatialReference( Units::FEET_US );

    return 0L;
}

void
VerticalSpatialReference::registerGeoid( const Geoid* geoid )
{
    if ( !_geoidRegistry )
        _geoidRegistry = new GeoidRegistry();

    if ( geoid )
        (*_geoidRegistry)[geoid->getName()] = geoid;
}

VerticalSpatialReference::GeoidRegistry*
VerticalSpatialReference::_geoidRegistry = 0L;

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
VerticalSpatialReference::transform(const VerticalSpatialReference* toSRS, 
                                    double lat_deg, double lon_deg, double in_z,
                                    double& out_z ) const
{    
    if ( this->isEquivalentTo( toSRS ) )
    {
        out_z = in_z;
    }
    else
    {
        double workZ = in_z;

        // transform out of the source VSRS (this):
        if ( _geoid.valid() )
        {
            if ( !_geoid->isValid() )
                return false;

            float offset = _geoid->getOffset( lat_deg, lon_deg, INTERP_BILINEAR );
            workZ -= offset;
        }

        // convert the value to output units:
        Units::convert( getUnits(), toSRS->getUnits(), workZ, workZ );

        // transform into the target VSRS:
        if ( toSRS->_geoid.valid() )
        {
            if ( !toSRS->_geoid->isValid() )
                return false;
        
            float offset = toSRS->_geoid->getOffset( lat_deg, lon_deg, INTERP_BILINEAR );
            workZ += offset;
        }

        out_z = workZ;
    }

    return true;
}

osg::HeightField*
VerticalSpatialReference::createReferenceHeightField( const GeoExtent& ex, int numCols, int numRows ) const
{
    osg::HeightField* hf = new osg::HeightField();
    hf->allocate( numCols, numRows );
    hf->setOrigin( osg::Vec3d( ex.xMin(), ex.yMin(), 0.0 ) );
    hf->setXInterval( (ex.xMax() - ex.xMin())/(double)(numCols-1) );
    hf->setYInterval( (ex.yMax() - ex.yMin())/(double)(numRows-1) );

    if ( _geoid.valid() && _geoid->isValid() )
    {
        // need the lat/long extent for geoid queries:
        GeoExtent geodeticExtent = ex.getSRS()->isGeographic() ? ex : ex.transform( ex.getSRS()->getGeographicSRS() );
        double latMin = geodeticExtent.yMin();
        double lonMin = geodeticExtent.xMin();
        double lonInterval = geodeticExtent.width() / (double)(numCols-1);
        double latInterval = geodeticExtent.height() / (double)(numRows-1);

        for( int r=0; r<numRows; ++r )
        {            
            double lat = latMin + latInterval*(double)r;
            for( int c=0; c<numCols; ++c )
            {
                double lon = lonMin + lonInterval*(double)c;
                double offset = _geoid->getOffset( lat, lon );
                hf->setHeight( c, r, offset );
            }
        }
    }
    else
    {
        for(unsigned int i=0; i<hf->getHeightList().size(); i++ )
            hf->getHeightList()[i] = 0.0;
    }

    hf->setBorderWidth( 0 );
    return hf;    
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
