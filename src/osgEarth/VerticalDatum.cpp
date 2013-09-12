/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/VerticalDatum>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/GeoData>

#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>

using namespace osgEarth;

#undef  LC
#define LC "[VerticalDatum] "

// --------------------------------------------------------------------------

namespace
{
    typedef std::map<std::string, osg::ref_ptr<VerticalDatum> > VDatumCache;
    VDatumCache      _vdatumCache;
    Threading::Mutex _vdataCacheMutex;
} 

VerticalDatum*
VerticalDatum::get( const std::string& initString )
{
    VerticalDatum* result = 0L;

    Threading::ScopedMutexLock exclusive(_vdataCacheMutex);

    std::string s = toLower( initString );
    VDatumCache::const_iterator i = _vdatumCache.find( s );
    if ( i != _vdatumCache.end() )
    {
        result = i->second.get();
    }

    if ( !result )
    {
        OE_INFO << LC << "Initializing vertical datum: " << initString << std::endl;
        result = VerticalDatumFactory::create( initString );
        if ( result )
            _vdatumCache[s] = result;
    }
    
    return result;
}

// --------------------------------------------------------------------------

VerticalDatum::VerticalDatum(const std::string& name,
                             const std::string& initString,
                             Geoid*             geoid ) :
_name      ( name ),
_initString( initString ),
_geoid     ( geoid ),
_units     ( Units::METERS )
{
    if ( _geoid.valid() )
        _units = _geoid->getUnits();
}

VerticalDatum::VerticalDatum( const Units& units ) :
_name      ( units.getName() ),
_initString( units.getName() ),
_units     ( units )
{
    //nop
}

bool
VerticalDatum::transform(const VerticalDatum* from,
                         const VerticalDatum* to,
                         double               lat_deg,
                         double               lon_deg,
                         double&              in_out_z)
{
    if ( from == to )
        return true;

    if ( from )
    {
        in_out_z = from->msl2hae( lat_deg, lon_deg, INTERP_BILINEAR );
    }

    Units fromUnits = from ? from->getUnits() : Units::METERS;
    Units toUnits = to ? to->getUnits() : fromUnits;

    in_out_z = fromUnits.convertTo(toUnits, in_out_z);

    if ( to )
    {
        in_out_z = to->hae2msl( lat_deg, lon_deg, INTERP_BILINEAR );
    }

    return true;
}

bool
VerticalDatum::transform(const VerticalDatum* from,
                         const VerticalDatum* to,
                         double               lat_deg,
                         double               lon_deg,
                         float&               in_out_z)
{
    double d(in_out_z);
    bool ok = transform(from, to, lat_deg, lon_deg, d);
    if (ok) in_out_z = float(d);
    return ok;
}

bool
VerticalDatum::transform(const VerticalDatum* from,
                         const VerticalDatum* to,
                         const GeoExtent&     extent,
                         osg::HeightField*    hf )
{
    if ( from == to )
        return true;

    unsigned cols = hf->getNumColumns();
    unsigned rows = hf->getNumRows();
    osg::Vec3d sw = hf->getOrigin();
    osg::Vec3d ne;
    ne.x() = sw.x() + hf->getXInterval()*double(rows);
    ne.y() = sw.y() + hf->getYInterval()*double(cols);
    double xstep = hf->getXInterval();
    double ystep = hf->getYInterval();

    if ( !extent.getSRS()->isGeographic() )
    {
        const SpatialReference* geoSRS = extent.getSRS()->getGeographicSRS();
        extent.getSRS()->transform(sw, geoSRS, sw);
        extent.getSRS()->transform(ne, geoSRS, ne);
        xstep = (ne.x()-sw.x()) / double(cols);
        ystep = (ne.y()-sw.y()) / double(rows);
    }

    for( unsigned c=0; c<cols; ++c)
    {
        double lon = sw.x() + xstep*double(c);
        for( unsigned r=0; r<rows; ++r)
        {
            double lat = sw.y() + ystep*double(r);
            float& h = hf->getHeight(r, c);
            VerticalDatum::transform( from, to, lat, lon, h );
        }
    }

    return true;
}

double 
VerticalDatum::msl2hae( double lat_deg, double lon_deg, double msl ) const
{
    return _geoid.valid() ? msl + _geoid->getHeight(lat_deg, lon_deg, INTERP_BILINEAR) : msl;
}

double
VerticalDatum::hae2msl( double lat_deg, double lon_deg, double hae ) const
{
    return _geoid.valid() ? hae - _geoid->getHeight(lat_deg, lon_deg, INTERP_BILINEAR) : hae;
}

bool 
VerticalDatum::isEquivalentTo( const VerticalDatum* rhs ) const
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


//------------------------------------------------------------------------

#undef  LC
#define LC "[VerticalDatumFactory] "

VerticalDatum*
VerticalDatumFactory::create( const std::string& init )
{
    VerticalDatum* result = 0L;

    std::string driverExt = Stringify() << ".osgearth_vdatum_" << init;
    result = dynamic_cast<VerticalDatum*>( osgDB::readObjectFile(driverExt) );
    if ( !result )
    {
        OE_WARN << "WARNING: Failed to load Vertical Datum driver for \"" << init << "\"" << std::endl;
    }

    return result;
}
