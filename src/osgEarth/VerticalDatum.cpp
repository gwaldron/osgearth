/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/VerticalDatum>
#include <osgEarth/Threading>
#include <osgEarth/GeoData>

#include <osgDB/ReadFile>

using namespace osgEarth;

#undef  LC
#define LC "[VerticalDatum] "

// --------------------------------------------------------------------------

namespace
{
    typedef std::map<std::string, osg::ref_ptr<VerticalDatum> > VDatumCache;
    VDatumCache _vdatumCache;
    std::mutex _vdataCacheMutex;
    bool _vdatumWarning = false;
} 

VerticalDatum*
VerticalDatum::get(const std::string& initString)
{
    VerticalDatum* result = nullptr;

    if (initString.empty())
        return result;

    std::lock_guard<std::mutex> exclusive(_vdataCacheMutex);

    if (::getenv("OSGEARTH_IGNORE_VERTICAL_DATUMS"))
    {
        if (!_vdatumWarning)
        {
            OE_WARN << LC << "WARNING *** Vertical datums have been deactivated; elevation values may be wrong!" << std::endl;
            _vdatumWarning = true;
        }
        return nullptr;
    }

    std::string s = toLower( initString );
    VDatumCache::const_iterator i = _vdatumCache.find( s );
    if ( i != _vdatumCache.end() )
    {
        result = i->second.get();
    }

    if ( !result )
    {
        OE_DEBUG << LC << "Initializing vertical datum: " << initString << std::endl;
        result = create( initString );
        if ( result )
            _vdatumCache[s] = result;
    }
    
    return result;
}

VerticalDatum*
VerticalDatum::create(const std::string& init)
{
    osg::ref_ptr<VerticalDatum> datum;

    bool inverted = false;
    std::string base_name = trim(init);

    // if the name starts with "-", we are creating an inverted datum.
    if (!base_name.empty() && base_name[0] == '-')
    {
        base_name = base_name.substr(1);
        inverted = true;
    }

    std::string driverExt = "osgearth_vdatum_" + base_name;
    auto rw = osgDB::Registry::instance()->getReaderWriterForExtension(driverExt);
    if (rw)
    {
        auto rr = rw->readObject("." + driverExt, nullptr);
        osg::ref_ptr<osg::Object> object = rr.getObject();
        datum = dynamic_cast<VerticalDatum*>(object.release());
        if (!datum)
        {
            OE_WARN << "WARNING: Failed to load Vertical Datum driver for \"" << init << "\"" << std::endl;
        }
    }

    // invert the heights if requested.
    if (datum.valid() && inverted)
    {
        auto geoid = datum->_geoid;
        osg::HeightField::HeightList& heights = geoid->getHeightField()->getHeightList();
        for (auto& h : heights)
            h = -h;

        datum->_initString = init;
        datum->_name = datum->_name + " (inverted)";
    }

    return datum.release();
}


// --------------------------------------------------------------------------

VerticalDatum::VerticalDatum(const std::string& name,
    const std::string& initString,
    Geoid* geoid) :
    _name(name),
    _initString(initString),
    _geoid(geoid),
    _units(Units::METERS)
{
    if (_geoid.valid())
        _units = _geoid->getUnits();
}

VerticalDatum::VerticalDatum(const UnitsType& units) :
    _name(units.getName()),
    _initString(units.getName()),
    _units(units)
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
        in_out_z = from->msl2hae( lat_deg, lon_deg, in_out_z );
    }

    auto fromUnits = from ? from->getUnits() : Units::METERS;
    auto toUnits = to ? to->getUnits() : Units::METERS;

    in_out_z = fromUnits.convertTo(toUnits, in_out_z);

    if ( to )
    {
        in_out_z = to->hae2msl( lat_deg, lon_deg, in_out_z );
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
    
    osg::Vec3d sw(extent.west(), extent.south(), 0.0);
    osg::Vec3d ne(extent.east(), extent.north(), 0.0);
    
    double xstep = std::abs(extent.east() - extent.west()) / double(cols-1);
    double ystep = std::abs(extent.north() - extent.south()) / double(rows-1);

    if ( !extent.getSRS()->isGeographic() )
    {
        const SpatialReference* geoSRS = extent.getSRS()->getGeographicSRS();
        extent.getSRS()->transform(sw, geoSRS, sw);
        extent.getSRS()->transform(ne, geoSRS, ne);
        xstep = (ne.x()-sw.x()) / double(cols-1);
        ystep = (ne.y()-sw.y()) / double(rows-1);
    }

    for( unsigned c=0; c<cols; ++c)
    {
        double lon = sw.x() + xstep*double(c);
        for( unsigned r=0; r<rows; ++r)
        {
            double lat = sw.y() + ystep*double(r);
            float& h = hf->getHeight(c, r);
            if (h != NO_DATA_VALUE)
            {
                VerticalDatum::transform( from, to, lat, lon, h );
            }
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

    if ( rhs == 0L && !_geoid.valid() )
        return true;

    if ( rhs == 0L )
        return false;

    if ( _units != rhs->_units )
        return false;

    if ( _geoid.valid() != rhs->_geoid.valid() )
        return false;
    
    if ( _geoid.valid() && !_geoid->isEquivalentTo( *rhs->_geoid.get() ) )
        return false;

    return true;
}
