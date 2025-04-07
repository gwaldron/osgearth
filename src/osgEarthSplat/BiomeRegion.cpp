/* osgEarth
 * Copyright 2015 Pelican Mapping
 * MIT License
 */
#include "BiomeRegion"

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[BiomeRegion] "

#define SPLAT_BIOME_CURRENT_VERSION 1


//............................................................................

BiomeRegion::BiomeRegion()
{
    //nop
}

bool
BiomeRegion::configure(const Config& conf)
{
    conf.getIfSet( "name", _name );
    conf.getIfSet( "catalog", _catalogURI );
    
    // only supports lat long for now
    const SpatialReference* srs = SpatialReference::create("wgs84");
    
    const Config& extentsConf = conf.child("regions");
    for(ConfigSet::const_iterator i = extentsConf.children().begin();
        i != extentsConf.children().end();
        ++i)
    {
        double xmin = i->value("xmin", -DBL_MAX);
        double xmax = i->value("xmax",  DBL_MAX);
        double ymin = i->value("ymin", -DBL_MAX);
        double ymax = i->value("ymax",  DBL_MAX);
        double zmin = i->value("zmin", -DBL_MAX);
        double zmax = i->value("zmax",  DBL_MAX);
        
        _regions.push_back( Region() );
        _regions.back().extent = GeoExtent(srs, xmin, ymin, xmax, ymax);
        _regions.back().zmin  = zmin;
        _regions.back().zmax  = zmax;
    }

    return true;
}

Config
BiomeRegion::getConfig() const
{
    Config conf("biome");
    conf.addIfSet( "name",    _name );
    conf.addIfSet( "catalog", _catalogURI );
    // TODO: add regions
    OE_WARN << LC << "INTERNAL: Biome::getConfig() not fully implemented.\n";
    return conf;
}

