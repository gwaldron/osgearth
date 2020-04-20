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
#include <osgEarth/FeatureSource>
#include <osgEarth/Filter>

#define LC "[FeatureSource] " << getName() << ": "

using namespace osgEarth;
using namespace OpenThreads;

//...................................................................

Config
FeatureSource::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();

    conf.set( "open_write",   openWrite() );
    conf.set( "profile",      profile() );
    conf.set( "geo_interpolation", "great_circle", geoInterp(), GEOINTERP_GREAT_CIRCLE );
    conf.set( "geo_interpolation", "rhumb_line",   geoInterp(), GEOINTERP_RHUMB_LINE );
    conf.set( "fid_attribute", fidAttribute() );
    conf.set( "rewind_polygons", rewindPolygons());

    if (!filters().empty())
    {
        Config filtersConf;
        for(unsigned i=0; i<filters().size(); ++i)
        {
            filtersConf.add( filters()[i].getConfig() );
        }
        conf.set( "filters", filtersConf );
    }

    return conf;
}

void
FeatureSource::Options::fromConfig(const Config& conf)
{
    _rewindPolygons.init(true);

    conf.get( "open_write",   openWrite() );
    conf.get( "profile",      profile() );
    conf.get( "geo_interpolation", "great_circle", geoInterp(), GEOINTERP_GREAT_CIRCLE );
    conf.get( "geo_interpolation", "rhumb_line",   geoInterp(), GEOINTERP_RHUMB_LINE );
    conf.get( "fid_attribute", fidAttribute() );
    conf.get( "rewind_polygons", rewindPolygons());

#if 0
    // For backwards-compatibility (before adding the "filters" block)
    // TODO: Remove at some point in the distant future.
    const std::string bcstrings[3] = { "resample", "buffer", "convert" };
    for(unsigned i=0; i<3; ++i) {
        if ( conf.hasChild(bcstrings[i]) ) {
            _filterOptions.push_back( conf.child(bcstrings[i]) );
        }
    }
#endif

    const Config& filtersConf = conf.child("filters");
    for(ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back( *i );
}

//...................................................................

FeatureSource*
FeatureSource::create(const ConfigOptions& options)
{
    osg::ref_ptr<Layer> layer = Layer::create(options);
    FeatureSource* fs = dynamic_cast<FeatureSource*>(layer.get());
    if (fs)
    {
        layer.release();
        return fs;
    }
    return 0L;
}

//...................................................................

OE_LAYER_PROPERTY_IMPL(FeatureSource, bool, OpenWrite, openWrite);
OE_LAYER_PROPERTY_IMPL(FeatureSource, GeoInterpolation, GeoInterpolation, geoInterp);
OE_LAYER_PROPERTY_IMPL(FeatureSource, std::string, FIDAttribute, fidAttribute);
OE_LAYER_PROPERTY_IMPL(FeatureSource, bool, RewindPolygons, rewindPolygons);

void
FeatureSource::init()
{
    Layer::init();
}

Status
FeatureSource::openImplementation()
{
    Status parent = Layer::openImplementation();
    if (parent.isError())
        return parent;

    // Create and initialize the filters.
    _filters = FeatureFilterChain::create(options().filters(), getReadOptions());
    if (_filters.valid() && _filters->getStatus().isError())
    {
        return _filters->getStatus();
    }

    return Status::NoError;
}

const Status&
FeatureSource::create(
    const FeatureProfile* profile,
    const FeatureSchema& schema,
    const Geometry::Type& geometryType,
    const osgDB::Options* readOptions)
{
    return setStatus(Status::ResourceUnavailable, "Driver does not support create");
}

void
FeatureSource::setFeatureProfile(const FeatureProfile* fp)
{
    _featureProfile = fp;
}

const FeatureProfile*
FeatureSource::getFeatureProfile() const
{
    return _featureProfile.get();
}

const FeatureFilterChain*
FeatureSource::getFilters() const
{
    return _filters.get();
}

const FeatureSchema&
FeatureSource::getSchema() const
{
    static FeatureSchema s_emptySchema;
    return s_emptySchema;
}

void
FeatureSource::addToBlacklist( FeatureID fid )
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.insert( fid );
}

void
FeatureSource::removeFromBlacklist( FeatureID fid )
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.erase( fid );
}

void
FeatureSource::clearBlacklist()
{
    Threading::ScopedWriteLock exclusive( _blacklistMutex );
    _blacklist.clear();
}

bool
FeatureSource::isBlacklisted( FeatureID fid ) const
{
    Threading::ScopedReadLock lock(_blacklistMutex);
    return _blacklist.find( fid ) != _blacklist.end();
}

void
FeatureSource::applyFilters(FeatureList& features, const GeoExtent& extent) const
{
    // apply filters before returning.
    if (_filters.valid() && _filters->empty() == false)
    {
        FilterContext cx;
        cx.setProfile( getFeatureProfile() );
        cx.extent() = extent;
        for(FeatureFilterChain::const_iterator filter = _filters->begin(); filter != _filters->end(); ++filter)
        {
            cx = filter->get()->push( features, cx );
        }
    }
}
