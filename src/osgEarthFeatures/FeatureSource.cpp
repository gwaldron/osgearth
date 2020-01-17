/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ConvertTypeFilter>
#include <osgEarthFeatures/FilterContext>
#include <osgEarth/Registry>
#include <osg/Notify>
#include <osgDB/ReadFile>
#include <OpenThreads/ScopedLock>

#define LC "[FeatureSource] "

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

FeatureSourceOptions::FeatureSourceOptions(const ConfigOptions& options) :
DriverConfigOptions( options )
{
    fromConfig( _conf );
}

FeatureSourceOptions::~FeatureSourceOptions()
{
    //nop
}

void
FeatureSourceOptions::fromConfig(const Config& conf)
{
    unsigned numResamples = 0;

    conf.get( "open_write",   _openWrite );
    conf.get( "name",         _name );
    conf.get( "profile",      _profile );
    conf.get( "cache_policy", _cachePolicy );
    conf.get( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.get( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.get( "fid_attribute", _fidAttribute );

    // For backwards-compatibility (before adding the "filters" block)
    // TODO: Remove at some point in the distant future.
    const std::string bcstrings[3] = { "resample", "buffer", "convert" };
    for(unsigned i=0; i<3; ++i) {
        if ( conf.hasChild(bcstrings[i]) ) {
            _filterOptions.push_back( conf.child(bcstrings[i]) );
        }
    }

    const Config& filters = conf.child("filters");
    for(ConfigSet::const_iterator i = filters.children().begin(); i != filters.children().end(); ++i)
        _filterOptions.push_back( *i );
}

Config
FeatureSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();

    conf.set( "open_write",   _openWrite );
    conf.set( "name",         _name );
    conf.set( "profile",      _profile );
    conf.set( "cache_policy", _cachePolicy );
    conf.set( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.set( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.set( "fid_attribute", _fidAttribute );

    if ( !_filterOptions.empty() )
    {
        Config filters;
        for(unsigned i=0; i<_filterOptions.size(); ++i)
        {
            filters.add( _filterOptions[i].getConfig() );
        }
        conf.set( "filters", filters );
    }

    return conf;
}

//------------------------------------------------------------------------


FeatureSource::FeatureSource()
{    
    //nop
}

FeatureSource::FeatureSource(const ConfigOptions&  options) :
_options( options )
{
    //nop
}

FeatureSource::~FeatureSource()
{
    //nop
}

void
FeatureSource::setReadOptions(const osgDB::Options* readOptions)
{
    _readOptions = readOptions;
    _uriContext = URIContext(_readOptions.get());
}

const Status&
FeatureSource::open(const osgDB::Options* readOptions)
{
    setReadOptions(readOptions);
    return open();
}

const Status&
FeatureSource::open()
{    
    // Create and initialize the filters.
    for(unsigned i=0; i<_options.filters().size(); ++i)
    {
        const ConfigOptions& conf = _options.filters()[i];
        FeatureFilter* filter = FeatureFilterRegistry::instance()->create( conf.getConfig(), 0L );
        if ( filter )
        {
            if (_filters.valid() == false)
                _filters = new FeatureFilterChain();

            _filters->push_back( filter );
            filter->initialize(_readOptions.get());
        }
    }

    _status = initialize(_readOptions.get());
    return _status;
}

const Status&
FeatureSource::create(
    const FeatureProfile* profile,
    const FeatureSchema& schema,
    const Geometry::Type& geometryType,
    const osgDB::Options* readOptions)
{
    _status = Status::Error(Status::ResourceUnavailable, "Driver does not support create");
    return _status;
}

void
FeatureSource::setFeatureProfile(const FeatureProfile* fp)
{
    _featureProfile = fp;
}

const FeatureFilterChain*
FeatureSource::getFilters() const
{
    return _filters.get();
}

FeatureCursor*
FeatureSource::createFeatureCursor(ProgressCallback* progress)
{
    return createFeatureCursor(Symbology::Query(), progress);
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
    Threading::ScopedReadLock shared( const_cast<FeatureSource*>(this)->_blacklistMutex );
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

//------------------------------------------------------------------------

#undef  LC
#define LC "[FeatureSourceFactory] "
#define FEATURE_SOURCE_OPTIONS_TAG "__osgEarth::FeatureSourceOptions"

FeatureSource*
FeatureSourceFactory::create( const FeatureSourceOptions& options )
{
    osg::ref_ptr<FeatureSource> source;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_feature_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( FEATURE_SOURCE_OPTIONS_TAG, (void*)&options );

        osg::ref_ptr<osg::Object> object = osgDB::readRefObjectFile( driverExt, rwopts.get() );
        source = dynamic_cast<FeatureSource*>( object.release() );
        if ( source )
        {
            if ( options.name().isSet() )
                source->setName( *options.name() );
            else
                source->setName( options.getDriver() );
        }
        else
        {
            OE_WARN << LC << "FAILED to load feature driver \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    else
    {
        OE_WARN << LC << "ILLEGAL null feature driver name" << std::endl;
    }

    return source.release();
}

//------------------------------------------------------------------------

const FeatureSourceOptions&
FeatureSourceDriver::getFeatureSourceOptions( const osgDB::ReaderWriter::Options* rwopt ) const
{
    static FeatureSourceOptions s_default;
    const void* data = rwopt->getPluginData(FEATURE_SOURCE_OPTIONS_TAG);
    return data ? *static_cast<const FeatureSourceOptions*>(data) : s_default;
}
