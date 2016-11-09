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
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ConvertTypeFilter>
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

    conf.getIfSet   ( "open_write",   _openWrite );
    conf.getIfSet   ( "name",         _name );
    conf.getObjIfSet( "profile",      _profile );
    conf.getObjIfSet( "cache_policy", _cachePolicy );
    conf.getIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.getIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.getIfSet   ( "fid_attribute", _fidAttribute );

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

    conf.updateIfSet   ( "open_write",   _openWrite );
    conf.updateIfSet   ( "name",         _name );
    conf.updateObjIfSet( "profile",      _profile );
    conf.updateObjIfSet( "cache_policy", _cachePolicy );
    conf.updateIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.updateIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.updateIfSet   ( "fid_attribute", _fidAttribute );

    if ( !_filterOptions.empty() )
    {
        Config filters;
        for(unsigned i=0; i<_filterOptions.size(); ++i)
        {
            filters.add( _filterOptions[i].getConfig() );
        }
        conf.update( "filters", filters );
    }

    return conf;
}

//------------------------------------------------------------------------

FeatureSource::FeatureSource(const ConfigOptions&  options,
                             const osgDB::Options* readOptions) :
_options( options )
{    
    _readOptions  = readOptions;
    _uriContext  = URIContext( _readOptions.get() );
}

FeatureSource::~FeatureSource()
{
    //nop
}

const Status&
FeatureSource::open(const osgDB::Options* readOptions)
{
    if ( readOptions )
        _readOptions = readOptions;
    
    // Create and initialize the filters.
    for(unsigned i=0; i<_options.filters().size(); ++i)
    {
        const ConfigOptions& conf = _options.filters().at(i);
        FeatureFilter* filter = FeatureFilterRegistry::instance()->create( conf.getConfig(), 0L );
        if ( filter )
        {
            _filters.push_back( filter );
            filter->initialize( readOptions );
        }
    }

    _status = initialize(readOptions);
    return _status;
}

void
FeatureSource::setFeatureProfile(const FeatureProfile* fp)
{
    _featureProfile = fp;
}

const FeatureFilterList&
FeatureSource::getFilters() const
{
    return _filters;
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
    if ( !getFilters().empty() )
    {
        FilterContext cx;
        cx.setProfile( getFeatureProfile() );
        cx.extent() = extent;
        for(FeatureFilterList::const_iterator filter = getFilters().begin(); filter != getFilters().end(); ++filter)
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
    FeatureSource* featureSource = 0L;

    if ( !options.getDriver().empty() )
    {
        std::string driverExt = std::string(".osgearth_feature_") + options.getDriver();

        osg::ref_ptr<osgDB::Options> rwopts = Registry::instance()->cloneOrCreateOptions();
        rwopts->setPluginData( FEATURE_SOURCE_OPTIONS_TAG, (void*)&options );

        featureSource = dynamic_cast<FeatureSource*>( osgDB::readObjectFile( driverExt, rwopts.get() ) );
        if ( featureSource )
        {
            if ( options.name().isSet() )
                featureSource->setName( *options.name() );
            else
                featureSource->setName( options.getDriver() );
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

    return featureSource;
}

//------------------------------------------------------------------------

const FeatureSourceOptions&
FeatureSourceDriver::getFeatureSourceOptions( const osgDB::ReaderWriter::Options* rwopt ) const
{
    static FeatureSourceOptions s_default;
    const void* data = rwopt->getPluginData(FEATURE_SOURCE_OPTIONS_TAG);
    return data ? *static_cast<const FeatureSourceOptions*>(data) : s_default;
}
