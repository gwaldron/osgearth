/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

FeatureSourceOptions::UserFeatureFilters::UserFeatureFilters()
: osg::Referenced()
{
    // NOP
}

FeatureSourceOptions::UserFeatureFilters::~UserFeatureFilters()
{
    // NOP
}

FeatureFilterList& FeatureSourceOptions::UserFeatureFilters::filters()
{
    return _filters;
}

const FeatureFilterList& FeatureSourceOptions::UserFeatureFilters::filters() const
{
    return _filters;
}

void FeatureSourceOptions::UserFeatureFilters::setFilters(const FeatureFilterList& filters)
{
    _filters = filters;
}

FeatureSourceOptions::FeatureSourceOptions(const ConfigOptions& options) :
DriverConfigOptions( options )
{
    fromConfig( _conf );
}

void
FeatureSourceOptions::fromConfig( const Config& conf )
{
    unsigned numResamples = 0;

    conf.getIfSet   ( "open_write",   _openWrite );
    conf.getIfSet   ( "name",         _name );
    conf.getObjIfSet( "profile",      _profile );
    conf.getObjIfSet( "cache_policy", _cachePolicy );

    const ConfigSet& children = conf.children();
    for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
    {
        const Config& child = *i;

        if ( child.key() == "buffer" && !child.empty() )
        {
            BufferFilter* buffer = new BufferFilter();
            child.getIfSet( "distance", buffer->distance() );
            _filters.push_back( buffer );

            if ( numResamples > 0 )
            {
                OE_WARN << LC 
                    << "Warning: Resampling should be applied before buffering, as buffering"
                    << " will remove colinear segments created by the buffer operation."
                    << std::endl;
            }

            OE_DEBUG << LC << "Added buffer filter" << std::endl;
        }

        else if ( child.key() == "resample" && !child.empty() )
        {
            ResampleFilter* resample = new ResampleFilter();
            child.getIfSet( "min_length", resample->minLength() );
            child.getIfSet( "max_length", resample->maxLength() );
            _filters.push_back( resample );
            numResamples++;

            OE_DEBUG << LC << "Added resample filter" << std::endl;
        }

        else if ( child.key() == "convert" && !child.empty() )
        {
            ConvertTypeFilter* convert = new ConvertTypeFilter();
            optional<Geometry::Type> type = Geometry::TYPE_POINTSET;
            child.getIfSet( "type", "point",   type, Geometry::TYPE_POINTSET );
            child.getIfSet( "type", "line",    type, Geometry::TYPE_LINESTRING );
            child.getIfSet( "type", "polygon", type, Geometry::TYPE_POLYGON );
            convert->toType() = *type;
            _filters.push_back( convert );

            OE_DEBUG << LC << "Added convert filter" << std::endl;
        }
    }

    // Load user filters if any
    osg::ref_ptr<UserFeatureFilters> userFeatureFilters = conf.getNonSerializable<UserFeatureFilters>( "UserFeatureFilters" );
    if (userFeatureFilters.valid() == true)
    {
        for( FeatureFilterList::const_iterator i = userFeatureFilters->filters().begin(); i != userFeatureFilters->filters().end(); ++i )
        {
            _filters.push_back(i->get());
        }
    }
}

Config
FeatureSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();

    conf.updateIfSet   ( "open_write",   _openWrite );
    conf.updateIfSet   ( "name",         _name );
    conf.updateObjIfSet( "profile",      _profile );
    conf.updateObjIfSet( "cache_policy", _cachePolicy );

    osg::ref_ptr<UserFeatureFilters> userFeatureFilters = new UserFeatureFilters();

    //TODO: make each of these filters Configurable.
    for( FeatureFilterList::const_iterator i = _filters.begin(); i != _filters.end(); ++i )
    {
        BufferFilter* buffer = dynamic_cast<BufferFilter*>( i->get() );
        ResampleFilter* resample = dynamic_cast<ResampleFilter*>( i->get() );
        ConvertTypeFilter* convert = dynamic_cast<ConvertTypeFilter*>( i->get() );
        
        if ( buffer ) {
            Config bufferConf( "buffer" );
            bufferConf.addIfSet( "distance", buffer->distance() );
            conf.update( bufferConf );
        }
        else if ( resample ) { 
            Config resampleConf( "resample" );
            resampleConf.addIfSet( "min_length", resample->minLength() );
            resampleConf.addIfSet( "max_length", resample->maxLength() );
            conf.update( resampleConf );
        }
        else if ( convert ) {
            Config convertConf( "convert" );
            optional<Geometry::Type> type( convert->toType(), convert->toType() ); // weird optional ctor :)
            convertConf.addIfSet( "type", "point",   type, Geometry::TYPE_POINTSET );
            convertConf.addIfSet( "type", "line",    type, Geometry::TYPE_LINESTRING );
            convertConf.addIfSet( "type", "polygon", type, Geometry::TYPE_POLYGON );
            conf.update( convertConf );
        }
        else
        {
            // Consider this filter as a user feature filter
            userFeatureFilters->filters().push_back(i->get());
        }
    }

    // Add user filters if any
    if (userFeatureFilters->filters().empty() == false)
    {
        conf.updateNonSerializable( "UserFeatureFilters", userFeatureFilters.get() );
    }

    return conf;
}

//------------------------------------------------------------------------

FeatureSource::FeatureSource(const ConfigOptions&  options,
                             const osgDB::Options* dbOptions) :
_options( options )
{    
    _dbOptions  = dbOptions;
    _uriContext = URIContext( dbOptions );
    _cache      = Cache::get( dbOptions );
}

const FeatureProfile*
FeatureSource::getFeatureProfile() const
{
    if ( !_featureProfile.valid() )
    {
        FeatureSource* nonConstThis = const_cast<FeatureSource*>(this);

        ScopedLock<Mutex> doubleCheckLock( nonConstThis->_createMutex );
        {
            if ( !_featureProfile.valid() )
            {
                // caching pattern                
                nonConstThis->_featureProfile = nonConstThis->createFeatureProfile();
            }
        }
    }
    return _featureProfile.get();
}

const FeatureFilterList&
FeatureSource::getFilters() const
{
    return _options.filters();
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
    return *static_cast<const FeatureSourceOptions*>( rwopt->getPluginData( FEATURE_SOURCE_OPTIONS_TAG ) );
}
