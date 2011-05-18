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
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/ResampleFilter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ConvertTypeFilter>
#include <osg/Notify>
#include <osgDB/ReadFile>
#include <OpenThreads/ScopedLock>

#define LC "[FeatureSource] "

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

FeatureSourceOptions::FeatureSourceOptions( const ConfigOptions& options ) :
DriverConfigOptions( options )
{
    fromConfig( _conf );
}

void
FeatureSourceOptions::fromConfig( const Config& conf )
{
    unsigned numResamples = 0;

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
}

Config
FeatureSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();

    //TODO: make each of these filters Configurable.
    for( FeatureFilterList::const_iterator i = _filters.begin(); i != _filters.end(); ++i )
    {
        BufferFilter* buffer = dynamic_cast<BufferFilter*>( i->get() );
        if ( buffer ) {
            Config bufferConf( "buffer" );
            bufferConf.addIfSet( "distance", buffer->distance() );
            conf.update( bufferConf );
        }

        ResampleFilter* resample = dynamic_cast<ResampleFilter*>( i->get() );
        if ( resample ) { 
            Config resampleConf( "resample" );
            resampleConf.addIfSet( "min_length", resample->minLength() );
            resampleConf.addIfSet( "max_length", resample->maxLength() );
            conf.update( resampleConf );
        }

        ConvertTypeFilter* convert = dynamic_cast<ConvertTypeFilter*>( i->get() );
        if ( convert ) {
            Config convertConf( "convert" );
            optional<Geometry::Type> type( convert->toType(), convert->toType() ); // weird optional ctor :)
            convertConf.addIfSet( "type", "point",   type, Geometry::TYPE_POINTSET );
            convertConf.addIfSet( "type", "line",    type, Geometry::TYPE_LINESTRING );
            convertConf.addIfSet( "type", "polygon", type, Geometry::TYPE_POLYGON );
            conf.update( convertConf );
        }
    }

    return conf;
}

//------------------------------------------------------------------------

FeatureSource::FeatureSource( const ConfigOptions& options ) :
_options( options )
{    
    //nop
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

        osg::ref_ptr<osgDB::ReaderWriter::Options> rwopts = new osgDB::ReaderWriter::Options();
        rwopts->setPluginData( FEATURE_SOURCE_OPTIONS_TAG, (void*)&options );

        featureSource = dynamic_cast<FeatureSource*>( osgDB::readObjectFile( driverExt, rwopts.get() ) );
        if ( featureSource )
        {
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
