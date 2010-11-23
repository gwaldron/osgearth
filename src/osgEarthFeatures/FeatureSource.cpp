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
#include <osg/Notify>
#include <osgDB/ReadFile>
#include <OpenThreads/ScopedLock>

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace OpenThreads;


/****************************************************************************/

FeatureListCursor::FeatureListCursor( const FeatureList& features ) :
_features( features )
{
    _iter = _features.begin();
}

bool
FeatureListCursor::hasMore() const {
    return _iter != _features.end();
}

Feature*
FeatureListCursor::nextFeature() {
    Feature* r = _iter->get();
    _iter++;
    return r;
}

/****************************************************************************/

GeometryFeatureCursor::GeometryFeatureCursor( Geometry* geom ) :
_geom( geom )
{
    //nop
}

GeometryFeatureCursor::GeometryFeatureCursor(Geometry* geom,
                                             const FeatureProfile* fp,
                                             const FeatureFilterList& filters ) :
_geom( geom ),
_featureProfile( fp ),
_filters( filters )
{
    //nop
}

bool
GeometryFeatureCursor::hasMore() const {
    return _geom.valid();
}

Feature*
GeometryFeatureCursor::nextFeature()
{
    if ( hasMore() )
    {
        _lastFeature = new Feature();
        _lastFeature->setGeometry( _geom.get() );
        FilterContext cx;
        cx.profile() = _featureProfile.get();
        FeatureList list;
        list.push_back( _lastFeature.get() );
        for( FeatureFilterList::const_iterator i = _filters.begin(); i != _filters.end(); ++i ) {
            cx = i->get()->push( list, cx );
        }
        _geom = 0L;
    }
    return _lastFeature.get();
}

/****************************************************************************/

FeatureSourceOptions::FeatureSourceOptions( const ConfigOptions& options ) :
DriverConfigOptions( options )
{
    fromConfig( _conf );
}

void
FeatureSourceOptions::fromConfig( const Config& conf )
{
    const Config& bufferConf = conf.child("buffer");
    if ( !bufferConf.empty() )
    {
        BufferFilter* buffer = new BufferFilter();
        bufferConf.getIfSet( "distance", buffer->distance() );
        _filters.push_back( buffer );
    }

    // resample operation:
    // resampling must occur AFTER buffering, because the buffer op will remove colinear segments.
    const Config& resampleConf = conf.child("resample");
    if ( !resampleConf.empty() )
    {
        ResampleFilter* resample = new ResampleFilter();
        resampleConf.getIfSet( "min_length", resample->minLength() );
        resampleConf.getIfSet( "max_length", resample->maxLength() );
        _filters.push_back( resample );
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
    }

    return conf;
}

/****************************************************************************/

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
FeatureSource::getFilters() const {
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
            //featureSource->setName( options.getName() );
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
