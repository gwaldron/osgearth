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
#include <osgEarth/MaskLayer>
#include <osgEarth/Map>
#include <osgEarth/Registry>

#define LC "[MaskLayer] "

using namespace osgEarth;
//------------------------------------------------------------------------

MaskLayerOptions::MaskLayerOptions( const ConfigOptions& options ) :
ConfigOptions( options ),
_minLevel( 0 )
{
    setDefaults();
    fromConfig( _conf ); 
}

MaskLayerOptions::MaskLayerOptions( const std::string& name, const MaskSourceOptions& driverOptions ) :
ConfigOptions()
{
    setDefaults();
    fromConfig( _conf );
    _name = name;
    _driver = driverOptions;
}

void
MaskLayerOptions::setDefaults()
{
    _minLevel.init( 0 );
}

Config
MaskLayerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();

    conf.updateIfSet( "name", _name );
    conf.updateIfSet( "min_level", _minLevel );

    return conf;
}

void
MaskLayerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "name", _name );
    conf.getIfSet( "min_level", _minLevel );
}

void
MaskLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

MaskLayer::MaskLayer( const MaskLayerOptions& options ) :
_initOptions( options )
{
    copyOptions();
}

MaskLayer::MaskLayer( const std::string& name, const MaskSourceOptions& options ) :
_initOptions( MaskLayerOptions( name, options ) )
{
    copyOptions();
}

MaskLayer::MaskLayer( const MaskLayerOptions& options, MaskSource* source ) :
_maskSource( source ),
_initOptions( options )
{
    copyOptions();
}

void
MaskLayer::copyOptions()
{
    _runtimeOptions = _initOptions;
}

void
MaskLayer::setReadOptions(const osgDB::Options* readOptions)
{
    _readOptions = Registry::cloneOrCreateOptions(readOptions);
}

const Status&
MaskLayer::open()
{
    _status = initialize();
    return _status;
}

Status
MaskLayer::initialize()
{
    if ( !_maskSource.valid() && _initOptions.driver().isSet() )
    {
        _maskSource = MaskSourceFactory::create( *_initOptions.driver() );
        if (!_maskSource.valid())
        {
            return Status::Error(Status::ServiceUnavailable, Stringify()<<"Failed to create mask driver (" << _initOptions.driver()->getDriver() << ")");
        }
    }

    if ( _maskSource.valid() )
    {
        const Status& sourceStatus = _maskSource->open(_readOptions.get());  
        if (sourceStatus.isError())
        {
            return sourceStatus;
        }
    }

    return Status::OK();
}

osg::Vec3dArray*
MaskLayer::getOrCreateMaskBoundary( float heightScale, const SpatialReference *srs, ProgressCallback* progress )
{
    if (getStatus().isError())
    {
        return 0L;
    }

    OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _mutex );
    if ( _maskSource.valid() )
    {
        // if the model source has changed, regenerate the node.
        if ( _boundary.valid() && !_maskSource->inSyncWith(_maskSourceRev) )
        {
            _boundary = 0L;
        }

        if ( !_boundary.valid() )
        {
			_boundary = _maskSource->createBoundary( srs, progress );
            
            if (_boundary.valid())
            {
			    for (osg::Vec3dArray::iterator vIt = _boundary->begin(); vIt != _boundary->end(); ++vIt)
    				vIt->z() = vIt->z() * heightScale;

                _maskSource->sync( _maskSourceRev );
            }
        }
    }

    if (!_boundary.valid())
    {
        setStatus(Status::Error("Failed to create masking boundary"));
    }

    return _boundary.get();
}
