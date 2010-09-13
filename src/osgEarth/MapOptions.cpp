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
#include <osgEarth/MapOptions>
#include <osg/Notify>
#include <OpenThreads/Thread>

using namespace osgEarth;

std::string MapOptions::OPTIONS_TAG = "__osgEarth::MapOptions";

//------------------------------------------------------------------------

LoadingPolicy::LoadingPolicy( const Config& conf ) :
_mode( MODE_STANDARD ),
_numThreads( 2 ),
_numThreadsPerCore( 4 ),
_numTileGenThreads( OpenThreads::GetNumberOfProcessors() )
{
    fromConfig( conf );
}

void
LoadingPolicy::fromConfig( const Config& conf )
{
    conf.getIfSet( "mode", "standard", _mode, MODE_STANDARD );
    conf.getIfSet( "mode", "sequential", _mode, MODE_SEQUENTIAL );
    conf.getIfSet( "mode", "preemptive", _mode, MODE_PREEMPTIVE );
    conf.getIfSet( "loading_threads", _numThreads );
    conf.getIfSet( "loading_threads_per_logical_processor", _numThreadsPerCore );
    conf.getIfSet( "loading_threads_per_core", _numThreadsPerCore );
    conf.getIfSet( "tile_generation_threads", _numTileGenThreads );
}

Config
LoadingPolicy::toConfig() const
{
    Config conf( "loading_policy" );
    conf.addIfSet( "mode", "standard", _mode, MODE_STANDARD );
    conf.addIfSet( "mode", "sequential", _mode, MODE_SEQUENTIAL );
    conf.addIfSet( "mode", "preemptive", _mode, MODE_PREEMPTIVE );
    conf.addIfSet( "loading_threads", _numThreads );
    //conf.addIfSet( "loading_threads_per_logical_processor", _numThreadsPerCore );
    conf.addIfSet( "loading_threads_per_core", _numThreadsPerCore );
    conf.addIfSet( "tile_generation_threads", _numTileGenThreads );
    return conf;
}

//----------------------------------------------------------------------------

ProxySettings::ProxySettings( const Config& conf )
{
    fromConfig( conf );
}

ProxySettings::ProxySettings( const std::string& host, int port ) :
_hostName(host),
_port(port)
{
    //nop
}

void
ProxySettings::fromConfig( const Config& conf )
{
    _hostName = conf.value<std::string>( "host", "" );
    _port = conf.value<int>( "port", 8080 );
	_userName = conf.value<std::string>( "username", "" );
	_password = conf.value<std::string>( "password", "" );
}

Config
ProxySettings::toConfig() const
{
    Config conf( "proxy" );
    conf.add( "host", _hostName );
    conf.add( "port", toString(_port) );
	conf.add( "username", _userName);
	conf.add( "password", _password);

    return conf;
}

//----------------------------------------------------------------------------

static TerrainOptions s_defaultTerrainOptions;

//----------------------------------------------------------------------------

MapOptions::MapOptions( const Config& conf ) :
ConfigOptions( conf ),
_proxySettings( ProxySettings() ),
_cacheOnly( false ),
_enableLighting( true ),
_terrainOptions( 0L )
{
    fromConfig( conf );
}

MapOptions::~MapOptions()
{
    if ( _terrainOptions )
    {
        delete _terrainOptions;
        _terrainOptions = 0L;
    }
}

Config
MapOptions::toConfig() const
{
    Config conf = ConfigOptions::toConfig();
    conf.key() = "map_options";

    conf.addObjIfSet( "proxy", _proxySettings );
    conf.addIfSet( "cache_only", _cacheOnly );
    conf.addIfSet( "lighting", _enableLighting );

    conf.addIfSet( "terrain_options", _terrainOptionsConf );

    return conf;
}

void
MapOptions::fromConfig( const Config& conf )
{
    ConfigOptions::fromConfig( conf );

    conf.getObjIfSet( "proxy", _proxySettings );
    conf.getIfSet( "cache_only", _cacheOnly );
    conf.getIfSet( "lighting", _enableLighting );

    if ( conf.hasChild( "terrain_options" ) )
    {
        _terrainOptionsConf = conf.child( "terrain_options" );
        if ( _terrainOptions )
        {
            delete _terrainOptions;
            _terrainOptions = 0L;
        }
    }
}

void
MapOptions::setTerrainOptions( const TerrainOptions& options )
{
    _terrainOptionsConf = options.toConfig();
    if ( _terrainOptions )
    {
        delete _terrainOptions;
        _terrainOptions = 0L;
    }
}

const TerrainOptions&
MapOptions::getTerrainOptions() const
{
    if ( _terrainOptionsConf.isSet() )
    {
        if ( !_terrainOptions )
        {
            const_cast<MapOptions*>(this)->_terrainOptions = new TerrainOptions( _terrainOptionsConf.value() );
        }
        return *_terrainOptions;
    }
    else
    {
        return s_defaultTerrainOptions;
    }
}
