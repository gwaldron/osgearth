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
    mergeConfig( conf );
}

MapOptions::MapOptions( const TerrainOptions& to ) :
_proxySettings( ProxySettings() ),
_cacheOnly( false ),
_enableLighting( true ),
_terrainOptions( 0L )
{
    setTerrainOptions( to );
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
MapOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.key() = "map_options";

    conf.updateObjIfSet( "proxy", _proxySettings );
    conf.updateIfSet( "cache_only", _cacheOnly );
    conf.updateIfSet( "lighting", _enableLighting );
    conf.updateIfSet( "terrain_options", _terrainOptionsConf );

    return conf;
}

void
MapOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );

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
    _terrainOptionsConf = options.getConfig();
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
