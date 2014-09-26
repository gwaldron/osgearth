/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/MapNodeOptions>
#include <osg/Notify>
#include <OpenThreads/Thread>

using namespace osgEarth;

std::string MapNodeOptions::OPTIONS_TAG = "__osgEarth::MapNodeOptions";

//----------------------------------------------------------------------------

static TerrainOptions s_defaultTerrainOptions;

//----------------------------------------------------------------------------

MapNodeOptions::MapNodeOptions( const Config& conf ) :
ConfigOptions          ( conf ),
_proxySettings         ( ProxySettings() ),
_cacheOnly             ( false ),
_enableLighting        ( true ),
_overlayBlending       ( true ),
_overlayMipMapping     ( false ),
_overlayTextureSize    ( 4096 ),
_terrainOptions        ( 0L ),
_overlayAttachStencil  ( false ),
_overlayResolutionRatio( 3.0f )
{
    mergeConfig( conf );
}

MapNodeOptions::MapNodeOptions( const TerrainOptions& to ) :
_proxySettings         ( ProxySettings() ),
_cacheOnly             ( false ),
_enableLighting        ( true ),
_overlayBlending       ( true ),
_overlayTextureSize    ( 4096 ),
_overlayMipMapping     ( false ),
_overlayAttachStencil  ( false ),
_overlayResolutionRatio( 3.0f ),
_terrainOptions        ( 0L )
{
    setTerrainOptions( to );
}

MapNodeOptions::MapNodeOptions( const MapNodeOptions& rhs ) :
_proxySettings         ( ProxySettings() ),
_cacheOnly             ( false ),
_enableLighting        ( true ),
_overlayBlending       ( true ),
_overlayTextureSize    ( 4096 ),
_overlayMipMapping     ( false ),
_overlayAttachStencil  ( false ),
_overlayResolutionRatio( 3.0f ),
_terrainOptions        ( 0L )
{
    mergeConfig( rhs.getConfig() );
}


MapNodeOptions::~MapNodeOptions()
{
    if ( _terrainOptions )
    {
        delete _terrainOptions;
        _terrainOptions = 0L;
    }
}

Config
MapNodeOptions::getConfig() const
{
    Config conf; // start with a fresh one since this is a FINAL object  // = ConfigOptions::getConfig();
    conf.key() = "options";

    conf.updateObjIfSet( "proxy",                    _proxySettings );
    conf.updateIfSet   ( "cache_only",               _cacheOnly );
    conf.updateIfSet   ( "lighting",                 _enableLighting );
    conf.updateIfSet   ( "terrain",                  _terrainOptionsConf );
    conf.updateIfSet   ( "overlay_warping",          _overlayVertexWarping );
    conf.updateIfSet   ( "overlay_blending",         _overlayBlending );
    conf.updateIfSet   ( "overlay_texture_size",     _overlayTextureSize );
    conf.updateIfSet   ( "overlay_mipmapping",       _overlayMipMapping );
    conf.updateIfSet   ( "overlay_attach_stencil",   _overlayAttachStencil );
    conf.updateIfSet   ( "overlay_resolution_ratio", _overlayResolutionRatio );

    return conf;
}

void
MapNodeOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );

    conf.getObjIfSet( "proxy",                    _proxySettings );
    conf.getIfSet   ( "cache_only",               _cacheOnly );
    conf.getIfSet   ( "lighting",                 _enableLighting );
    conf.getIfSet   ( "overlay_warping",          _overlayVertexWarping );
    conf.getIfSet   ( "overlay_blending",         _overlayBlending );
    conf.getIfSet   ( "overlay_texture_size",     _overlayTextureSize );
    conf.getIfSet   ( "overlay_mipmapping",       _overlayMipMapping );
    conf.getIfSet   ( "overlay_attach_stencil",   _overlayAttachStencil );
    conf.getIfSet   ( "overlay_resolution_ratio", _overlayResolutionRatio );

    if ( conf.hasChild( "terrain" ) )
    {
        _terrainOptionsConf = conf.child( "terrain" );
        if ( _terrainOptions )
        {
            delete _terrainOptions;
            _terrainOptions = 0L;
        }
    }
}

void
MapNodeOptions::setTerrainOptions( const TerrainOptions& options )
{
    _terrainOptionsConf = options.getConfig();
    if ( _terrainOptions )
    {
        delete _terrainOptions;
        _terrainOptions = 0L;
    }
}

const TerrainOptions&
MapNodeOptions::getTerrainOptions() const
{
    if ( _terrainOptionsConf.isSet() )
    {
        if ( !_terrainOptions )
        {
            const_cast<MapNodeOptions*>(this)->_terrainOptions = new TerrainOptions( _terrainOptionsConf.value() );
        }
        return *_terrainOptions;
    }
    else
    {
        return s_defaultTerrainOptions;
    }
}
