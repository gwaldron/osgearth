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

#include <osgEarth/TerrainOptions>
#include <osg/Notify>
#include <OpenThreads/Thread>

using namespace osgEarth;

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
LoadingPolicy::getConfig() const
{
    Config conf( "loading_policy" );
    conf.addIfSet( "mode", "standard", _mode, MODE_STANDARD );
    conf.addIfSet( "mode", "sequential", _mode, MODE_SEQUENTIAL );
    conf.addIfSet( "mode", "preemptive", _mode, MODE_PREEMPTIVE );
    conf.addIfSet( "loading_threads", _numThreads );
    conf.addIfSet( "loading_threads_per_core", _numThreadsPerCore );
    conf.addIfSet( "tile_generation_threads", _numTileGenThreads );
    return conf;
}

//----------------------------------------------------------------------------

TerrainOptions::TerrainOptions( const ConfigOptions& options ) :
DriverConfigOptions( options ),
_loadingPolicy( LoadingPolicy() ),
_verticalScale( 1.0f ),
_heightFieldSampleRatio( 1.0f ),
_minTileRangeFactor( 6.0 ),
_normalizeEdges( false ),
_combineLayers( true ),
_maxLOD( 23 ),
_layeringTechnique( LAYERING_MULTITEXTURE ),
_enableLighting( false ),
_elevationInterpolation( INTERP_BILINEAR )
{
    fromConfig( _conf );
}

Config
TerrainOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.key() = "terrain_options";

    conf.updateObjIfSet( "loading_policy", _loadingPolicy );
    conf.updateIfSet( "vertical_scale", _verticalScale );
    conf.updateIfSet( "sample_ratio", _heightFieldSampleRatio );
    conf.updateIfSet( "min_tile_range_factor", _minTileRangeFactor );
    conf.updateIfSet( "normalize_edges", _normalizeEdges );
    conf.updateIfSet( "combine_layers", _combineLayers );
    conf.updateIfSet( "max_lod", _maxLOD );
    conf.updateIfSet( "lighting", _enableLighting );

    conf.updateIfSet( "layering_technique", "multipass", _layeringTechnique, LAYERING_MULTIPASS );
    conf.updateIfSet( "layering_technique", "multitexture", _layeringTechnique, LAYERING_MULTITEXTURE );
    conf.updateIfSet( "layering_technique", "composite", _layeringTechnique, LAYERING_COMPOSITE );

    conf.updateIfSet( "elevation_interpolation", "nearest",     _elevationInterpolation, INTERP_NEAREST);
    conf.updateIfSet( "elevation_interpolation", "average",     _elevationInterpolation, INTERP_AVERAGE);
    conf.updateIfSet( "elevation_interpolation", "bilinear",    _elevationInterpolation, INTERP_BILINEAR);
    conf.updateIfSet( "elevation_interpolation", "triangulate", _elevationInterpolation, INTERP_TRIANGULATE);

    return conf;
}

void
TerrainOptions::fromConfig( const Config& conf )
{
    conf.getObjIfSet( "loading_policy", _loadingPolicy );
    conf.getIfSet( "vertical_scale", _verticalScale );
    conf.getIfSet( "sample_ratio", _heightFieldSampleRatio );
    conf.getIfSet( "min_tile_range_factor", _minTileRangeFactor );
    conf.getIfSet( "normalize_edges", _normalizeEdges );
    conf.getIfSet( "combine_layers", _combineLayers );
    conf.getIfSet( "max_lod", _maxLOD );
    conf.getIfSet( "lighting", _enableLighting );

    conf.getIfSet( "layering_technique", "multipass", _layeringTechnique, LAYERING_MULTIPASS );
    conf.getIfSet( "layering_technique", "multitexture", _layeringTechnique, LAYERING_MULTITEXTURE );
    conf.getIfSet( "layering_technique", "composite", _layeringTechnique, LAYERING_COMPOSITE );

    conf.getIfSet( "elevation_interpolation", "nearest",     _elevationInterpolation, INTERP_NEAREST);
    conf.getIfSet( "elevation_interpolation", "average",     _elevationInterpolation, INTERP_AVERAGE);
    conf.getIfSet( "elevation_interpolation", "bilinear",    _elevationInterpolation, INTERP_BILINEAR);
    conf.getIfSet( "elevation_interpolation", "triangulate", _elevationInterpolation, INTERP_TRIANGULATE);
}
