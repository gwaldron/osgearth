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

#include <osgEarth/TerrainOptions>
#include <osg/Notify>
#include <OpenThreads/Thread>

using namespace osgEarth;

//----------------------------------------------------------------------------

TerrainOptions::TerrainOptions( const ConfigOptions& options ) :
DriverConfigOptions( options ),
_tileSize( 17 ),
_verticalScale( 1.0f ),
_verticalOffset( 0.0f ),
_minTileRangeFactor( 7.0 ),
_combineLayers( true ),
_maxLOD( 23 ),
_minLOD( 0 ),
_firstLOD( 0 ),
_enableLighting( false ),
_attenuationDistance( 0.0f ),
_lodTransitionTimeSeconds( 0.5f ),
_enableMipmapping( true ),
_clusterCulling( true ),
_enableBlending( true ),
_mercatorFastPath( true ),
_minFilter( osg::Texture::LINEAR_MIPMAP_LINEAR ),
_magFilter( osg::Texture::LINEAR),
_minNormalMapLOD( 0u ),
_gpuTessellation( false ),
_debug( false ),
_binNumber( 0 )
{
    fromConfig( _conf );
}

Config
TerrainOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.key() = "terrain";
    
    conf.updateIfSet( "tile_size", _tileSize );
    conf.updateIfSet( "vertical_scale", _verticalScale );
    conf.updateIfSet( "vertical_offset", _verticalOffset );
    conf.updateIfSet( "min_tile_range_factor", _minTileRangeFactor );
    conf.updateIfSet( "range_factor", _minTileRangeFactor );  
    conf.updateIfSet( "max_lod", _maxLOD );
    conf.updateIfSet( "min_lod", _minLOD );
    conf.updateIfSet( "first_lod", _firstLOD );
    conf.updateIfSet( "lighting", _enableLighting );
    conf.updateIfSet( "attenuation_distance", _attenuationDistance );
    conf.updateIfSet( "lod_transition_time", _lodTransitionTimeSeconds );
    conf.updateIfSet( "mipmapping", _enableMipmapping );
    conf.updateIfSet( "cluster_culling", _clusterCulling );
    conf.updateIfSet( "blending", _enableBlending );
    conf.updateIfSet( "mercator_fast_path", _mercatorFastPath );
    conf.updateIfSet( "min_normal_map_lod", _minNormalMapLOD );
    conf.updateIfSet( "gpu_tessellation", _gpuTessellation );
    conf.updateIfSet( "debug", _debug );
    conf.updateIfSet( "bin_number", _binNumber );
    conf.updateIfSet( "min_expiry_time", _minExpiryTime);
    conf.updateIfSet( "min_expiry_frames", _minExpiryFrames);

    //Save the filter settings
	conf.updateIfSet("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.updateIfSet("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.updateIfSet("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.updateIfSet("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.updateIfSet("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.updateIfSet("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.updateIfSet("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.updateIfSet("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.updateIfSet("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.updateIfSet("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.updateIfSet("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.updateIfSet("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    return conf;
}

void
TerrainOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "tile_size", _tileSize );
    conf.getIfSet( "vertical_scale", _verticalScale );
    conf.getIfSet( "vertical_offset", _verticalOffset );
    conf.getIfSet( "min_tile_range_factor", _minTileRangeFactor );   
    conf.getIfSet( "range_factor", _minTileRangeFactor );   
    conf.getIfSet( "max_lod", _maxLOD ); conf.getIfSet( "max_level", _maxLOD );
    conf.getIfSet( "min_lod", _minLOD ); conf.getIfSet( "min_level", _minLOD );
    conf.getIfSet( "first_lod", _firstLOD ); conf.getIfSet( "first_level", _firstLOD );
    conf.getIfSet( "lighting", _enableLighting );
    conf.getIfSet( "attenuation_distance", _attenuationDistance );
    conf.getIfSet( "lod_transition_time", _lodTransitionTimeSeconds );
    conf.getIfSet( "mipmapping", _enableMipmapping );
    conf.getIfSet( "cluster_culling", _clusterCulling );
    conf.getIfSet( "blending", _enableBlending );
    conf.getIfSet( "mercator_fast_path", _mercatorFastPath );
    conf.getIfSet( "min_normal_map_lod", _minNormalMapLOD );
    conf.getIfSet( "gpu_tessellation", _gpuTessellation );
    conf.getIfSet( "debug", _debug );
    conf.getIfSet( "bin_number", _binNumber );
    conf.getIfSet( "min_expiry_time", _minExpiryTime);
    conf.getIfSet( "min_expiry_frames", _minExpiryFrames);

    //Load the filter settings
	conf.getIfSet("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.getIfSet("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.getIfSet("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.getIfSet("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.getIfSet("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.getIfSet("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.getIfSet("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.getIfSet("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.getIfSet("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.getIfSet("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.getIfSet("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.getIfSet("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
}
