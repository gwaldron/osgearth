/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
_numLoadingThreads( 4 ),
_numLoadingThreadsPerCore( 2 ),
_numCompileThreads( 2 ),
_numCompileThreadsPerCore( 0.5 )
{
    fromConfig( conf );
}

void
LoadingPolicy::fromConfig( const Config& conf )
{
    conf.getIfSet( "mode", "standard", _mode, MODE_SERIAL );
    conf.getIfSet( "mode", "serial", _mode, MODE_SERIAL );
    conf.getIfSet( "mode", "parallel", _mode, MODE_PARALLEL );
    conf.getIfSet( "mode", "sequential", _mode, MODE_SEQUENTIAL );
    conf.getIfSet( "mode", "preemptive", _mode, MODE_PREEMPTIVE );
    conf.getIfSet( "loading_threads", _numLoadingThreads );
    conf.getIfSet( "loading_threads_per_logical_processor", _numLoadingThreadsPerCore );
    conf.getIfSet( "loading_threads_per_core", _numLoadingThreadsPerCore );
    conf.getIfSet( "compile_threads", _numCompileThreads );
    conf.getIfSet( "compile_threads_per_core", _numCompileThreadsPerCore );
}

Config
LoadingPolicy::getConfig() const
{
    Config conf( "loading_policy" );
    conf.addIfSet( "mode", "standard", _mode, MODE_STANDARD ); // aka MODE_SERIAL
    conf.addIfSet( "mode", "parallel", _mode, MODE_PARALLEL );
    conf.addIfSet( "mode", "sequential", _mode, MODE_SEQUENTIAL );
    conf.addIfSet( "mode", "preemptive", _mode, MODE_PREEMPTIVE );
    conf.addIfSet( "loading_threads", _numLoadingThreads );
    conf.addIfSet( "loading_threads_per_core", _numLoadingThreadsPerCore );
    conf.addIfSet( "compile_threads", _numCompileThreads );
    conf.addIfSet( "compile_threads_per_core", _numCompileThreadsPerCore );
    return conf;
}

int osgEarth::computeLoadingThreads(const LoadingPolicy& policy)
{
    const char* env_numTaskServiceThreads = getenv("OSGEARTH_NUM_PREEMPTIVE_LOADING_THREADS");
    if ( env_numTaskServiceThreads )
    {
        return ::atoi( env_numTaskServiceThreads );
    }
    else if ( policy.numLoadingThreads().isSet() )
    {
        return osg::maximum( 1, policy.numLoadingThreads().get() );
    }
    else
    {
        return (int)osg::maximum( 1.0f, policy.numLoadingThreadsPerCore().get()
                                  * (float)OpenThreads::GetNumberOfProcessors() );
    }
}

//----------------------------------------------------------------------------

TerrainOptions::TerrainOptions( const ConfigOptions& options ) :
DriverConfigOptions( options ),
_verticalScale( 1.0f ),
_heightFieldSampleRatio( 1.0f ),
_minTileRangeFactor( 6.0 ),
_combineLayers( true ),
_loadingPolicy( LoadingPolicy() ),
_compositingTech( COMPOSITING_AUTO ),
_maxLOD( 23 ),
_minLOD( 0 ),
_firstLOD( 0 ),
_enableLighting( false ),
_attenuationDistance( 1000000 ),
_lodTransitionTimeSeconds( 0.5f ),
_enableMipmapping( true ),
_clusterCulling( true ),
_enableBlending( false ),
_mercatorFastPath( true ),
_minFilter( osg::Texture::LINEAR_MIPMAP_LINEAR ),
_magFilter( osg::Texture::LINEAR),
_primaryTraversalMask  ( 0xFFFFFFFF ),
_secondaryTraversalMask( 0x80000000 )
{
    fromConfig( _conf );
}

Config
TerrainOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.key() = "terrain";
    
    if ( _heightFieldSampleRatio.isSetTo( 0.0f ) )
        conf.update( "sample_ratio", "auto" );
    else
        conf.updateIfSet( "sample_ratio", _heightFieldSampleRatio );

    conf.updateObjIfSet( "loading_policy", _loadingPolicy );
    conf.updateIfSet( "vertical_scale", _verticalScale );
    conf.updateIfSet( "min_tile_range_factor", _minTileRangeFactor );    
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
    conf.updateIfSet( "primary_traversal_mask", _primaryTraversalMask );
    conf.updateIfSet( "secondary_traversal_mask", _secondaryTraversalMask );

    conf.updateIfSet( "compositor", "auto",             _compositingTech, COMPOSITING_AUTO );
    conf.updateIfSet( "compositor", "texture_array",    _compositingTech, COMPOSITING_TEXTURE_ARRAY );
    conf.updateIfSet( "compositor", "multitexture",     _compositingTech, COMPOSITING_MULTITEXTURE_GPU );
    conf.updateIfSet( "compositor", "multitexture_ffp", _compositingTech, COMPOSITING_MULTITEXTURE_FFP );
    conf.updateIfSet( "compositor", "multipass",        _compositingTech, COMPOSITING_MULTIPASS );

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
    if ( conf.value("sample_ratio") == "auto" )
        _heightFieldSampleRatio = 0.0f;
    else
        conf.getIfSet( "sample_ratio", _heightFieldSampleRatio );

    conf.getObjIfSet( "loading_policy", _loadingPolicy );
    conf.getIfSet( "vertical_scale", _verticalScale );
    conf.getIfSet( "min_tile_range_factor", _minTileRangeFactor );    
    conf.getIfSet( "max_lod", _maxLOD ); conf.getIfSet( "max_level", _maxLOD );
    conf.getIfSet( "min_lod", _minLOD ); conf.getIfSet( "min_level", _minLOD );
    conf.getIfSet( "first_lod", _firstLOD );
    conf.getIfSet( "lighting", _enableLighting );
    conf.getIfSet( "attenuation_distance", _attenuationDistance );
    conf.getIfSet( "lod_transition_time", _lodTransitionTimeSeconds );
    conf.getIfSet( "mipmapping", _enableMipmapping );
    conf.getIfSet( "cluster_culling", _clusterCulling );
    conf.getIfSet( "blending", _enableBlending );
    conf.getIfSet( "mercator_fast_path", _mercatorFastPath );
    conf.getIfSet( "primary_traversal_mask", _primaryTraversalMask );
    conf.getIfSet( "secondary_traversal_mask", _secondaryTraversalMask );

    conf.getIfSet( "compositor", "auto",             _compositingTech, COMPOSITING_AUTO );
    conf.getIfSet( "compositor", "texture_array",    _compositingTech, COMPOSITING_TEXTURE_ARRAY );
    conf.getIfSet( "compositor", "multitexture",     _compositingTech, COMPOSITING_MULTITEXTURE_GPU );
    conf.getIfSet( "compositor", "multitexture_gpu", _compositingTech, COMPOSITING_MULTITEXTURE_GPU );
    conf.getIfSet( "compositor", "multitexture_ffp", _compositingTech, COMPOSITING_MULTITEXTURE_FFP );
    conf.getIfSet( "compositor", "multipass",        _compositingTech, COMPOSITING_MULTIPASS );

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
