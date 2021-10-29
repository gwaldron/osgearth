/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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

using namespace osgEarth;

#undef LC
#define LC "[TerrainOptions] "

//...................................................................

Config
TerrainOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.key() = "terrain";
    
    conf.set( "tile_size", _tileSize );
    conf.set( "min_tile_range_factor", _minTileRangeFactor );
    conf.set( "range_factor", _minTileRangeFactor );  
    conf.set( "max_lod", _maxLOD );
    conf.set( "min_lod", _minLOD );
    conf.set( "first_lod", _firstLOD );
    conf.set( "lighting", _enableLighting );
    conf.set( "cluster_culling", _clusterCulling );
    conf.set( "blending", _enableBlending );
    conf.set( "compress_normal_maps", _compressNormalMaps);
    conf.set( "min_normal_map_lod", _minNormalMapLOD );
    conf.set( "gpu_tessellation", _gpuTessellation );
    conf.set( "debug", _debug );
    conf.set( "bin_number", _renderBinNumber );
    conf.set( "min_expiry_time", _minExpiryTime);
    conf.set( "min_expiry_frames", _minExpiryFrames);
    conf.set( "max_tiles_to_unload_per_frame", _maxTilesToUnloadPerFrame);
    conf.set( "cast_shadows", _castShadows);
    conf.set( "tile_pixel_size", _tilePixelSize);
    conf.set( "range_mode", "PIXEL_SIZE_ON_SCREEN", _rangeMode, osg::LOD::PIXEL_SIZE_ON_SCREEN);
    conf.set( "range_mode", "DISTANCE_FROM_EYE_POINT", _rangeMode, osg::LOD::DISTANCE_FROM_EYE_POINT);
    conf.set( "skirt_ratio", heightFieldSkirtRatio() );
    conf.set( "color", color() );
    conf.set( "expiration_range", minExpiryRange() );
    conf.set( "expiration_threshold", expirationThreshold() );
    conf.set( "progressive", progressive() );
    conf.set( "use_normal_maps", useNormalMaps() );
    conf.set( "normalize_edges", normalizeEdges() );
    conf.set( "morph_terrain", morphTerrain() );
    conf.set( "morph_elevation", morphTerrain() );
    conf.set( "morph_imagery", morphImagery() );
    conf.set( "merges_per_frame", mergesPerFrame() );
    conf.set( "priority_scale", priorityScale() );
    conf.set( "texture_compression", textureCompression());
    conf.set( "concurrency", concurrency());
    conf.set( "use_land_cover", useLandCover() );

    return conf;
}

void
TerrainOptions::fromConfig(const Config& conf)
{
    tileSize().setDefault(17);
    minTileRangeFactor().setDefault(7.0);
    maxLOD().setDefault(19u);
    minLOD().setDefault(0u);
    firstLOD().setDefault(0u);
    enableLighting().setDefault(true);
    clusterCulling().setDefault(true);
    enableBlending().setDefault(true);
    compressNormalMaps().setDefault(false);
    minNormalMapLOD().setDefault(0);
    gpuTessellation().setDefault(false);
    debug().setDefault(false);
    renderBinNumber().setDefault(0);
    castShadows().setDefault(false);
    rangeMode().setDefault(osg::LOD::DISTANCE_FROM_EYE_POINT);
    tilePixelSize().setDefault(256);
    minExpiryFrames().setDefault(0);
    minExpiryTime().setDefault(0.0);
    minExpiryRange().setDefault(0.0f);
    maxTilesToUnloadPerFrame().setDefault(~0u);
    heightFieldSkirtRatio().setDefault(0.0f);
    color().setDefault(osg::Vec4f(1,1,1,1));
    expirationThreshold().setDefault(300u);
    progressive().setDefault(false);
    useNormalMaps().setDefault(true);
    normalizeEdges().setDefault(false);
    morphTerrain().setDefault(true);
    morphImagery().setDefault(true);
    mergesPerFrame().setDefault(20u);
    priorityScale().setDefault(1.0f);
    textureCompression().setDefault("");
    concurrency().setDefault(4u);
    useLandCover().setDefault(true);

    conf.get( "tile_size", _tileSize );
    conf.get( "min_tile_range_factor", _minTileRangeFactor );   
    conf.get( "range_factor", _minTileRangeFactor );   
    conf.get( "max_lod", _maxLOD ); conf.get( "max_level", _maxLOD );
    conf.get( "min_lod", _minLOD ); conf.get( "min_level", _minLOD );
    conf.get( "first_lod", _firstLOD ); conf.get( "first_level", _firstLOD );
    conf.get( "lighting", _enableLighting );
    conf.get( "cluster_culling", _clusterCulling );
    conf.get( "blending", _enableBlending );
    conf.get( "compress_normal_maps", _compressNormalMaps);
    conf.get( "min_normal_map_lod", _minNormalMapLOD );
    conf.get( "gpu_tessellation", _gpuTessellation );
    conf.get( "debug", _debug );
    conf.get( "bin_number", _renderBinNumber );
    conf.get( "min_expiry_time", _minExpiryTime);
    conf.get( "min_expiry_frames", _minExpiryFrames);
    conf.get( "max_tiles_to_unload_per_frame", _maxTilesToUnloadPerFrame);
    conf.get( "cast_shadows", _castShadows);
    conf.get( "tile_pixel_size", _tilePixelSize);
    conf.get( "range_mode", "PIXEL_SIZE_ON_SCREEN", rangeMode(), osg::LOD::PIXEL_SIZE_ON_SCREEN);
    conf.get( "range_mode", "pixel_size", rangeMode(), osg::LOD::PIXEL_SIZE_ON_SCREEN);
    conf.get( "range_mode", "DISTANCE_FROM_EYE_POINT", rangeMode(), osg::LOD::DISTANCE_FROM_EYE_POINT);
    conf.get( "range_mode", "distance", rangeMode(), osg::LOD::DISTANCE_FROM_EYE_POINT);
    conf.get( "skirt_ratio", heightFieldSkirtRatio() );
    conf.get( "color", color() );
    conf.get( "expiration_range", minExpiryRange() );
    conf.get( "expiration_threshold", expirationThreshold() );
    conf.get( "progressive", progressive() );
    conf.get( "use_normal_maps", useNormalMaps() );
    conf.get( "normal_maps", useNormalMaps()); // backwards compatible
    conf.get( "normalize_edges", normalizeEdges() );
    conf.get( "morph_terrain", morphTerrain() );
    conf.get( "morph_imagery", morphImagery() );
    conf.get( "merges_per_frame", mergesPerFrame() );
    conf.get( "priority_scale", priorityScale());
    conf.get( "texture_compression", textureCompression());
    conf.get( "concurrency", concurrency());
    conf.get( "use_land_cover", useLandCover());

    // report on deprecated usage
    const std::string deprecated_keys[] = {
        "compress_normal_maps",
        "min_expiry_frames",
        "expiration_threshold",
        "priority_scale"
    };
    for (const auto& key : deprecated_keys)
    {
        if (conf.hasValue(key))
        {
            OE_INFO << LC << "Deprecated key \"" << key << "\" ignored" << std::endl;
        }
    }
}

//...................................................................

TerrainOptionsAPI::TerrainOptionsAPI(TerrainOptions* optionsPtr) :
_ptr(optionsPtr)
{
    //nop
}

OE_PROPERTY_IMPL(TerrainOptionsAPI, int, TileSize, tileSize);
OE_PROPERTY_IMPL(TerrainOptionsAPI, float, MinTileRangeFactor, minTileRangeFactor);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, MaxLOD, maxLOD);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, MinLOD, minLOD);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, FirstLOD, firstLOD);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, EnableLighting, enableLighting);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, ClusterCulling, clusterCulling);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, EnableBlending, enableBlending);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, CompressNormalMaps, compressNormalMaps);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, MinNormalMapLOD, minNormalMapLOD);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, GPUTessellation, gpuTessellation);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, Debug, debug);
OE_PROPERTY_IMPL(TerrainOptionsAPI, int, RenderBinNumber, renderBinNumber);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, CastShadows, castShadows);
OE_PROPERTY_IMPL(TerrainOptionsAPI, osg::LOD::RangeMode, RangeMode, rangeMode);
OE_PROPERTY_IMPL(TerrainOptionsAPI, float, TilePixelSize, tilePixelSize);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, MinExpiryFrames, minExpiryFrames);
OE_PROPERTY_IMPL(TerrainOptionsAPI, double, MinExpiryTime, minExpiryTime);
OE_PROPERTY_IMPL(TerrainOptionsAPI, float, MinExpiryRange, minExpiryRange);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, MaxTilesToUnloadPerFrame, maxTilesToUnloadPerFrame);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, ExpirationThreshold, expirationThreshold);
OE_PROPERTY_IMPL(TerrainOptionsAPI, float, HeightFieldSkirtRatio, heightFieldSkirtRatio);
OE_PROPERTY_IMPL(TerrainOptionsAPI, Color, Color, color);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, Progressive, progressive);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, UseNormalMaps, useNormalMaps);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, UseLandCover, useLandCover);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, NormalizeEdges, normalizeEdges);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, MorphTerrain, morphTerrain);
OE_PROPERTY_IMPL(TerrainOptionsAPI, bool, MorphImagery, morphImagery);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, MergesPerFrame, mergesPerFrame);
OE_PROPERTY_IMPL(TerrainOptionsAPI, float, PriorityScale, priorityScale);
OE_PROPERTY_IMPL(TerrainOptionsAPI, std::string, TextureCompressionMethod, textureCompression);
OE_PROPERTY_IMPL(TerrainOptionsAPI, unsigned, Concurrency, concurrency);

void
TerrainOptionsAPI::setDriver(const std::string& value)
{
    options().setDriver(value);
}
