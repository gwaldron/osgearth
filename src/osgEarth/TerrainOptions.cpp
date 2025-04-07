/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "TerrainOptions"
#include "Notify"
#include "GLUtils"

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
    conf.set( "tessellation", _gpuTessellation );
    conf.set( "tessellation_level", tessellationLevel());
    conf.set( "tessellation_range", tessellationRange());
    conf.set( "debug", _debug );
    conf.set( "bin_number", _renderBinNumber );
    conf.set( "min_expiry_time", _minExpiryTime);
    conf.set( "min_expiry_frames", _minExpiryFrames);
    conf.set( "min_resident_tiles", minResidentTiles());
    conf.set( "max_tiles_to_unload_per_frame", _maxTilesToUnloadPerFrame);
    conf.set( "cast_shadows", _castShadows);
    conf.set( "tile_pixel_size", _tilePixelSize);
    conf.set( "lod_method", "screen_space", _lodMethod, LODMethod::SCREEN_SPACE);
    conf.set( "lod_method", "camera_distance", _lodMethod, LODMethod::CAMERA_DISTANCE);
    conf.set( "range_mode", "PIXEL_SIZE_ON_SCREEN", _lodMethod, LODMethod::SCREEN_SPACE); // backwards compatible
    conf.set( "range_mode", "DISTANCE_FROM_EYE_POINT", _lodMethod, LODMethod::CAMERA_DISTANCE); // backwards compatible
    conf.set( "skirt_ratio", heightFieldSkirtRatio() );
    conf.set( "color", color() );
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
    //conf.set("screen_space_error", screenSpaceError()); // don't serialize me, i'm set by the MapNode
    conf.set("max_texture_size", maxTextureSize());
    conf.set("visible", visible());
    conf.set("create_tiles_async", createTilesAsync());
    conf.set("create_tiles_grouped", createTilesGrouped());
    conf.set("restrict_polar_subdivision", restrictPolarSubdivision());

    conf.set("expiration_range", minExpiryRange()); // legacy
    conf.set("expiration_threshold", minResidentTiles()); // legacy

    return conf;
}

void
TerrainOptions::fromConfig(const Config& conf)
{
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
    conf.get( "tessellation", _gpuTessellation );
    conf.get( "gpu_tessellation", _gpuTessellation); //bc
    conf.get("tessellation_level", tessellationLevel());
    conf.get("tessellation_range", tessellationRange());
    conf.get( "debug", _debug );
    conf.get( "bin_number", _renderBinNumber );
    conf.get( "min_expiry_time", _minExpiryTime);
    conf.get( "min_expiry_frames", _minExpiryFrames);
    conf.get( "min_resident_tiles", minResidentTiles());
    conf.get( "max_tiles_to_unload_per_frame", _maxTilesToUnloadPerFrame);
    conf.get( "cast_shadows", _castShadows);
    conf.get( "tile_pixel_size", _tilePixelSize);
    conf.get( "lod_method", "screen_space", _lodMethod, LODMethod::SCREEN_SPACE);
    conf.get( "lod_method", "camera_distance", _lodMethod, LODMethod::CAMERA_DISTANCE);
    conf.get( "range_mode", "PIXEL_SIZE_ON_SCREEN", _lodMethod, LODMethod::SCREEN_SPACE); // backwards compatible
    conf.get( "range_mode", "DISTANCE_FROM_EYE_POINT", _lodMethod, LODMethod::CAMERA_DISTANCE); // backwards compatible
    conf.get( "skirt_ratio", heightFieldSkirtRatio() );
    conf.get( "color", color() );
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
    //conf.get("screen_space_error", screenSpaceError()); // don't serialize me, i'm set by the MapNode
    conf.get("max_texture_size", maxTextureSize());
    conf.get("visible", visible());

    conf.get("create_tiles_async", createTilesAsync());
    conf.get("create_tiles_grouped", createTilesGrouped());
    conf.get("restrict_polar_subdivision", restrictPolarSubdivision());

    conf.get("expiration_range", minExpiryRange()); // legacy
    conf.get("expiration_threshold", minResidentTiles()); // legacy

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
            OE_DEBUG << LC << "Deprecated key \"" << key << "\" ignored" << std::endl;
        }
    }
}

//...................................................................

namespace
{
    static TerrainOptions __default_to;
}

TerrainOptionsAPI::TerrainOptionsAPI() :
    _ptr(&__default_to)
{
    //nop
}
TerrainOptionsAPI::TerrainOptionsAPI(TerrainOptions* optionsPtr) :
    _ptr(optionsPtr)
{
    OE_HARD_ASSERT(_ptr != nullptr);
}

TerrainOptionsAPI::TerrainOptionsAPI(const TerrainOptionsAPI& rhs) :
    _ptr(rhs._ptr)
{
    OE_HARD_ASSERT(_ptr != nullptr);
}

OE_OPTION_IMPL(TerrainOptionsAPI, int, TileSize, tileSize);
OE_OPTION_IMPL(TerrainOptionsAPI, float, MinTileRangeFactor, minTileRangeFactor);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MaxLOD, maxLOD);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MinLOD, minLOD);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, FirstLOD, firstLOD);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, EnableLighting, enableLighting);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, ClusterCulling, clusterCulling);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, EnableBlending, enableBlending);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, CompressNormalMaps, compressNormalMaps);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MinNormalMapLOD, minNormalMapLOD);
OE_OPTION_IMPL(TerrainOptionsAPI, float, TessellationLevel, tessellationLevel);
OE_OPTION_IMPL(TerrainOptionsAPI, float, TessellationRange, tessellationRange);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, Debug, debug);
OE_OPTION_IMPL(TerrainOptionsAPI, int, RenderBinNumber, renderBinNumber);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, CastShadows, castShadows);
OE_OPTION_IMPL(TerrainOptionsAPI, LODMethod, LODMethod, lodMethod)
OE_OPTION_IMPL(TerrainOptionsAPI, float, TilePixelSize, tilePixelSize);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MinExpiryFrames, minExpiryFrames);
OE_OPTION_IMPL(TerrainOptionsAPI, double, MinExpiryTime, minExpiryTime);
OE_OPTION_IMPL(TerrainOptionsAPI, float, MinExpiryRange, minExpiryRange);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MaxTilesToUnloadPerFrame, maxTilesToUnloadPerFrame);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MinResidentTiles, minResidentTiles);
OE_OPTION_IMPL(TerrainOptionsAPI, float, HeightFieldSkirtRatio, heightFieldSkirtRatio);
OE_OPTION_IMPL(TerrainOptionsAPI, Color, Color, color);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, Progressive, progressive);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, UseNormalMaps, useNormalMaps);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, UseLandCover, useLandCover);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, NormalizeEdges, normalizeEdges);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, MorphTerrain, morphTerrain);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, MorphImagery, morphImagery);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MergesPerFrame, mergesPerFrame);
OE_OPTION_IMPL(TerrainOptionsAPI, float, PriorityScale, priorityScale);
OE_OPTION_IMPL(TerrainOptionsAPI, std::string, TextureCompressionMethod, textureCompression);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, Concurrency, concurrency);
OE_OPTION_IMPL(TerrainOptionsAPI, float, ScreenSpaceError, screenSpaceError);
OE_OPTION_IMPL(TerrainOptionsAPI, unsigned, MaxTextureSize, maxTextureSize);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, Visible, visible);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, CreateTilesAsync, createTilesAsync);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, CreateTilesGrouped, createTilesGrouped);
OE_OPTION_IMPL(TerrainOptionsAPI, bool, RestrictPolarSubdivision, restrictPolarSubdivision);

bool
TerrainOptionsAPI::getGPUTessellation() const
{
    return _ptr->gpuTessellation() == true && GLUtils::useNVGL() == true;
}

void
TerrainOptionsAPI::setGPUTessellation(bool value)
{
    _ptr->gpuTessellation() = value;
}

void
TerrainOptionsAPI::setDriver(const std::string& value)
{
    options().setDriver(value);
}
