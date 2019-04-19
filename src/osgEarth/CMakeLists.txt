IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

# CURL options
OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

# NVTT mipmap generation 
option(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS "Use NVTT, if available, to generate texture mipmaps on the CPU" OFF)
mark_as_advanced(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)
if(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)
    add_definitions(-DOSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)
endif(OSGEARTH_ENABLE_NVTT_CPU_MIPMAPS)

# TinyXML options
ADD_DEFINITIONS(-DTIXML_USE_STL)

# Builds the HTTPClient with WIN_INET instead of CURL
OPTION(OSGEARTH_USE_WININET_FOR_HTTP "Whether to use the WinInet library for HTTP requests (instead of cURL)" OFF)
if (OSGEARTH_USE_WININET_FOR_HTTP)
    ADD_DEFINITIONS(-DOSGEARTH_USE_WININET_FOR_HTTP)
endif (OSGEARTH_USE_WININET_FOR_HTTP)

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
    IF(CURL_IS_STATIC)
        ADD_DEFINITIONS(-DCURL_STATICLIB)
        SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm wldap32)
    ENDIF(CURL_IS_STATIC)
    LIST(APPEND TARGET_EXTERNAL_LIBRARIES psapi)
ENDIF(WIN32)

SET(LIB_NAME osgEarth)

set(TARGET_GLSL
    CascadeDraping.glsl
    ClipPlane.glsl
    DepthOffset.vert.glsl
    Draping.vert.glsl
    Draping.frag.glsl
    GPUClamping.vert.glsl
    GPUClamping.vert.lib.glsl
    GPUClamping.frag.glsl
    Instancing.vert.glsl
    LineDrawable.glsl
    PhongLighting.vert.glsl
    PhongLighting.frag.glsl
    PointDrawable.glsl
    Text.vert.glsl
    Text.frag.glsl
    Text_legacy.glsl
    )

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    Shaders.cpp.in)

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(LIB_PUBLIC_HEADERS
    Async
    Bounds
    Cache
    CacheEstimator
    CacheBin
    CachePolicy
    CacheSeed
    Capabilities
    CascadeDrapingDecorator
    Clamping
    ClampableNode
    ClampingTechnique
    ColorFilter
    Common
    CompositeTileSource
    Config
    Containers
    Cube
    CullingUtils
    DateTime
    DateTimeRange
    DepthOffset
    DrapeableNode
    DrapingCullSet
    DrapingTechnique
    DrawInstanced
    ECEF
    ElevationLayer
    ElevationLOD
    ElevationPool
    ElevationQuery
    EllipsoidIntersector
    Endian
    Export
    Extension
    FadeEffect
    FileUtils
    GeoCommon
    GeoData
    Geoid
    GeoMath
    GeoTransform
    GeometryClamper
    GLSLChunker
    GLUtils
    HeightFieldUtils
    Horizon
    HorizonClipPlane
    HTTPClient
    ImageLayer
    ImageMosaic
    ImageToHeightFieldConverter
    ImageUtils
    IntersectionPicker
    IOTypes
    JsonUtils
    LandCover
    LandCoverLayer
    Layer
    LayerListener
    LayerShader
    Lighting
    LineDrawable
    LineFunctor
    Locators
    LocalTangentPlane
    Map
    MapCallback
    MapFrame
    MapInfo
    MapModelChange
    MapNode
    MapNodeObserver
    MapNodeOptions
    MapOptions
    MaskLayer
    MaskSource
    Memory
    MemCache
    MetaTile
    Metrics
    ModelLayer
    ModelSource
    NativeProgramAdapter
    NodeUtils
    Notify
    optional
    ObjectIndex
    OverlayDecorator
    PagedNode
    PatchLayer
    PhongLightingEffect
    Picker
    PluginLoader
    PointDrawable
    PrimitiveIntersector
    Profile
    Progress
    Random
    Registry
    ResourceReleaser
    Revisioning
    SceneGraphCallback
    ScreenSpaceLayout
    Shaders
    ShaderFactory
    ShaderGenerator
    ShaderLoader
    ShaderMerger
    ShaderUtils
    Shadowing
    SimplexNoise
    SpatialReference
    StateSetCache
    Status
    StringUtils
    #TDTiles
    TaskService
    Terrain
    TerrainEffect
    TerrainLayer
    TerrainOptions
    TerrainEngineNode
    TerrainEngineRequirements
    TerrainResources
    TerrainTileModel
    TerrainTileModelFactory
    TerrainTileNode
    Tessellator
    Text
    TileKey
    TileHandler
    TileRasterizer
    TileSource
    TileVisitor
    TimeControl
    TraversalData
    ThreadingUtils
    Units
    URI
    Utils
    Version
    VerticalDatum
    VideoLayer
    Viewpoint
    VirtualProgram
    VisibleLayer
    XmlUtils
)


IF (NOT TINYXML_FOUND)
    SET(LIB_PUBLIC_HEADERS
        ${LIB_PUBLIC_HEADERS}
        tinystr.h
        tinyxml.h
    )
ENDIF (NOT TINYXML_FOUND)

IF (NOT TINYXML_FOUND)
    SET(TINYXML_SRC
        tinystr.cpp
        tinyxml.cpp
        tinyxmlerror.cpp
        tinyxmlparser.cpp
    )
ENDIF (NOT TINYXML_FOUND)


SET(VERSION_GIT_SOURCE "")
IF (OSGEARTH_EMBED_GIT_SHA)
	ADD_DEFINITIONS(-DOSGEARTH_EMBED_GIT_SHA)

	# auto-generate the VersionGit file to include the git SHA1 variable.
	configure_file(
		"${CMAKE_CURRENT_SOURCE_DIR}/VersionGit.cpp.in"
		"${CMAKE_CURRENT_BINARY_DIR}/VersionGit.cpp"
		@ONLY)

	set(VERSION_GIT_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/VersionGit.cpp")
ENDIF (OSGEARTH_EMBED_GIT_SHA)

set(TARGET_SRC
    Async.cpp
    Bounds.cpp
    Cache.cpp
    CacheBin.cpp
    CacheEstimator.cpp
    CachePolicy.cpp
    CacheSeed.cpp
    Capabilities.cpp
    CascadeDrapingDecorator.cpp
    Clamping.cpp
    ClampableNode.cpp
    ClampingTechnique.cpp
    ColorFilter.cpp
    CompositeTileSource.cpp
    Config.cpp
    Cube.cpp
    CullingUtils.cpp
    DateTime.cpp
    DateTimeRange.cpp
    DepthOffset.cpp
    DrapeableNode.cpp
    DrapingCullSet.cpp
    DrapingTechnique.cpp
    DrawInstanced.cpp
    ECEF.cpp
    ElevationLayer.cpp
    ElevationLOD.cpp
    ElevationPool.cpp
    ElevationQuery.cpp
    EllipsoidIntersector.cpp
    Extension.cpp
    FadeEffect.cpp
    FileUtils.cpp
    GeoData.cpp
    Geoid.cpp
    GeoMath.cpp
    GeoTransform.cpp
    GeometryClamper.cpp
    GLSLChunker.cpp
    GLUtils.cpp
    HeightFieldUtils.cpp
    Horizon.cpp
    HorizonClipPlane.cpp
    HTTPClient.cpp
    ImageLayer.cpp
    ImageMosaic.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    IntersectionPicker.cpp
    IOTypes.cpp
    JsonUtils.cpp
    LandCover.cpp
    LandCoverLayer.cpp
    Layer.cpp
    LayerShader.cpp
    Lighting.cpp
    LineDrawable.cpp
    Locators.cpp
    LocalTangentPlane.cpp
    Map.cpp
    MapCallback.cpp
    MapFrame.cpp
    MapInfo.cpp
    MapNode.cpp
    MapNodeOptions.cpp
    MapOptions.cpp
    MaskLayer.cpp
    MaskSource.cpp
    MemCache.cpp
    Memory.cpp
    MetaTile.cpp
    Metrics.cpp
    MimeTypes.cpp
    ModelLayer.cpp
    ModelSource.cpp
    NodeUtils.cpp
    Notify.cpp
    ObjectIndex.cpp
    OverlayDecorator.cpp
    PagedNode.cpp
    PatchLayer.cpp
    PhongLightingEffect.cpp
    PointDrawable.cpp
    PrimitiveIntersector.cpp
    Profile.cpp
    Progress.cpp
    Random.cpp
    Registry.cpp
    ResourceReleaser.cpp
    Revisioning.cpp
    SceneGraphCallback.cpp
    ScreenSpaceLayout.cpp
    ShaderFactory.cpp
    ShaderGenerator.cpp
    ShaderLoader.cpp
    ShaderMerger.cpp
    ShaderUtils.cpp
    Shadowing.cpp
    SimplexNoise.cpp
    SpatialReference.cpp
    StateSetCache.cpp
    Status.cpp
    StringUtils.cpp
    TaskService.cpp
    #TDTiles.cpp
    Terrain.cpp
    TerrainLayer.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TerrainResources.cpp
    TerrainTileModel.cpp
    TerrainTileModelFactory.cpp
    Tessellator.cpp
    Text.cpp
    TextureBufferSerializer.cpp
    TileKey.cpp
    TileHandler.cpp
    TileRasterizer.cpp
    TileVisitor.cpp
    TileSource.cpp
    TimeControl.cpp
    TraversalData.cpp
    ThreadingUtils.cpp
    Units.cpp
    URI.cpp
    Utils.cpp
    Version.cpp
    VerticalDatum.cpp
    VideoLayer.cpp
    Viewpoint.cpp
    VirtualProgram.cpp
    VisibleLayer.cpp
    XmlUtils.cpp
    ${SHADERS_CPP} )


ADD_LIBRARY(
    ${LIB_NAME} ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TINYXML_SRC}
    ${VERSION_GIT_SOURCE}
    ${TARGET_SRC}
    ${TARGET_GLSL}
    ${TARGET_IN} )

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

IF (TINYXML_FOUND)
    INCLUDE_DIRECTORIES(${TINYXML_INCLUDE_DIR})
ENDIF (TINYXML_FOUND)

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ENDIF(WIN32)

#LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY ZLIB_LIBRARY OSGMANIPULATOR_LIBRARY)

OPTION(NRL_STATIC_LIBRARIES "Link osgEarth against static GDAL and cURL, including static OpenSSL, Proj4, JPEG, PNG, and TIFF." OFF)
if(NOT NRL_STATIC_LIBRARIES)
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY OSGMANIPULATOR_LIBRARY)
else(NOT NRL_STATIC_LIBRARIES)
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY OSGMANIPULATOR_LIBRARY SSL_EAY_RELEASE LIB_EAY_RELEASE TIFF_LIBRARY PROJ4_LIBRARY PNG_LIBRARY JPEG_LIBRARY)
endif(NOT NRL_STATIC_LIBRARIES)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

IF (TINYXML_FOUND)
    LINK_WITH_VARIABLES(${LIB_NAME} TINYXML_LIBRARY)
    get_directory_property(output INCLUDE_DIRECTORIES)
    message(STATUS ${output})
ENDIF (TINYXML_FOUND)

INCLUDE(ModuleInstall OPTIONAL)
