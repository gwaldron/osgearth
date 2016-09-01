IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

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
    AlphaEffect.frag.glsl
    DepthOffset.vert.glsl
    Draping.vert.glsl
    Draping.frag.glsl
    GPUClamping.vert.glsl
    GPUClamping.vert.lib.glsl
    GPUClamping.frag.glsl
    Instancing.vert.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    Shaders.cpp.in)

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(LIB_PUBLIC_HEADERS
    AlphaEffect
    AutoScale
    Bounds
    Cache
    CacheEstimator
    CacheBin
    CachePolicy
    CacheSeed
    Capabilities
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
    DPLineSegmentIntersector
    DrapeableNode
    DrapingCullSet
    DrapingTechnique
    DrawInstanced
    ECEF
    ElevationLayer
    ElevationLOD
    ElevationQuery
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
    HeightFieldUtils
    Horizon
    HTTPClient
    ImageLayer
    ImageMosaic
    ImageToHeightFieldConverter
    ImageUtils
    IOTypes
    JsonUtils
    Layer
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
    MaskNode
    MaskSource
    Memory
    MemCache
    ModelLayer
    ModelSource
    NativeProgramAdapter
    NodeUtils
    Notify
    optional
    ObjectIndex
    OverlayDecorator
    OverlayNode
    PhongLightingEffect
    Picker
    IntersectionPicker
    PrimitiveIntersector
    Profile
    Profiler
    Progress
    QuadTree
    Random
    Registry
    ResourceReleaser
    Revisioning
    ScreenSpaceLayout
    Shaders
    ShaderFactory
    ShaderGenerator
    ShaderLoader
    ShaderUtils
    Shadowing
    SharedSARepo
    SpatialReference
    StateSetCache
    StateSetLOD
    Status
    StringUtils
    TaskService
    Terrain
    TerrainEffect
    TerrainLayer
    TerrainOptions
    TerrainEngineNode
    TerrainEngineRequirements
    TerrainTileModel
    TerrainTileModelFactory
    TerrainTileNode
    TileKeyDataStore
    TilePatchCallback
    Tessellator
    TextureCompositor
    TileKey
    TileHandler
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
    Viewpoint
    VirtualProgram
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
    AlphaEffect.cpp
    AutoScale.cpp
    Bounds.cpp
    Cache.cpp
    CacheBin.cpp
    CacheEstimator.cpp
    CachePolicy.cpp
    CacheSeed.cpp
    Capabilities.cpp
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
    DPLineSegmentIntersector.cpp
    DrapeableNode.cpp
    DrapingCullSet.cpp
    DrapingTechnique.cpp
    DrawInstanced.cpp
    ECEF.cpp
    ElevationLayer.cpp
    ElevationLOD.cpp
    ElevationQuery.cpp
    Extension.cpp
    FadeEffect.cpp
    FileUtils.cpp
    GeoData.cpp
    Geoid.cpp
    GeoMath.cpp
    GeoTransform.cpp
    GeometryClamper.cpp
    GLSLChunker.cpp
    HeightFieldUtils.cpp
    Horizon.cpp
    HTTPClient.cpp
    ImageLayer.cpp
    ImageMosaic.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    IOTypes.cpp
    JsonUtils.cpp
    Layer.cpp
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
    MaskNode.cpp
    MaskSource.cpp
    MemCache.cpp
    Memory.cpp
    MimeTypes.cpp
    ModelLayer.cpp
    ModelSource.cpp
    NodeUtils.cpp
    Notify.cpp
    ObjectIndex.cpp
    OverlayDecorator.cpp
    OverlayNode.cpp
    PhongLightingEffect.cpp
    IntersectionPicker.cpp
    PrimitiveIntersector.cpp
    Profile.cpp
    Profiler.cpp
    Progress.cpp
    QuadTree.cpp
    Random.cpp
    Registry.cpp
    ResourceReleaser.cpp
    Revisioning.cpp
    ScreenSpaceLayout.cpp
    ShaderFactory.cpp
    ShaderGenerator.cpp
    ShaderLoader.cpp
    ShaderUtils.cpp
    Shadowing.cpp
    SpatialReference.cpp
    StateSetCache.cpp
    StateSetLOD.cpp
    Status.cpp
    StringUtils.cpp
    TaskService.cpp
    Terrain.cpp
    TerrainLayer.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TerrainTileModel.cpp
    TerrainTileModelFactory.cpp
    Tessellator.cpp
    TextureBufferSerializer.cpp
    TextureCompositor.cpp
    TileKey.cpp
    TileHandler.cpp
    TilePatchCallback.cpp
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
    Viewpoint.cpp
    VirtualProgram.cpp
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
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY ZLIB_LIBRARY OSGMANIPULATOR_LIBRARY)
else(NOT NRL_STATIC_LIBRARIES)
  LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY ZLIB_LIBRARY OSGMANIPULATOR_LIBRARY SSL_EAY_RELEASE LIB_EAY_RELEASE TIFF_LIBRARY PROJ4_LIBRARY PNG_LIBRARY JPEG_LIBRARY)
endif(NOT NRL_STATIC_LIBRARIES)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

IF (TINYXML_FOUND)
    LINK_WITH_VARIABLES(${LIB_NAME} TINYXML_LIBRARY)
    get_directory_property(output INCLUDE_DIRECTORIES)
    message(STATUS ${output})
ENDIF (TINYXML_FOUND)

INCLUDE(ModuleInstall OPTIONAL)
