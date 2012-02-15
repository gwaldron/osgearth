IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

IF(WIN32)
    SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    IF(CURL_IS_STATIC)
        ADD_DEFINITIONS(-DCURL_STATICLIB)
        SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm wldap32)
    ENDIF(CURL_IS_STATIC)
ENDIF(WIN32)

SET(LIB_NAME osgEarth)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})
SET(LIB_PUBLIC_HEADERS
    Bounds
    Cache
    CacheBin
    CachePolicy
	CacheSeed
	Capabilities
    Common
    CompositeTileSource
    Config
    Containers
    Cube
    CullingUtils
    DepthOffset
    DPLineSegmentIntersector
    DrapeableNode
    ECEF
    ElevationLayer
    ElevationQuery
    Export
    FileUtils
	FindNode
    GeoData
    Geoid
    GeoMath
    HeightFieldUtils
    HTTPClient
    ImageToHeightFieldConverter
    ImageLayer
    ImageMosaic
    ImageUtils
    IOTypes
    JsonUtils
    Layer
    LineFunctor
    Locators
    LocalTangentPlane
    Map
    MapNode
    MapNodeOptions
    MapOptions
    MaskLayer
    MaskNode
    MaskSource
    MemCache
    ModelLayer
    ModelSource
    NodeUtils
	Notify
	OverlayDecorator
    Profile
	Progress
	Random
    Registry
    Revisioning
    ShaderComposition
    ShaderUtils
    SparseTexture2DArray
    SpatialReference
    StringUtils
    TaskService
    Terrain
    TerrainLayer
    TerrainOptions
    TerrainEngineNode
    TextureCompositor
    TextureCompositorMulti
    TextureCompositorTexArray
    TileFactory
    TileKey
    TileSource
    ThreadingUtils
    Units
    URI
    Utils
	Version
	VerticalDatum
    Viewpoint
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

ADD_LIBRARY(${LIB_NAME} SHARED
#    ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TINYXML_SRC}
    Bounds.cpp
    Cache.cpp
    CachePolicy.cpp
    CacheSeed.cpp
	Capabilities.cpp
    CompositeTileSource.cpp
    Config.cpp
    Cube.cpp
    CullingUtils.cpp
    DepthOffset.cpp
    DPLineSegmentIntersector.cpp
    DrapeableNode.cpp
    ECEF.cpp
    ElevationLayer.cpp
    ElevationQuery.cpp
    FileUtils.cpp
    GeoData.cpp
    Geoid.cpp
    GeoMath.cpp
    HeightFieldUtils.cpp
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
    MapNode.cpp
    MapNodeOptions.cpp
    MapOptions.cpp
    MaskLayer.cpp
    MaskNode.cpp
    MaskSource.cpp
    MemCache.cpp
	MimeTypes.cpp
	ModelLayer.cpp
	ModelSource.cpp
	NodeUtils.cpp
	Notify.cpp
	OverlayDecorator.cpp
    Profile.cpp
	Progress.cpp
	Random.cpp
    Registry.cpp
    ShaderComposition.cpp
    ShaderUtils.cpp
    SparseTexture2DArray.cpp
    SpatialReference.cpp
    StringUtils.cpp
    TaskService.cpp
    Terrain.cpp
    TerrainLayer.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TextureCompositor.cpp
    TextureCompositorMulti.cpp
    TextureCompositorTexArray.cpp
    TileFactory.cpp
    TileKey.cpp
    TileSource.cpp
    Units.cpp
    URI.cpp
    Utils.cpp
    Version.cpp
	VerticalDatum.cpp
    Viewpoint.cpp
    XmlUtils.cpp
)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

IF (TINYXML_FOUND)
    INCLUDE_DIRECTORIES(${TINYXML_INCLUDE_DIR})
ENDIF (TINYXML_FOUND)

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ENDIF(WIN32)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY ZLIB_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

IF (TINYXML_FOUND)
    LINK_WITH_VARIABLES(${LIB_NAME} TINYXML_LIBRARY)
    get_directory_property(output INCLUDE_DIRECTORIES)
    message(STATUS ${output})
ENDIF (TINYXML_FOUND)

INCLUDE(ModuleInstall OPTIONAL)
