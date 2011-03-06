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
    Caching
	CacheSeed
	Capabilities
    Common
    CompositeTileSource
    Config
    Cube
    ElevationLayer
    Export
    EGM
    FileUtils
	FindNode
    GeoData
    GeoMath
    HeightFieldUtils
    HTTPClient
    ImageToHeightFieldConverter
    ImageLayer
    ImageMosaic
    ImageUtils
    JsonUtils
    Layer
    Locators
    Map
    MapNode
    MapNodeOptions
    MapOptions
    MaskLayer
    MaskNode
    MaskSource
    ModelLayer
    ModelSource
	Notify
	OverlayDecorator
    Profile
	Progress
    Registry
    Revisioning
    ShaderComposition
    ShaderUtils
    SparseTexture2DArray
    SpatialReference
    StringUtils
    TaskService
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
    tinystr.h
    tinyxml.h 
    TMS
    Units
	Version
    VerticalSpatialReference
    XmlUtils
)

ADD_LIBRARY(${LIB_NAME} SHARED
#    ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    Caching.cpp
    CacheSeed.cpp
	Capabilities.cpp
    CompositeTileSource.cpp
    Config.cpp
    Cube.cpp
    EGM.cpp
    ElevationLayer.cpp
    FileUtils.cpp
    GeoData.cpp
    GeoMath.cpp
    HeightFieldUtils.cpp
    HTTPClient.cpp
    ImageLayer.cpp
    ImageMosaic.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    JsonUtils.cpp
    Layer.cpp
    Locators.cpp
    Map.cpp
    MapNode.cpp
    MapNodeOptions.cpp
    MapOptions.cpp
    MaskLayer.cpp
    MaskNode.cpp
    MaskSource.cpp
	MimeTypes.cpp
	ModelLayer.cpp
	ModelSource.cpp
	Notify.cpp
	OverlayDecorator.cpp
    Profile.cpp
	Progress.cpp
    Registry.cpp
    ShaderComposition.cpp
    ShaderUtils.cpp
    SparseTexture2DArray.cpp
    SpatialReference.cpp
    TaskService.cpp
    TerrainLayer.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TextureCompositor.cpp
    TextureCompositorMulti.cpp
    TextureCompositorTexArray.cpp
    TileFactory.cpp
    TileKey.cpp
    TileSource.cpp
    tinystr.cpp
    tinyxml.cpp
    tinyxmlerror.cpp
    tinyxmlparser.cpp
    TMS.cpp
    Units.cpp
    Version.cpp
    VerticalSpatialReference.cpp
    XmlUtils.cpp
)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} )
ENDIF(WIN32)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY ZLIB_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
