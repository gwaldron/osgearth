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
    Common
    Compositing
    Cube
    DirectReadTileSource 
    EarthTerrainTechnique
	EarthTerrain
    ElevationManager
    Export
    FileUtils
	FileLocationCallback
	FindNode
    GeocentricMap
    GeoData
    HeightFieldUtils
    HTTPClient
    ImageToHeightFieldConverter
    ImageUtils
    JsonUtils
    MapNode
    MapConfig
    MapEngine
    Mercator
    Profile
    ProjectedMap
    Registry
    SpatialReference
    TerrainTileEdgeNormalizerUpdateCallback
    TileKey
    TileSource
    TileSourceFactory
    TMS
    XmlUtils
)

ADD_LIBRARY(${LIB_NAME} SHARED
#    ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    Caching.cpp
    CacheSeed.cpp
    Compositing.cpp
    Cube.cpp
    DirectReadTileSource.cpp
    EarthTerrainTechnique.cpp
    ElevationManager.cpp
    FileUtils.cpp
    GeocentricMap.cpp
    GeoData.cpp
    HeightFieldUtils.cpp
    HTTPClient.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    JsonUtils.cpp
    MapNode.cpp
    MapConfig.cpp
    MapEngine.cpp
    Mercator.cpp
    Profile.cpp
    ProjectedMap.cpp
    Registry.cpp
    SpatialReference.cpp
    TerrainTileEdgeNormalizerUpdateCallback.cpp
    TileKey.cpp
    TileSource.cpp
    TileSourceFactory.cpp
    TMS.cpp
    Version.cpp
    XmlUtils.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${EXPAT_INCLUDE_DIR} ${GDAL_INCLUDE_DIR})

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} ${EXPAT_LIBRARY} )
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY} ${EXPAT_LIBRARY} )
ENDIF(WIN32)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OPENTHREADS_LIBRARY CURL_LIBRARY GDAL_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
