IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthUtil)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(HEADERS_ROOT
    AnnotationEvents
    AutoClipPlaneHandler
    ArcGIS
    Common
    Controls
    ContourMap
    ClampCallback
    DataScanner
    DateTime
    DetailTexture
    EarthManipulator
    ExampleResources
    Export
    FeatureManipTool
    FeatureQueryTool
    Formatter
    GeodeticGraticule
    HTM
    LatLongFormatter
    LineOfSight
    LinearLineOfSight
    LODBlending
    MeasureTool
    MGRSFormatter
    MGRSGraticule
    MouseCoordsTool
    NormalMap
    ObjectLocator
    PolyhedralLineOfSight
    RadialLineOfSight
    SkyNode
    SpatialData
    StarData
    TerrainProfile
    TileIndex
    TileIndexBuilder
    TFS
    TFSPackager
    TMS
    TMSBackFiller
    TMSPackager
    UTMGraticule
    VerticalScale
    WFS
    WMS
)
IF(NOT ${OPENSCENEGRAPH_VERSION} VERSION_LESS "3.1.0")
    SET(HEADERS_ROOT ${HEADERS_ROOT}
        ShadowUtils)
ENDIF()
SOURCE_GROUP( Headers FILES ${HEADERS_ROOT} )


SET(SOURCES_ROOT
    AnnotationEvents.cpp
    ArcGIS.cpp
    AutoClipPlaneHandler.cpp
    ClampCallback.cpp
    Controls.cpp
    ContourMap.cpp
    DataScanner.cpp
    #DateTime.cpp #moved to osgEarth namespace
    DetailTexture.cpp
    EarthManipulator.cpp
    ExampleResources.cpp
    FeatureManipTool.cpp
    FeatureQueryTool.cpp
    GeodeticGraticule.cpp
    HTM.cpp
    LatLongFormatter.cpp
    LinearLineOfSight.cpp
    LODBlending.cpp
    MeasureTool.cpp
    MGRSFormatter.cpp
    MGRSGraticule.cpp
    MouseCoordsTool.cpp
    NormalMap.cpp
    ObjectLocator.cpp
    PolyhedralLineOfSight.cpp
    RadialLineOfSight.cpp
    SpatialData.cpp
    SkyNode.cpp
    TerrainProfile.cpp
    TileIndex.cpp
    TileIndexBuilder.cpp
    TFS.cpp
    TFSPackager.cpp
    TMS.cpp
    TMSBackFiller.cpp
    TMSPackager.cpp
    UTMGraticule.cpp
    VerticalScale.cpp
    WFS.cpp
    WMS.cpp
)
IF(NOT ${OPENSCENEGRAPH_VERSION} VERSION_LESS "3.1.0")
    SET(SOURCES_ROOT ${SOURCES_ROOT} 
        ShadowUtils.cpp)
ENDIF()

SOURCE_GROUP( Sources FILES ${SOURCES_ROOT} )


SET(HEADERS_COLORFILTER
    BrightnessContrastColorFilter
    CMYKColorFilter
    GammaColorFilter
    GLSLColorFilter
    HSLColorFilter
    RGBColorFilter
    ChromaKeyColorFilter
)
SOURCE_GROUP( Headers\\ColorFilters FILES ${HEADERS_COLORFILTER} )


SET(SOURCES_COLORFILTER
    BrightnessContrastColorFilter.cpp
    CMYKColorFilter.cpp
    GammaColorFilter.cpp
    GLSLColorFilter.cpp
    HSLColorFilter.cpp
    RGBColorFilter.cpp
    ChromaKeyColorFilter.cpp
)
SOURCE_GROUP( Sources\\ColorFilters FILES ${SOURCES_COLORFILTER} )


SET(LIB_PUBLIC_HEADERS
    ${HEADERS_ROOT}
    ${HEADERS_COLORFILTER}
)

SET(LIB_COMMON_FILES 
    ${SOURCES_ROOT} ${SOURCES_COLORFILTER}
)

ADD_LIBRARY(${LIB_NAME} ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${LIB_COMMON_FILES}
)

# Setting this tells ModuleInstall not to set source groups (since we're doing it here)
SET(USE_CUSTOM_SOURCE_GROUPS 1)



INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR} ${GDAL_INCLUDE_DIR})

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

LINK_INTERNAL(${LIB_NAME}
    osgEarth
    osgEarthFeatures
    osgEarthSymbology
    osgEarthAnnotation
)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGMANIPULATOR_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
