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
    Common
    Controls
    ClampCallback
    DataScanner
    EarthManipulator    
    ExampleResources
    Export
    FeatureManipTool
    FeatureQueryTool
    Formatter
    GeodeticGraticule
    LatLongFormatter
    LineOfSight
    LinearLineOfSight
    MeasureTool
    MGRSFormatter
    MGRSGraticule
    MouseCoordsTool
    ObjectPlacer
    ObjectLocator
    PolyhedralLineOfSight
    RadialLineOfSight
    SkyNode
    SpatialData
    StarData
    TerrainProfile
    TFS
    TFSPackager
    TMS
    TMSBackFiller
    TMSPackager
    UTMGraticule
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
    AutoClipPlaneHandler.cpp
    ClampCallback.cpp
    Controls.cpp
    DataScanner.cpp
    EarthManipulator.cpp    
    ExampleResources.cpp
    FeatureManipTool.cpp
    FeatureQueryTool.cpp
    GeodeticGraticule.cpp
    LatLongFormatter.cpp
    LinearLineOfSight.cpp
    MeasureTool.cpp
    MGRSFormatter.cpp
    MGRSGraticule.cpp
    MouseCoordsTool.cpp
    ObjectPlacer.cpp
    ObjectLocator.cpp
    PolyhedralLineOfSight.cpp
    RadialLineOfSight.cpp
    SpatialData.cpp
    SkyNode.cpp
    TerrainProfile.cpp
    TFS.cpp
    TFSPackager.cpp
    TMS.cpp
    TMSBackFiller.cpp
    TMSPackager.cpp
    UTMGraticule.cpp
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
    HSLColorFilter
    RGBColorFilter
    ChromaKeyColorFilter
)
SOURCE_GROUP( Headers\\ColorFilters FILES ${HEADERS_COLORFILTER} )


SET(SOURCES_COLORFILTER
    BrightnessContrastColorFilter.cpp
    CMYKColorFilter.cpp
    GammaColorFilter.cpp
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



INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR})

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
