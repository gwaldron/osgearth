IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthUtil)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(HEADERS_ROOT
	ActivityMonitorTool
    AnnotationEvents
    AutoClipPlaneHandler	
    ArcGIS
    AtlasBuilder
    Common
    Controls
    ContourMap
    ClampCallback
    DataScanner
    EarthManipulator
	Ephemeris
    ExampleResources
    Export
    FeatureQueryTool
    Fog
    Formatter
    GeodeticGraticule
    GraticuleExtension
    GraticuleNode
    GraticuleOptions
    GraticuleTerrainEffect
    HTM
    LatLongFormatter
    LineOfSight
    LinearLineOfSight
    LODBlending
	LogarithmicDepthBuffer
    MeasureTool
    MGRSFormatter
    MGRSGraticule
    MouseCoordsTool
    ObjectLocator
    Ocean
    PolyhedralLineOfSight
    RadialLineOfSight
    RTTPicker
    Shaders
	Shadowing
	SimplexNoise
    SimplePager
    Sky
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
SOURCE_GROUP( Headers FILES ${HEADERS_ROOT} )


# Generate inline shaders.
set(SHADERS_CPP
    "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_GLSL
    ContourMap.vert.glsl
    ContourMap.frag.glsl
    Fog.vert.glsl
    Fog.frag.glsl
    Graticule.vert.glsl
    Graticule.frag.glsl
    LogDepthBuffer.vert.glsl
    LogDepthBuffer.VertOnly.vert.glsl    
    LogDepthBuffer.frag.glsl
    Shadowing.vert.glsl
    Shadowing.frag.glsl)

set(TARGET_IN
    Shaders.cpp.in)

configure_shaders(            
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )


SET(SOURCES_ROOT
	ActivityMonitorTool.cpp
    AnnotationEvents.cpp
    ArcGIS.cpp
    AtlasBuilder.cpp
    AutoClipPlaneHandler.cpp
    ClampCallback.cpp
    Controls.cpp
    ContourMap.cpp
    DataScanner.cpp
    EarthManipulator.cpp
	Ephemeris.cpp
    ExampleResources.cpp
    FeatureQueryTool.cpp
    Fog.cpp
    GeodeticGraticule.cpp
    GraticuleExtension.cpp
    GraticuleTerrainEffect.cpp
    GraticuleNode.cpp
    HTM.cpp
    LatLongFormatter.cpp
    LinearLineOfSight.cpp
	LogarithmicDepthBuffer.cpp
    LODBlending.cpp
    MeasureTool.cpp
    MGRSFormatter.cpp
    MGRSGraticule.cpp
    MouseCoordsTool.cpp
    ObjectLocator.cpp
    Ocean.cpp
    PolyhedralLineOfSight.cpp
    RadialLineOfSight.cpp
    RTTPicker.cpp
	Shadowing.cpp
	SimplexNoise.cpp
    SimplePager.cpp
    SpatialData.cpp
    Sky.cpp
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
    ${SHADERS_CPP}
)

SOURCE_GROUP( Sources FILES ${SOURCES_ROOT} )


SET(HEADERS_COLORFILTER
    BrightnessContrastColorFilter
    CMYKColorFilter
    GammaColorFilter
    GLSLColorFilter
    HSLColorFilter
    RGBColorFilter
    ChromaKeyColorFilter
    NightColorFilter
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
    NightColorFilter.cpp
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
    ${TARGET_GLSL}
    ${TARGET_IN}
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
