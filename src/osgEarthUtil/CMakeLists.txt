IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHUTIL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthUtil)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(LIB_PUBLIC_HEADERS  
    AnnotationEvents
    AutoClipPlaneHandler
    Common
    Controls
    ClampCallback
    EarthManipulator
    ElevationManager
    Export
    Formatters
    GeodeticGraticule
	LineOfSight
	MeasureTool
	MGRSGraticule
	MouseCoordsTool
	NearFarGroup	
    ObjectPlacer
    ObjectLocator
    #OceanSurfaceNode
    Pickers
    SkyNode
    SpatialData
    StarData
    TMS
    TMSPackager
    UTMGraticule
	WFS
    WMS
)

SET(LIB_COMMON_FILES
    AnnotationEvents.cpp
    AutoClipPlaneHandler.cpp
    ClampCallback.cpp
    Controls.cpp
    EarthManipulator.cpp	
    ElevationManager.cpp
    Formatters.cpp
    GeodeticGraticule.cpp
	LineOfSight.cpp
	MeasureTool.cpp
	MGRSGraticule.cpp
	MouseCoordsTool.cpp
	NearFarGroup.cpp
    ObjectPlacer.cpp
    ObjectLocator.cpp
    #OceanSurfaceNode.cpp
    Pickers.cpp
    SpatialData.cpp
    SkyNode.cpp
    TMS.cpp
    TMSPackager.cpp
    UTMGraticule.cpp
	WFS.cpp
    WMS.cpp
)

ADD_LIBRARY(${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}  
	${LIB_COMMON_FILES}
)

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

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGMANIPULATOR_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OPENTHREADS_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
