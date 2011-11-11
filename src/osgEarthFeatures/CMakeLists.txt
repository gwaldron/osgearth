IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHFEATURES_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

IF (GEOS_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_GEOS)
ENDIF(GEOS_FOUND)

SET(LIB_NAME osgEarthFeatures)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})
SET(LIB_PUBLIC_HEADERS
    AltitudeFilter
    BufferFilter
    BuildGeometryFilter  
    BuildTextFilter
    BuildTextOperator
    CentroidFilter
    Common
    ConvertTypeFilter
    CropFilter
    ExtrudeGeometryFilter
    Feature
    FeatureCursor
    FeatureDisplayLayout
    FeatureGeometryIndex
    FeatureGridder
	FeatureListSource
    FeatureModelGraph
    FeatureModelSource
    FeatureNode
    FeatureSource
	FeatureSourceNode
	FeatureSourceMeshConsolidator
    FeatureTileSource
    Filter
    FilterContext
    GeometryCompiler
	GeometryUtils
    LabelSource
	OgrUtils
    OptimizerHints	
    ResampleFilter
    ScaleFilter
    Session
    ScatterFilter
    Script
    ScriptEngine
    SubstituteModelFilter
    TransformFilter
    VirtualFeatureSource
)

ADD_LIBRARY(${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
    AltitudeFilter.cpp
    BufferFilter.cpp
    BuildGeometryFilter.cpp 
    BuildTextFilter.cpp
    BuildTextOperator.cpp
    CentroidFilter.cpp
    ConvertTypeFilter.cpp
    CropFilter.cpp
    ExtrudeGeometryFilter.cpp
    Feature.cpp
    FeatureCursor.cpp
    FeatureDisplayLayout.cpp
    FeatureGeometryIndex.cpp
    FeatureGridder.cpp
	FeatureListSource.cpp
    FeatureModelGraph.cpp
    FeatureModelSource.cpp
    FeatureNode.cpp
    FeatureSource.cpp
	FeatureSourceNode.cpp
	FeatureSourceMeshConsolidator.cpp
    FeatureTileSource.cpp
    Filter.cpp
    FilterContext.cpp
    GeometryCompiler.cpp
	GeometryUtils.cpp
    LabelSource.cpp
    OptimizerHints.cpp
    ResampleFilter.cpp
    ScaleFilter.cpp
    Session.cpp
    ScatterFilter.cpp
    ScriptEngine.cpp
    SubstituteModelFilter.cpp
    TransformFilter.cpp
    VirtualFeatureSource.cpp
)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} )


IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

LINK_INTERNAL(${LIB_NAME}
    osgEarth
    osgEarthSymbology
)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OPENTHREADS_LIBRARY)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
