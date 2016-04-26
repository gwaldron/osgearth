IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHANNO_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

set(LIB_NAME osgEarthAnnotation)

set(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

set(LIB_PUBLIC_HEADERS  
    AnnotationSettings
    AnnotationEditing
    AnnotationExtension
    AnnotationData
    AnnotationNode
    AnnotationRegistry
    AnnotationUtils
	BboxDrawable
    CircleNode
    Common
    Decoration
    Draggers
    EllipseNode
    Export
    FeatureNode
    FeatureEditing
    GeoPositionNode
    GeoPositionNodeAutoScaler
    LocalGeometryNode
    HighlightDecoration
    ImageOverlay
    ImageOverlayEditor
    LabelNode
    ModelNode
    PlaceNode
    RectangleNode
    ScaleDecoration
    TrackNode
)

set(LIB_COMMON_FILES
    AnnotationEditing.cpp
    AnnotationExtension.cpp
    AnnotationSettings.cpp
    AnnotationData.cpp
    AnnotationNode.cpp
    AnnotationRegistry.cpp
    AnnotationUtils.cpp
	BboxDrawable.cpp
    CircleNode.cpp
    Decoration.cpp
    Draggers.cpp
    EllipseNode.cpp
    FeatureNode.cpp
    FeatureEditing.cpp
    GeoPositionNode.cpp
    GeoPositionNodeAutoScaler.cpp
    LocalGeometryNode.cpp
    HighlightDecoration.cpp
    ImageOverlay.cpp
    ImageOverlayEditor.cpp
    LabelNode.cpp
    RectangleNode.cpp
    ModelNode.cpp
    PlaceNode.cpp
    TrackNode.cpp
)


ADD_LIBRARY( ${LIB_NAME} ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}  
    ${LIB_COMMON_FILES}
)



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
)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGMANIPULATOR_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OPENTHREADS_LIBRARY GDAL_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
