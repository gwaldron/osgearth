IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthQt)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

INCLUDE( ${QT_USE_FILE} )

# Header files that need moc'd
set(LIB_MOC_HDRS
    AnnotationDialogs
    AnnotationListWidget
    AnnotationToolbar
    CollapsiblePairWidget
    DataManager
    LayerManagerWidget
    LOSControlWidget
    LOSCreationDialog
    MapCatalogWidget
    TerrainProfileGraph
    TerrainProfileWidget
    ViewerWidget
)

# Qt resource files
set(LIB_QT_RCS
    images.qrc
)

# Qt UI files
set(LIB_QT_UIS
    ui/LOSCreationDialog.ui
)

QT4_ADD_RESOURCES( LIB_RC_SRCS ${LIB_QT_RCS} )

QT4_WRAP_UI( LIB_UI_HDRS ${LIB_QT_UIS} )

QT4_WRAP_CPP( LIB_UI_SRCS ${LIB_UI_HDRS} OPTIONS "-f" )

QT4_WRAP_CPP( LIB_MOC_SRCS ${LIB_MOC_HDRS} OPTIONS "-f" )


SET(LIB_PUBLIC_HEADERS
#   header files go here
    Actions
    AnnotationDialogs
    AnnotationListWidget
    AnnotationToolbar
    CollapsiblePairWidget
    Common
    DataManager
    GuiActions
    LayerManagerWidget
    LOSControlWidget
    LOSCreationDialog
    MapCatalogWidget
    TerrainProfileGraph
    TerrainProfileWidget
    ViewWidget
    ViewerWidget
    ${LIB_UI_HDRS}
    ${LIB_QT_UIS}
    ${LIB_QT_RCS}
)

ADD_LIBRARY(${LIB_NAME} SHARED
    ${LIB_PUBLIC_HEADERS}
#  .cpp files go here
    ${LIB_RC_SRCS}
    ${LIB_UI_SRCS}
    ${LIB_MOC_SRCS}
    AnnotationDialogs.cpp
    AnnotationListWidget.cpp
    AnnotationToolbar.cpp
    CollapsiblePairWidget.cpp
    DataManager.cpp
    LayerManagerWidget.cpp
    LOSControlWidget.cpp
    LOSCreationDialog.cpp
    MapCatalogWidget.cpp
    TerrainProfileGraph.cpp
    TerrainProfileWidget.cpp
    ViewWidget.cpp
    ViewerWidget.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

LINK_INTERNAL(${LIB_NAME}
    osgEarth
    osgEarthAnnotation
    osgEarthUtil
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGWIDGET_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGQT_LIBRARY OPENTHREADS_LIBRARY)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

INCLUDE(ModuleInstall OPTIONAL)
