IF (Qt5Widgets_FOUND OR QT4_FOUND AND NOT ANDROID AND OSGEARTH_USE_QT)


IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHQT_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthQt)

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

if(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)
    # Header files that need moc'd
    set(LIB_MOC_HDRS
        #AnnotationDialogs
        #AnnotationListWidget
        #AnnotationToolbar
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
else(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)
    # Header files that need moc'd
    set(LIB_MOC_HDRS
        ViewerWidget
    )
endif(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)

IF(Qt5Widgets_FOUND)
    QT5_ADD_RESOURCES( LIB_RC_SRCS ${LIB_QT_RCS} )
    QT5_WRAP_UI( LIB_UI_HDRS ${LIB_QT_UIS} )
    QT5_WRAP_CPP( LIB_UI_SRCS ${LIB_UI_HDRS} )
    SET(LIB_MOC_SRCS)
    FOREACH( LIB_MOC_HDR ${LIB_MOC_HDRS} )
        GET_FILENAME_COMPONENT( LIB_MOC_HDR_ABS ${LIB_MOC_HDR} ABSOLUTE )
        QT5_WRAP_CPP( LIB_MOC_SRC ${LIB_MOC_HDR} OPTIONS "-f${LIB_MOC_HDR_ABS}" )
        LIST( APPEND LIB_MOC_SRCS ${LIB_MOC_SRC} )
    ENDFOREACH()
ELSE()
    INCLUDE( ${QT_USE_FILE} )
    QT4_ADD_RESOURCES( LIB_RC_SRCS ${LIB_QT_RCS} )
    QT4_WRAP_UI( LIB_UI_HDRS ${LIB_QT_UIS} )
    QT4_WRAP_CPP( LIB_UI_SRCS ${LIB_UI_HDRS} OPTIONS "-f" )
    QT4_WRAP_CPP( LIB_MOC_SRCS ${LIB_MOC_HDRS} OPTIONS "-f" )
ENDIF()

if(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)
    SET(LIB_PUBLIC_HEADERS
    #   header files go here
        Actions
        #AnnotationDialogs
        #AnnotationListWidget
        #AnnotationToolbar
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
        #AnnotationDialogs.cpp
        #AnnotationListWidget.cpp
        #AnnotationToolbar.cpp
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
else(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)
    SET(LIB_PUBLIC_HEADERS
    #   header files go here
        Common
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
        ViewWidget.cpp
        ViewerWidget.cpp
    )
endif(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${OSGEARTH_SOURCE_DIR} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

if(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)
    link_internal(${LIB_NAME}
        osgEarth
        osgEarthAnnotation
        osgEarthUtil
        ${QT_QTCORE_LIBRARY}
        ${QT_QTGUI_LIBRARY}
        ${QT_QTOPENGL_LIBRARY}
    )
else(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)
    link_internal(${LIB_NAME}
        osgEarth
        osgEarthUtil
        ${QT_QTCORE_LIBRARY}
        ${QT_QTGUI_LIBRARY}
        ${QT_QTOPENGL_LIBRARY}
    )
endif(OSGEARTH_QT_BUILD_LEGACY_WIDGETS)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGWIDGET_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGDB_LIBRARY OSGFX_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGQT_LIBRARY OPENTHREADS_LIBRARY)

LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

IF ( Qt5Widgets_FOUND )
    qt5_use_modules( ${LIB_NAME} Gui Widgets OpenGL )
ENDIF( Qt5Widgets_FOUND )

# Add this project to the NodeKits solution folder
set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

INCLUDE(ModuleInstall OPTIONAL)

ENDIF (Qt5Widgets_FOUND OR QT4_FOUND AND NOT ANDROID AND OSGEARTH_USE_QT)