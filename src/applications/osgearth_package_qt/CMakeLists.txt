INCLUDE( ${QT_USE_FILE} )

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

SET(UI_FILES
    ExportDialog.ui
)

SET(MOC_HDRS
    PackageQtMainWindow
    ExportDialog
    WaitDialog
)

set(LIB_QT_RCS
    images.qrc
)

QT4_ADD_RESOURCES( LIB_RC_SRCS ${LIB_QT_RCS} )

QT4_WRAP_UI( UI_HDRS ${UI_FILES} )

QT4_WRAP_CPP( UI_SRCS ${UI_HDRS} )

QT4_WRAP_CPP( MOC_SRCS ${MOC_HDRS} OPTIONS "-f" )

SET(TARGET_H
    PackageQtMainWindow
    ExportDialog
    WaitDialog
    SceneController.h
    TMSExporter.h
    ${UI_HDRS}
    ${LIB_QT_RCS}
)

SET(TARGET_SRC
    ${UI_SRCS}
    ${MOC_SRCS}
    ${LIB_RC_SRCS}
    ExportDialog.cpp
    SceneController.cpp
    TMSExporter.cpp
    WaitDialog.cpp
    package_qt.cpp
)

SET(TARGET_ADDED_LIBRARIES
    osgEarthQt
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
)

#### end var setup  ###
SETUP_APPLICATION(osgearth_package_qt)
