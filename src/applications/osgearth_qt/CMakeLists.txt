INCLUDE( ${QT_USE_FILE} )

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${QT_INCLUDES})

SET(MOC_HDRS
    DemoMainWindow
)

# Qt resource files
set(LIB_QT_RCS
    images.qrc
)

QT4_ADD_RESOURCES( LIB_RC_SRCS ${LIB_QT_RCS} )

QT4_WRAP_CPP( MOC_SRCS ${MOC_HDRS} OPTIONS "-f" )

SET(TARGET_H
    DemoMainWindow
    ${LIB_QT_RCS}
)

SET(TARGET_SRC
    ${MOC_SRCS}
    ${LIB_RC_SRCS}
    osgearth_qt.cpp
)

SET(TARGET_ADDED_LIBRARIES
    osgEarthQt
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
)

#### end var setup  ###
SETUP_APPLICATION(osgearth_qt)
