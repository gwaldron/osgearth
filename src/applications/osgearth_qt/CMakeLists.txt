INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${QT_INCLUDES} )

SET(MOC_HDRS
    DemoMainWindow
)

# Qt resource files
set(LIB_QT_RCS
    images.qrc
)

IF(Qt5Widgets_FOUND)
    QT5_ADD_RESOURCES( LIB_RC_SRCS ${LIB_QT_RCS} )
    IF(Qt5Widgets_VERSION VERSION_LESS 5.2.0)
        QT5_WRAP_CPP( MOC_SRCS ${MOC_HDRS} OPTIONS "-f" )
    ELSE()
        QT5_WRAP_CPP( MOC_SRCS ${MOC_HDRS} )
    ENDIF()
ELSE()
    INCLUDE( ${QT_USE_FILE} )
    QT4_ADD_RESOURCES( LIB_RC_SRCS ${LIB_QT_RCS} )
    QT4_WRAP_CPP( MOC_SRCS ${MOC_HDRS} OPTIONS "-f" )
ENDIF()

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
