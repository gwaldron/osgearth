FIND_PACKAGE(glew)
IF(GLEW_FOUND)
    INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${GLEW_INCLUDE_DIR})
    SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY GLEW_LIBRARY)

    SET(TARGET_SRC
    osgearth_monitor.cpp
    imgui.cpp
    imgui_draw.cpp
    imgui_demo.cpp
    imgui_widgets.cpp
    imgui_impl_opengl3.cpp
    OsgImGuiHandler.cpp
    )

    #### end var setup  ###
    SETUP_APPLICATION(osgearth_monitor)
ENDIF(GLEW_FOUND)