IF(GLEW_FOUND)    
    INCLUDE_DIRECTORIES(
        ${GLEW_INCLUDE_DIR}
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/examples
    )

    INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )
    SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY GLEW_LIBRARIES)

    SET(TARGET_SRC
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui_demo.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui_draw.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui_widgets.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/examples/imgui_impl_opengl3.cpp
        ${OSGEARTH_SOURCE_DIR}/src/osgEarth/OsgImGuiHandler.cpp
    osgearth_imgui.cpp )

    #### end var setup  ###
    SETUP_APPLICATION(osgearth_imgui)
ENDIF(GLEW_FOUND)