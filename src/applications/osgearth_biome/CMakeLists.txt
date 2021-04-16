IF(GLEW_FOUND)    
    include_directories(
        ${GLEW_INCLUDE_DIR}
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/examples
    )

    include_directories(${OSG_INCLUDE_DIRS} )
    
    set(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY GLEW_LIBRARIES)
    
    set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthProcedural)

    set(TARGET_SRC
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui_demo.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui_draw.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui_widgets.cpp
        ${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/examples/imgui_impl_opengl3.cpp
        ${OSGEARTH_SOURCE_DIR}/src/osgEarth/ImGui/OsgImGuiHandler.cpp
        osgearth_biome.cpp )

    SETUP_APPLICATION(osgearth_biome)
    
ENDIF(GLEW_FOUND)