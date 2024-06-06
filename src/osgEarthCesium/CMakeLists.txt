find_package(CesiumNative)

if (CESIUM_NATIVE_FOUND)

    message(STATUS "Found Cesium Native")

    set(LIB_NAME osgEarthCesium)

    set(OSGEARTH_HAVE_CESIUM_NODEKIT ON PARENT_SCOPE) # for BuildConfig

    if(OSGEARTH_BUILD_SHARED_LIBS)
        add_definitions(-DOSGEARTHCESIUM_LIBRARY)
    else()
        add_definitions(-DOSGEARTHCESIUM_LIBRARY_STATIC)
    endif()

    # Add definitions for linking against async++ and spdlog.  Found in the cesium-omniverse project.
    add_definitions(-DSPDLOG_COMPILED_LIB -DLIBASYNC_STATIC -DGLM_FORCE_XYZW_ONLY -DGLM_FORCE_EXPLICIT_CTOR -DGLM_FORCE_SIZE_T_LENGTH)

    set(TARGET_SRC
        AssetAccessor.cpp
        CesiumIon.cpp
        CesiumCreditsNode.cpp
        CesiumTilesetNode.cpp
        CesiumLayer.cpp
        Context.cpp
        PrepareRenderResources.cpp
        Settings.cpp
        TaskProcessor.cpp)
        
    set(TARGET_H
        CesiumTilesetNode
        CesiumCreditsNode
        CesiumLayer
        CesiumIon
        Context
        Export
        Settings
        AssetAccessor
        PrepareRenderResources
        TaskProcessor)

    # ImGui headers:
    if(OSGEARTH_HAVE_IMGUI)
        set(TARGET_H_IMGUI ImGui/CesiumIonGUI)    
    endif()

    set(CMAKE_CXX_STANDARD 17)

    add_osgearth_library(
        TARGET ${LIB_NAME}
        PUBLIC_HEADERS ${TARGET_H}
        IMGUI_HEADERS ${TARGET_H_IMGUI}
        SOURCES ${TARGET_SRC}
        SHADERS ${TARGET_GLSL}
        TEMPLATES ${TARGET_IN}
        FOLDER "NodeKits")

    # Include the Cesium Native headers BEFORE anything else to ensure that dependencies that might be duplicated like spdlog are included from the Cesium Native headers.
    include_directories(BEFORE ${CESIUM_NATIVE_INCLUDE_DIR})

    target_link_libraries(${LIB_NAME} PRIVATE OE::CESIUM_NATIVE)

endif()
