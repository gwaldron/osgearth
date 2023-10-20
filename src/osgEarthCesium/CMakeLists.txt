SET(LIB_NAME osgEarthCesium)

IF(OSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DOSGEARTHCESIUM_LIBRARY)
ELSE ()
    ADD_DEFINITIONS(-DOSGEARTHCESIUM_LIBRARY_STATIC)
ENDIF()

# Add definitions for linking against async++ and spdlog.  Found in the cesium-omniverse project.
ADD_DEFINITIONS(-DSPDLOG_COMPILED_LIB -DLIBASYNC_STATIC)

set(TARGET_SRC
    AssetAccessor.cpp
    CesiumIon.cpp
    CesiumCreditsNode.cpp
    CesiumTilesetNode.cpp
    CesiumLayer.cpp
    Context.cpp
    PrepareRenderResources.cpp
    Settings.cpp
    TaskProcessor.cpp    
)
	
set(LIB_PUBLIC_HEADERS
    CesiumTilesetNode
    CesiumCreditsNode
    CesiumLayer
    CesiumIon
    Context
	Export
    Settings
    )
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
    AssetAccessor
    PrepareRenderResources
    TaskProcessor    
	)

# ImGui headers:
if(OSGEARTH_ENABLE_IMGUI)
    set(LIB_PUBLIC_HEADERS_IMGUI
        ImGui/CesiumIonGUI        
        )    
endif()

set(CMAKE_CXX_STANDARD 17)

ADD_LIBRARY(
    ${LIB_NAME}
    ${OSGEARTH_DYNAMIC_OR_STATIC}
    ${TARGET_H}
    ${LIB_PUBLIC_HEADERS_IMGUI}
    ${TARGET_SRC}
    ${TARGET_GLSL}
    ${TARGET_IN}
)

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

LINK_INTERNAL(${LIB_NAME}
    osgEarth
)

IF (TRACY_FOUND)
    INCLUDE_DIRECTORIES(${TRACY_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} TRACY_LIBRARY)
ENDIF(TRACY_FOUND)

# Include the Cesium Native headers BEFORE anything else to ensure that dependencies that might be duplicated like spdlog are included from the Cesium Native headers.
INCLUDE_DIRECTORIES(BEFORE ${CESIUM_NATIVE_INCLUDE_DIR})

LINK_WITH_VARIABLES(${LIB_NAME}
                    OSG_LIBRARY
                    OSGUTIL_LIBRARY
                    OSGDB_LIBRARY
                    OSGTEXT_LIBRARY
                    CESIUM_NATIVE_3DTILES_LIBRARY                    
                    CESIUM_NATIVE_3DTILES_SELECTION_LIBRARY
                    CESIUM_NATIVE_UTILITY_LIBRARY
                    CESIUM_NATIVE_GLTF_READER_LIBRARY
                    CESIUM_NATIVE_ION_CLIENT_LIBRARY
                    CESIUM_NATIVE_GEOSPATIAL_LIBRARY
                    CESIUM_NATIVE_GEOMETRY_LIBRARY
                    CESIUM_NATIVE_GLTF_LIBRARY
                    CESIUM_NATIVE_KTX_READ_LIBRARY
                    CESIUM_NATIVE_DRACO_LIBRARY
                    CESIUM_NATIVE_SPDLOG_LIBRARY
                    CESIUM_NATIVE_ASYNC++_LIBRARY
                    CESIUM_NATIVE_WEBPDECODER_LIBRARY
                    CESIUM_NATIVE_URIPARSER_LIBRARY
                    CESIUM_NATIVE_TURBOJPEG_LIBRARY
                    CESIUM_NATIVE_CSPRNG_LIBRARY
                    CESIUM_NATIVE_MODPB64_LIBRARY
                    CESIUM_NATIVE_JSONREADER_LIBRARY
                    CESIUM_NATIVE_ASYNC_LIBRARY
                    CESIUM_NATIVE_S2GEOMETRY_LIBRARY
                    CESIUM_NATIVE_TINYXML2_LIBRARY
                    )
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

# Add this project to the NodeKits solution folder
set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

INCLUDE(ModuleInstall OPTIONAL)