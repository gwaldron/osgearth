#
# FindCesiumNative.cmake
#
# Inputs:
#   CESIUM_NATIVE_DIR : folder containing include/ and lib/ for Cesium Native
#       (or "CESIUM_NATIVE_DIR" environment variable)
#
# Outputs:
#   CESIUM_NATIVE_FOUND (boolean variable)
#   OE::CESIUM_NATIVE (imported link library)
#
# Usage:
#   find_package(CesiumNative)
#   ...
#   target_link_libraries(TARGET my_target PRIVATE OE::CESIUM_NATIVE)
#
set(CESIUM_NATIVE_DIR "" CACHE PATH "Root directory of cesium-native distribution")

unset(CESIUM_NATIVE_FOUND)

# Location the cesium-native installation:
find_path(CESIUM_NATIVE_INCLUDE_DIR CesiumUtility/Uri.h
    PATHS
        ${CESIUM_NATIVE_DIR}
        $ENV{CESIUM_NATIVE_DIR}
    PATH_SUFFIXES
        include )
        
set(CESIUM_ANY_LIBRARY_MISSING FALSE)
        
# Macro to locate each cesium library.
macro(find_cesium_library MY_LIBRARY_VAR MY_LIBRARY_NAME)

    # start by clearing any existing value so we pick the dependency
    # that built with the cesium-native package
    unset(${MY_LIBRARY_VAR}_LIBRARY_DEBUG CACHE)
    unset(${MY_LIBRARY_VAR}_LIBRARY_RELEASE CACHE)

    if (NOT CESIUM_ANY_LIBRARY_MISSING)
        find_library(${MY_LIBRARY_VAR}_LIBRARY_DEBUG
            NAMES
                ${MY_LIBRARY_NAME}d
            PATHS
                ${CESIUM_NATIVE_DIR}/lib
                $ENV{CESIUM_NATIVE_DIR}
                PATH_SUFFIXES lib64 lib
            NO_DEFAULT_PATH )

        find_library(${MY_LIBRARY_VAR}_LIBRARY_RELEASE
             NAMES
                 ${MY_LIBRARY_NAME}
             PATHS
                 ${CESIUM_NATIVE_DIR}/lib
                 $ENV{CESIUM_NATIVE_DIR}
                 PATH_SUFFIXES lib64 lib
             NO_DEFAULT_PATH )

        set(MY_DEBUG_LIBRARY "${${MY_LIBRARY_VAR}_LIBRARY_DEBUG}")
        set(MY_RELEASE_LIBRARY "${${MY_LIBRARY_VAR}_LIBRARY_RELEASE}")       
        
        if(MY_DEBUG_LIBRARY OR MY_RELEASE_LIBRARY)
            # name of the import for this component:
            set(MY_IMPORT_LIBRARY_NAME "CesiumNative::${MY_LIBRARY_NAME}")

            # create the import library for this component:
            add_library(${MY_IMPORT_LIBRARY_NAME} UNKNOWN IMPORTED)
            
            # Normally you would need to add the include folders to the import library.
            # But because cesium native's includes need to come FIRST, we will omit
            # it and specify it later with a BEFORE argument to include_directories.
            #set_target_properties(${MY_IMPORT_LIBRARY_NAME} PROPERTIES
            #    INTERFACE_INCLUDE_DIRECTORIES "${CESIUM_NATIVE_INCLUDE_DIR}")
            
            set_target_properties(${MY_IMPORT_LIBRARY_NAME} PROPERTIES
                IMPORTED_LOCATION ${MY_RELEASE_LIBRARY}
                IMPORTED_LOCATION_DEBUG ${MY_DEBUG_LIBRARY} )

            # finally, add it to the main list for later.
            list(APPEND CESIUM_NATIVE_IMPORT_LIBRARIES "${MY_IMPORT_LIBRARY_NAME}")
        else()
            message(WARNING "Cesium Native: Could NOT find ${MY_LIBRARY_NAME}")
            set(CESIUM_ANY_LIBRARY_MISSING TRUE)
        endif()
    endif()
endmacro()

# Note, it's not strictly necessary to set all these cache variables,
# but it's nice to be able to see them in cmake-gui. -gw
find_cesium_library(CESIUM_NATIVE_3DTILES_SELECTION Cesium3DTilesSelection)
find_cesium_library(CESIUM_NATIVE_ION_CLIENT CesiumIonClient)
find_cesium_library(CESIUM_NATIVE_RASTER_OVERLAYS CesiumRasterOverlays)
find_cesium_library(CESIUM_NATIVE_3DTILES_CONTENT Cesium3DTilesContent)
find_cesium_library(CESIUM_NATIVE_3DTILES_READER Cesium3DTilesReader)
find_cesium_library(CESIUM_NATIVE_3DTILES Cesium3DTiles)
find_cesium_library(CESIUM_NATIVE_QUANTIZED_MESH_TERRAIN CesiumQuantizedMeshTerrain)
find_cesium_library(CESIUM_NATIVE_GLTF_CONTENT CesiumGltfContent)
find_cesium_library(CESIUM_NATIVE_GLTF_READER CesiumGltfReader)
find_cesium_library(CESIUM_NATIVE_GLTF CesiumGltf)
find_cesium_library(CESIUM_NATIVE_GEOSPATIAL CesiumGeospatial)
find_cesium_library(CESIUM_NATIVE_GEOMETRY CesiumGeometry)
find_cesium_library(CESIUM_NATIVE_ASYNC CesiumAsync)
find_cesium_library(CESIUM_NATIVE_JSONREADER CesiumJsonReader)
find_cesium_library(CESIUM_NATIVE_UTILITY CesiumUtility) 

find_cesium_library(CESIUM_NATIVE_ASYNC++ async++)
find_cesium_library(CESIUM_NATIVE_CSPRNG csprng)
find_cesium_library(CESIUM_NATIVE_DRACO draco)
find_cesium_library(CESIUM_NATIVE_KTX ktx)
find_cesium_library(CESIUM_NATIVE_MODPB64 modp_b64)
find_cesium_library(CESIUM_NATIVE_S2GEOMETRY s2geometry)
find_cesium_library(CESIUM_NATIVE_SPDLOG spdlog)
find_cesium_library(CESIUM_NATIVE_TINYXML2 tinyxml2)
find_cesium_library(CESIUM_NATIVE_TURBOJPEG turbojpeg)
find_cesium_library(CESIUM_NATIVE_URIPARSER uriparser)
find_cesium_library(CESIUM_NATIVE_WEBPDECODER webpdecoder)
find_cesium_library(CESIUM_NATIVE_MESHOPTIMIZER meshoptimizer)



if(NOT CESIUM_ANY_LIBRARY_MISSING)
    set(CESIUM_NATIVE_FOUND TRUE)

    # Assemble all the component libraries into one single import library:
    # https://stackoverflow.com/a/48397346/4218920
    
    add_library(OE::CESIUM_NATIVE INTERFACE IMPORTED)
    
    set_property(TARGET OE::CESIUM_NATIVE PROPERTY
        INTERFACE_LINK_LIBRARIES ${CESIUM_NATIVE_IMPORT_LIBRARIES} )
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CesiumNative DEFAULT_MSG CESIUM_NATIVE_FOUND)

