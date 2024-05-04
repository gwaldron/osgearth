cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

# VCPKG customization ....................................................

if (${CMAKE_TOOLCHAIN_FILE} MATCHES ".*vcpkg.cmake.*")
    message(STATUS "Building with vcpkg toolchain.")
    set(USING_VCPKG ON)
    set(VCPKG_OVERLAY_TRIPLETS ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/triplets)
endif()


# Project setup ..........................................................

project(
    OSGEARTH
    DESCRIPTION "osgEarth 3D Geospatial SDK"
    HOMEPAGE_URL "https://github.com/gwaldron/osgearth"
    LANGUAGES CXX)

# SDK version number
set(OSGEARTH_MAJOR_VERSION 3)
set(OSGEARTH_MINOR_VERSION 5)
set(OSGEARTH_PATCH_VERSION 0)
set(OSGEARTH_VERSION ${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION})

# Increment this each time the ABI changes
set(OSGEARTH_SOVERSION 152)

# Require C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# We have some custom .cmake scripts not in the official distribution.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")

# Special folder for build-time generated include files
set(OSGEARTH_BUILDTIME_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/build_include")
include_directories(${OSGEARTH_BUILDTIME_INCLUDE_DIR})

# Third-party sources included in the reposotory
set(OSGEARTH_EMBEDDED_THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/src/third_party)
    
# Platform-specific settings ............................................

include(oe_ios)
include(oe_osx)
include(oe_unix)
include(oe_win32)

# Build options ..........................................................

option(OSGEARTH_BUILD_TOOLS "Build the osgEarth command-line tools" ON)
option(OSGEARTH_BUILD_EXAMPLES "Build the osgEarth example applications" ON)
option(OSGEARTH_BUILD_CESIUM_NODEKIT "Build the Cesium nodekit (osgEarthCesium)" OFF)
option(OSGEARTH_BUILD_TRITON_NODEKIT "Build support for SunDog Triton SDK" OFF)
option(OSGEARTH_BUILD_SILVERLINING_NODEKIT "Build support for SunDog SilverLining SDK" OFF)
option(OSGEARTH_ENABLE_GEOCODER "Enable the geocoder (GDAL/OGR must be built with geocoder support)" OFF)

option(OSGEARTH_BUILD_DOCS "Include the documentation folder" ON)
mark_as_advanced(OSGEARTH_BUILD_DOCS)

option(OSGEARTH_BUILD_TESTS "Build the osgEarth unit tests" OFF)
mark_as_advanced(OSGEARTH_BUILD_TESTS)

option(OSGEARTH_BUILD_PROCEDURAL_NODEKIT "Build the procedural terrain nodekit (osgEarthProcedural)" OFF)
mark_as_advanced(OSGEARTH_BUILD_PROCEDURAL_NODEKIT)

option(OSGEARTH_BUILD_LEGACY_SPLAT_NODEKIT "Build the legacy procedural nodekit (osgEarthSplat)" OFF)
mark_as_advanced(OSGEARTH_BUILD_LEGACY_SPLAT_NODEKIT)

option(OSGEARTH_BUILD_LEGACY_CONTROLS_API "Build the legacy Controls UI API" OFF)
mark_as_advanced(OSGEARTH_BUILD_LEGACY_CONTROLS_API)

option(OSGEARTH_BUILD_LEVELDB_CACHE "Build the LevelDB cache plugin" OFF)
mark_as_advanced(OSGEARTH_BUILD_LEVELDB_CACHE)

option(OSGEARTH_BUILD_ZIP_PLUGIN "Build osgEarth's zip plugin based on libzip" ON)
mark_as_advanced(OSGEARTH_BUILD_ZIP_PLUGIN)

option(OSGEARTH_ENABLE_PROFILING "Enable profiling with Tracy" OFF)
mark_as_advanced(OSGEARTH_ENABLE_PROFILING)

option(OSGEARTH_ASSUME_SINGLE_GL_CONTEXT "Assume the use of a single GL context for all GL objects (advanced)" OFF)
mark_as_advanced(OSGEARTH_ASSUME_SINGLE_GL_CONTEXT)

option(OSGEARTH_ASSUME_SINGLE_THREADED_OSG "Assume OSG will always be configured to run in SingleThreaded mode (advanced)" OFF)
mark_as_advanced(OSGEARTH_ASSUME_SINGLE_THREADED_OSG)

option(OSGEARTH_INSTALL_SHADERS "Whether to deploy GLSL shaders when installing (OFF=inlined shaders)" OFF)
mark_as_advanced(OSGEARTH_INSTALL_SHADERS)


# Dependencies ...........................................................

# Update git submodules
# https://cliutils.gitlab.io/modern-cmake/chapters/projects/submodule.html
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    message(STATUS "Submodule update")
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
        message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
endif()

# required
find_package(OpenGL REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgGA osgManipulator osgShadow osgSim osgText osgUtil osgViewer OpenThreads)
find_package(CURL REQUIRED)
find_package(GDAL REQUIRED)
find_package(SQLite3 REQUIRED)

# optional
find_package(geos)
find_package(GLEW)
find_package(Protobuf)
find_package(WebP)
find_package(blend2d)
find_package(blosc)
find_package(spdlog)
find_package(meshoptimizer)
#find_package(BASISU)



# Configure OSGEARTH_HAVE_XXX variables for the BuildConfig ..........

if(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE)
  SET(OSGEARTH_HAVE_MVT ON)
endif()

if(SQLite3_FOUND)
    set(OSGEARTH_HAVE_SQLITE3 ON)
    set(OSGEARTH_HAVE_MBTILES ON)
endif()

if(Protobuf_FOUND)
    set(OSGEARTH_HAVE_PROTOBUF ON)
endif()

if(geos_FOUND)
    set(OSGEARTH_HAVE_GEOS ON)
endif()

if (meshoptimizer_FOUND)
    set(OSGEARTH_HAVE_MESH_OPTIMIZER ON)
endif()

if(OSGEARTH_BUILD_CESIUM_NODEKIT)
    find_package(CesiumNative)
endif()

if(OSGEARTH_BUILD_TRITON_NODEKIT)
    find_package(Triton)
endif()

if(OSGEARTH_BUILD_SILVERLINING_NODEKIT)
    find_package(SilverLining)
endif()

if(OSGEARTH_ENABLE_PROFILING)
    find_package(Tracy)
endif()

if(OSGEARTH_BUILD_ZIP_PLUGIN)
    find_package(LibZip)
endif()

set(WITH_EXTERNAL_DUKTAPE FALSE CACHE BOOL "Use external Duktape instead of inlining it")
if(WITH_EXTERNAL_DUKTAPE)
    find_package(Duktape)
endif()

set(WITH_EXTERNAL_TINYXML FALSE CACHE BOOL "Use external TinyXML instead of inlining it")
if(WITH_EXTERNAL_TINYXML)
    find_package(TinyXML)
endif()


# Logging SDK
if(spdlog_FOUND)
    set(OSGEARTH_HAVE_SPDLOG ON)
endif()

# Tracy is a profiler
if(TRACY_FOUND AND OSGEARTH_ENABLE_PROFILING)
    ADD_DEFINITIONS(-DOSGEARTH_PROFILING)
endif()

if(GLEW_FOUND AND EXISTS "${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/imgui.cpp")
    set(OSGEARTH_ENABLE_IMGUI 1)
    add_definitions(-DOSGEARTH_ENABLE_IMGUI)
endif()

if(OSGEARTH_ENABLE_GEOCODER)
    set(OSGEARTH_HAVE_GEOCODER ON)
endif()

if(blend2d_FOUND)
    set(OSGEARTH_HAVE_BLEND2D ON)
endif()

if(OSGEARTH_ASSUME_SINGLE_GL_CONTEXT)
    add_definitions(-DOSGEARTH_SINGLE_GL_CONTEXT)
endif()

if(OSGEARTH_ASSUME_SINGLE_THREADED_OSG)
    add_definitions(-DOSGEARTH_SINGLE_THREADED_OSG)
endif()

# support for the old Controls API (superceded by imgui)
if(OSGEARTH_BUILD_LEGACY_CONTROLS_API)
    add_definitions(-DOSGEARTH_HAVE_CONTROLS_API)
endif()

# Protobuf enables the MVT (Mapnik Vector Tiles) format
set(PROTOBUF_USE_DLLS FALSE CACHE BOOL "Set this to true if Protobuf is compiled as dll")

# Bring in out utility macros
include(OsgEarthMacroUtils)

# Detect the OSG version so we can append it to plugin DLLs just like OSG does.
detect_osg_version()

if(NOT OPENSCENEGRAPH_VERSION)
	set(OPENSCENEGRAPH_VERSION ${OPENSCENEGRAPH_MAJOR_VERSION}.${OPENSCENEGRAPH_MINOR_VERSION}.${OPENSCENEGRAPH_PATCH_VERSION})
endif()

if(OPENSCENEGRAPH_VERSION)
    message(STATUS "Found OSG version ${OPENSCENEGRAPH_VERSION}")
endif()

# Make the headers visible to everything
include_directories(
    ${OSGEARTH_SOURCE_DIR}/src
    ${OSG_INCLUDE_DIR}
    ${OSG_GEN_INCLUDE_DIR}
)


# Dynamic vs Static Linking
option(OSGEARTH_BUILD_SHARED_LIBS "ON to build shared libraries; OFF to build static libraries." ON)
if(OSGEARTH_BUILD_SHARED_LIBS)
    set(OSGEARTH_DYNAMIC_OR_STATIC "SHARED")
else()
    set(OSGEARTH_DYNAMIC_OR_STATIC "STATIC")
endif()


# Source code ............................................................

add_subdirectory(src)

if(OSGEARTH_BUILD_DOCS)
    add_subdirectory(docs)
endif()


# CMake Config File Creation .............................................

include(cmake/install-package-config-files.cmake)

osgearth_install_package_config_files(
    osgEarth
    ${OSGEARTH_VERSION}
    ${CMAKE_INSTALL_PREFIX}/include
    ${CMAKE_INSTALL_PREFIX}/lib)
    
    
    
# IDE configuration ......................................................

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake Targets")
    

#message(STATUS "....................................")
#get_cmake_property(_variableNames VARIABLES)
#list (SORT _variableNames)
#foreach (_variableName ${_variableNames})
    #message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
#message(STATUS "....................................")

