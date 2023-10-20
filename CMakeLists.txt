cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

# VCPKG customization ....................................................

if (${CMAKE_TOOLCHAIN_FILE} MATCHES ".*vcpkg.cmake.*")
    message(STATUS "Building with vcpkg toolchain.")
    set(USING_VCPKG ON)
    set(VCPKG_OVERLAY_TRIPLETS ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/triplets)
endif()


# Project setup ..........................................................

project(OSGEARTH)

# SDK version number (informational)
set(OSGEARTH_MAJOR_VERSION 3)
set(OSGEARTH_MINOR_VERSION 5)
set(OSGEARTH_PATCH_VERSION 0)
set(OSGEARTH_VERSION ${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION})

# Increment this each time the ABI changes
set(OSGEARTH_SOVERSION 149)

# Require C++11
set_property(GLOBAL PROPERTY CXX_STANDARD 11)
set_property(GLOBAL PROPERTY CXX_STANDARD_REQUIRED ON)

# Detect out-of-source build. You should always do an out-of-source build!
if (NOT "${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
    SET(OUT_OF_SOURCE_BUILD 1)
endif()


# IDE configuration ......................................................

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake Targets")


# Search Locations .......................................................

# Expose CMAKE_INCLUDE_PATH and CMAKE_LIBARY_PATH to the GUI so users
# may set these values without needing to manipulate the environment.
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} CACHE STRING "Add additional search paths here. Use ; to separate multiple paths.")
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} CACHE STRING "Add additional search paths here. Use ; to separate multiple paths.")
set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH}  CACHE STRING "Add additional search paths here. Use ; to separate multiple paths.")

set(OSGEARTH_DEPENDENCY_DIR "" CACHE PATH "Folder to search for dependencies (e.g., vcpkg/installed/x64-windows)")
if(OSGEARTH_DEPENDENCY_DIR)
    LIST(APPEND CMAKE_INCLUDE_PATH ${OSGEARTH_DEPENDENCY_DIR}/include)
    LIST(APPEND CMAKE_LIBRARY_PATH ${OSGEARTH_DEPENDENCY_DIR}/lib)
endif()

# We have some custom .cmake scripts not in the official distribution.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${OSGEARTH_SOURCE_DIR}/CMakeModules;${OSGEARTH_DEPENDENCY_DIR}")

# Special folder for build-time generated include files
set(OSGEARTH_BUILDTIME_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/build_include")
include_directories(${OSGEARTH_BUILDTIME_INCLUDE_DIR})

# Third-party sources included in the osgEarth repo
set(OSGEARTH_EMBEDDED_THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/src/third_party)

# Location of GLCORE headers if you are using GLCORE
set(OSGEARTH_GLCORE_INCLUDE_DIR CACHE PATH "For OpenGL CORE Profile, location of the GLCORE header files")
if (OSGEARTH_GLCORE_INCLUDE_DIR)
    include_directories(${OSGEARTH_GLCORE_INCLUDE_DIR})
endif()


# Platform-specific settings ............................................

include(oe_ios)
include(oe_osx)
include(oe_unix)
include(oe_win32)


# Build options ..........................................................

option(OSGEARTH_BUILD_TOOLS "Build the osgEarth command-line tools" ON)
option(OSGEARTH_BUILD_EXAMPLES "Build the osgEarth example applications" ON)
option(OSGEARTH_BUILD_TESTS "Build the osgEarth unit tests" OFF)
option(OSGEARTH_BUILD_PROCEDURAL_NODEKIT "Build the procedural terrain nodekit (osgEarthProcedural)" OFF)
option(OSGEARTH_BUILD_LEGACY_SPLAT_NODEKIT "Build the legacy procedural nodekit (osgEarthSplat)" OFF)
option(OSGEARTH_BUILD_CESIUM_NODEKIT "Build the Cesium nodekit (osgEarthCesium)" OFF)
option(OSGEARTH_BUILD_TRITON_NODEKIT "Build support for SunDog Triton SDK" OFF)
option(OSGEARTH_BUILD_SILVERLINING_NODEKIT "Build support for SunDog SilverLining SDK" OFF)
option(OSGEARTH_BUILD_LEVELDB_CACHE "Build the LevelDB cache plugin" OFF)
option(OSGEARTH_BUILD_ZIP_PLUGIN "Build osgEarth's zip plugin based on libzip" ON)
option(OSGEARTH_BUILD_DOCS "Include the documentation folder" ON)
option(OSGEARTH_ENABLE_PROFILING "Enable profiling with Tracy" OFF)
option(OSGEARTH_ENABLE_GEOCODER "Enable the geocoder (GDAL/OGR must be built with geocoder support)" OFF)
option(OSGEARTH_ASSUME_SINGLE_GL_CONTEXT "Assume the use of a single GL context for all GL objects (advanced)" OFF)
option(OSGEARTH_INSTALL_SHADERS "Whether to deploy GLSL shaders when installing (OFF=inlined shaders)" OFF)

mark_as_advanced(OSGEARTH_BUILD_TESTS)
mark_as_advanced(OSGEARTH_BUILD_LEVELDB_CACHE)
mark_as_advanced(OSGEARTH_BUILD_DOCS)
mark_as_advanced(OSGEARTH_ENABLE_PROFILING)
mark_as_advanced(OSGEARTH_ASSUME_SINGLE_GL_CONTEXT)
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
find_package(OSG REQUIRED)
find_package(CURL REQUIRED)
find_package(GDAL REQUIRED)

# optional
find_package(GEOS)
find_package(Sqlite3)
find_package(Draco)
find_package(BASISU)
find_package(GLEW)
find_package(Protobuf)
find_package(WEBP)
find_package(Blend2D)
find_package(Blosc)
find_package(spdlog)

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
endif(WITH_EXTERNAL_DUKTAPE)

set(WITH_EXTERNAL_TINYXML FALSE CACHE BOOL "Use external TinyXML instead of inlining it")
if(WITH_EXTERNAL_TINYXML)
    find_package(TinyXML)
endif(WITH_EXTERNAL_TINYXML)


# Logging SDK
if(spdlog_FOUND)
    add_definitions(-DHAVE_SPDLOG)
endif()

# Sqlite enables the MBTiles format:
if(SQLITE3_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_MBTILES)
endif(SQLITE3_FOUND)

# DRACO enables geometry compression sometimes used in glTF:
if(draco_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_DRACO)
endif(draco_FOUND)

if(Protobuf_FOUND)
    add_definitions(-DOSGEARTH_HAVE_PROTOBUF)
endif()

# Tracy is a profiler
if(TRACY_FOUND AND OSGEARTH_ENABLE_PROFILING)
    ADD_DEFINITIONS(-DOSGEARTH_PROFILING)
endif()

if(GLEW_FOUND AND EXISTS "${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui.cpp")
    set(OSGEARTH_ENABLE_IMGUI 1)
    add_definitions(-DOSGEARTH_ENABLE_IMGUI)
endif()

if(OSGEARTH_ENABLE_GEOCODER)
    add_definitions(-DOSGEARTH_HAVE_GEOCODER)
endif()

if(OSGEARTH_ASSUME_SINGLE_GL_CONTEXT)
    add_definitions(-DOSGEARTH_SINGLE_GL_CONTEXT)
endif()

# Protobuf enables the MVT (Mapnik Vector Tiles) format
set(PROTOBUF_USE_DLLS FALSE CACHE BOOL "Set this to true if Protobuf is compiled as dll")

include(OsgEarthMacroUtils)

# Detect the OSG version so we can append it to plugin DLLs just like OSG does.
DETECT_OSG_VERSION()
if(NOT OPENSCENEGRAPH_VERSION)
	set(OPENSCENEGRAPH_VERSION ${OPENSCENEGRAPH_MAJOR_VERSION}.${OPENSCENEGRAPH_MINOR_VERSION}.${OPENSCENEGRAPH_PATCH_VERSION})
endif(NOT OPENSCENEGRAPH_VERSION)

if(OPENSCENEGRAPH_VERSION)
    message(STATUS "Found OSG version ${OPENSCENEGRAPH_VERSION}")
endif()

# Make the headers visible to everything
include_directories(
    ${OSGEARTH_SOURCE_DIR}/src
    ${OSG_INCLUDE_DIR}
    ${OSG_GEN_INCLUDE_DIR}
)


# Linking & Intallation ..................................................

# Assign SONAMEs to the shared libraries on Linux
set(OSGEARTH_SONAMES TRUE)

# postfix settings for various configs
set(CMAKE_DEBUG_POSTFIX  "d" CACHE STRING "add a Debug postfix, usually d on windows")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a Release postfix, usually empty on windows")
set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "add a RelWithDebInfo postfix, usually empty on windows")
set(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a MinSizeRel postfix, usually empty on windows")

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
   file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin ${CMAKE_BINARY_DIR}/lib)
endif("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")

if(UNIX AND NOT WIN32 AND NOT APPLE AND NOT ANDROID)
  if(CMAKE_SIZEOF_VOID_P MATCHES "8")
      set(LIB_POSTFIX "64" CACHE STRING "suffix for 32/64 dir placement")
      mark_as_advanced(LIB_POSTFIX)
  endif(CMAKE_SIZEOF_VOID_P MATCHES "8")
endif(UNIX AND NOT WIN32 AND NOT APPLE AND NOT ANDROID)
if(NOT DEFINED LIB_POSTFIX)
    set(LIB_POSTFIX "")
endif(NOT DEFINED LIB_POSTFIX)

set(OUTPUT_BINDIR ${PROJECT_BINARY_DIR}/bin)
make_directory(${OUTPUT_BINDIR})
set(EXECUTABLE_OUTPUT_PATH ${OUTPUT_BINDIR})

set(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib)
make_directory(${OUTPUT_LIBDIR})
set(LIBRARY_OUTPUT_PATH ${OUTPUT_LIBDIR})

link_directories( ${LINK_DIRECTORIES} ${OUTPUT_LIBDIR} )

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


# VCPkg Config File Creation .............................................

set(INSTALL_INCDIR ${CMAKE_INSTALL_PREFIX}/include)
set(INSTALL_LIBDIR ${PROJECT_BINARY_DIR}/lib)	
message(STATUS "INCDIR: ${INSTALL_INCDIR}")
include(CMakePackageConfigHelpers)

configure_package_config_file(
    osgEarthConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/osgEarthConfig.cmake
    INSTALL_DESTINATION ${LIB_INSTALL_DIR}/osgearth/cmake)
  
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/osgEarthConfigVersion.cmake
    VERSION 3.4.0
    COMPATIBILITY SameMajorVersion)
  
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/osgEarthConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/osgEarthConfigVersion.cmake
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}/cmake )
