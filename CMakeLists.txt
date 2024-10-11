cmake_minimum_required(VERSION 3.20)

# VCPKG customization ....................................................

if (${CMAKE_TOOLCHAIN_FILE} MATCHES ".*vcpkg.cmake.*")
    message(STATUS "Building with vcpkg toolchain.")
    set(USING_VCPKG ON)
    set(VCPKG_OVERLAY_TRIPLETS ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/triplets)
endif()


# Project setup ..........................................................

project(
    OSGEARTH
    DESCRIPTION "osgEarth SDK"
    HOMEPAGE_URL "https://github.com/gwaldron/osgearth"
    LANGUAGES CXX C)

# SDK version number
set(OSGEARTH_MAJOR_VERSION 3)
set(OSGEARTH_MINOR_VERSION 7)
set(OSGEARTH_PATCH_VERSION 1)
set(OSGEARTH_VERSION ${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION})

# Increment this each time the ABI changes
set(OSGEARTH_SOVERSION 164)

# Require C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect out-of-source build. We'll need this for protobuf generated files.
if (NOT "${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
    message(STATUS "Detected an out-of-source build. Kudos.")    
    set(OSGEARTH_OUT_OF_SOURCE_BUILD TRUE)
else()
    message(STATUS "Detected a in-source build.")  
endif()

# We have some custom .cmake scripts not in the official distribution.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")

# Special folder for build-time generated include files
set(OSGEARTH_BUILDTIME_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/build_include")
include_directories(${OSGEARTH_BUILDTIME_INCLUDE_DIR})

# Third-party sources included in the reposotory
set(OSGEARTH_EMBEDDED_THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/src/third_party)
    
# Platform-specific settings ............................................

# Default installation folders. The platform-specific includes will
# override these values as necessary.
set(INSTALL_RUNTIME_FOLDER "bin${OSGEARTH_INSTALL_FOLDER_SUFFIX}")  # executables
set(INSTALL_LIBRARY_FOLDER "bin${OSGEARTH_INSTALL_FOLDER_SUFFIX}")  # .dll/.so
set(INSTALL_ARCHIVE_FOLDER "lib${OSGEARTH_INSTALL_FOLDER_SUFFIX}")  # .lib/.a
set(INSTALL_PLUGINS_FOLDER "bin${OSGEARTH_INSTALL_FOLDER_SUFFIX}")  # parent folder of OSG plugins folder

include(oe_ios)
include(oe_osx)
include(oe_unix)
include(oe_win32)

# Build options ..........................................................

option(OSGEARTH_BUILD_TOOLS "Build the osgEarth command-line tools" ON)
option(OSGEARTH_BUILD_EXAMPLES "Build the osgEarth example applications" ON)
option(OSGEARTH_BUILD_IMGUI_NODEKIT "Build the osgEarth ImGui nodekit and ImGui-based apps" ON)
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

if(WIN32)
    option(OSGEARTH_INSTALL_PDBS "Whether to deploy Windows .pdb files" OFF)
endif()

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

# required - globally used
find_package(OpenGL REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS osgManipulator osgShadow osgSim osgViewer osgGA osgUtil osgText osgDB osg OpenThreads)

# For static builds we need to link with Fontconfig directly.
# There is an unknown issue where the Fc symbols are not found when linking executables statically.
find_package(Fontconfig QUIET)
if (Fontconfig_FOUND)
    list(APPEND OPENSCENEGRAPH_LIBRARIES Fontconfig::Fontconfig)
endif()

# integrated profiling with tracy?
if(OSGEARTH_ENABLE_PROFILING)
    find_package(Tracy)
    if(Tracy_FOUND)
        message(STATUS "Found Tracy. Enabling frame profiling.")
    endif()
endif()

# optimization option:
if(OSGEARTH_ASSUME_SINGLE_GL_CONTEXT)
    add_definitions(-DOSGEARTH_SINGLE_GL_CONTEXT)
endif()

# optimization option:
if(OSGEARTH_ASSUME_SINGLE_THREADED_OSG)
    add_definitions(-DOSGEARTH_SINGLE_THREADED_OSG)
endif()

# Bring in our utility macros that sub-projects will use to configure themselves.
include(cmake/osgearth-macros.cmake)
include(cmake/install-package-config-files.cmake)

# Detect the OSG version so we can append it to plugin DLLs just like OSG does.
detect_osg_version()

if(NOT OPENSCENEGRAPH_VERSION)
	set(OPENSCENEGRAPH_VERSION ${OPENSCENEGRAPH_MAJOR_VERSION}.${OPENSCENEGRAPH_MINOR_VERSION}.${OPENSCENEGRAPH_PATCH_VERSION})
endif()
message(STATUS "Found OSG version ${OPENSCENEGRAPH_VERSION}")

# Make the headers visible to everything
include_directories(
    ${OSGEARTH_SOURCE_DIR}/src
    ${OPENSCENEGRAPH_INCLUDE_DIR})
    
# If OSG is built with OPENGL_PROFILE=GLCORE, <osg/GL> will try to include the GLCORE ARB headers.
# Set this variable to make those headers available when building osgEarth.
set(OSGEARTH_GLCORE_INCLUDE_DIR "" CACHE PATH "Location of OpenGL CORE profile header parent folder")
mark_as_advanced(OSGEARTH_GLCORE_INCLUDE_DIR)
if(OSGEARTH_GLCORE_INCLUDE_DIR)
    include_directories(${OSGEARTH_GLCORE_INCLUDE_DIR})
endif()

# Shared v. static build .................................................

option(OSGEARTH_BUILD_SHARED_LIBS "ON to build shared libraries; OFF to build static libraries." ON)
if(OSGEARTH_BUILD_SHARED_LIBS)
    set(OSGEARTH_DYNAMIC_OR_STATIC "SHARED")
else()
    set(OSGEARTH_DYNAMIC_OR_STATIC "STATIC")
endif()
message(STATUS "Building ${OSGEARTH_DYNAMIC_OR_STATIC} libraries")

# Whether to append SOVERSIONs to libraries (unix)
option(OSGEARTH_SONAMES "ON to append so-version numbers to libraries" ON)


# Source code ............................................................

add_subdirectory(src)

if(OSGEARTH_BUILD_DOCS)
    add_subdirectory(docs)
endif()
    
    
# IDE configuration ......................................................

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake Targets")


# Uninstall target .......................................................

configure_file(
   "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
   "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
   IMMEDIATE @ONLY
)

add_custom_target(uninstall
   "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
)
