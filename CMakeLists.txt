CMAKE_MINIMUM_REQUIRED(VERSION 3.1 FATAL_ERROR)

if(COMMAND cmake_policy)
    # Works around warnings libraries linked against that don't
    # have absolute paths (e.g. -lpthreads)
    cmake_policy(SET CMP0003 NEW)

    # Qt5 qt5_use_modules usage was causing "Policy CMP0043 is not set: Ignore COMPILE_DEFINITIONS_<Config> properties." warnings
    cmake_policy(SET CMP0043 NEW)

    # Double colon in target name means IMPORTED (and not ALIAS or IMPORTED).
    # Fixes a warning when cache_rocksdb links to target "RocksDB::rocksdb"
    cmake_policy(SET CMP0028 NEW)
    
    # Defaults find_package to using *_ROOT, which did not become the default
    # policy until CMake 3.12
    if (POLICY CMP0074)
        cmake_policy(SET CMP0074 NEW)
    endif()
endif(COMMAND cmake_policy)

# Set up CMake to use Solution Folders in Visual Studio.
SET_PROPERTY( GLOBAL PROPERTY USE_FOLDERS ON )
SET_PROPERTY( GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake Targets" )

PROJECT(OSGEARTH)

# SDK version number (informational)
SET(OSGEARTH_MAJOR_VERSION 3)
SET(OSGEARTH_MINOR_VERSION 2)
SET(OSGEARTH_PATCH_VERSION 0)

# Increment this each time the ABI changes
SET(OSGEARTH_SOVERSION 118)

# C++11 is required for osgEarth 3.x
set_property(GLOBAL PROPERTY CXX_STANDARD 11)
set_property(GLOBAL PROPERTY CXX_STANDARD_REQUIRED ON)

# Assign a custom prefix if necessary.
SET(OSGEARTH_PLUGIN_PREFIX "")

IF (CYGWIN)
    SET(OSGEARTH_PLUGIN_PREFIX "cygwin_")
ENDIF()

IF(MINGW)
    SET(OSGEARTH_PLUGIN_PREFIX "mingw_")
ENDIF()

# Detect out-of-source build. You should always do an out-of-source build!
if (NOT "${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
    SET(OUT_OF_SOURCE_BUILD 1)
endif()

SET(OSGEARTH_VERSION ${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION})


# Assign SONAMEs to the shared libraries on Linux
SET(OSGEARTH_SONAMES TRUE)

# Good place to look for dependencies (e.g., vcpkg)
SET(OSGEARTH_DEPENDENCY_DIR "" CACHE PATH "Folder to search for dependencies (e.g., vcpkg/installed/x64-windows)")

IF(OSGEARTH_DEPENDENCY_DIR)
    LIST(APPEND CMAKE_INCLUDE_PATH ${OSGEARTH_DEPENDENCY_DIR}/include)
    LIST(APPEND CMAKE_LIBRARY_PATH ${OSGEARTH_DEPENDENCY_DIR}/lib)
ENDIF()

# We have some custom .cmake scripts not in the official distribution.
SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${OSGEARTH_SOURCE_DIR}/CMakeModules;${OSGEARTH_DEPENDENCY_DIR}")

# Whether to compile for GLES (mobile/web)
SET(OSGEARTH_USE_GLES FALSE)

# Special folder for build-time generated include files
set(OSGEARTH_BUILDTIME_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/build_include")
include_directories(${OSGEARTH_BUILDTIME_INCLUDE_DIR})

# Third-party sources included in the osgEarth repo
SET(OSGEARTH_EMBEDDED_THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/src/third_party)

# Ensure and configure C++11 compiler support.
INCLUDE(CXX11)
check_for_cxx11_compiler(CXX11_AVAILABLE)

# Location of GLCORE headers if you are using GLCORE
set(OSGEARTH_GLCORE_INCLUDE_DIR CACHE PATH "For OpenGL CORE Profile, location of the GLCORE header files")
if (OSGEARTH_GLCORE_INCLUDE_DIR)
    include_directories(${OSGEARTH_GLCORE_INCLUDE_DIR})
endif()

# Platform-specific settings
INCLUDE(oe_ios)
INCLUDE(oe_osx)
INCLUDE(oe_unix)
INCLUDE(oe_win32)


# ........................................................................
# Optional build targets

option(OSGEARTH_BUILD_TOOLS "Build the osgEarth command-line tools" ON)
option(OSGEARTH_BUILD_EXAMPLES "Build the osgEarth example applications" ON)
option(OSGEARTH_BUILD_TESTS "Build the osgEarth unit tests" ON)

option(OSGEARTH_BUILD_PROCEDURAL_NODEKIT "Build the procedural terrain nodekit (osgEarthProcedural)" OFF)
option(OSGEARTH_BUILD_LEGACY_SPLAT_NODEKIT "Build the legacy procedural nodekit (osgEarthSplat)" OFF)
option(OSGEARTH_BUILD_TRITON_NODEKIT "Build support for SunDog Triton SDK" OFF)
option(OSGEARTH_BUILD_SILVERLINING_NODEKIT "Build support for SunDog SilverLining SDK" OFF)
option(OSGEARTH_BUILD_LEVELDB_CACHE "Build the LevelDB cache plugin" OFF)
option(OSGEARTH_BUILD_ROCKSDB_CACHE "Build the RocksDB cache plugin" OFF)
option(OSGEARTH_BUILD_ZIP_PLUGIN "Build osgEarth's zip plugin based on libzip" ON)
option(OSGEARTH_BUILD_DOCS "Include the documentation folder" ON)

# ........................................................................
# Optional features

option(OSGEARTH_ENABLE_PROFILING "Enable profiling with Tracy" OFF)
option(OSGEARTH_ENABLE_GEOCODER "Enable the geocoder (GDAL/OGR must be built with geocoder support)" OFF)

# ........................................................................
# Dependencies

# Mobile/GLES:
IF (OSGEARTH_USE_GLES)
    find_package(OpenGLES)
ELSE ()
    find_package(OpenGL)
ENDIF (OSGEARTH_USE_GLES)


# Update git submodules in case the user didn't do it already
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

if(OSGEARTH_ENABLE_PROFILING)
    find_package(Tracy)
endif()

if(OSGEARTH_BUILD_ZIP_PLUGIN)
    find_package(LIBZIP)
endif()

if(OSGEARTH_BUILD_TRITON_NODEKIT)
    find_package(Triton QUIET)
endif()

if(OSGEARTH_BUILD_SILVERLINING_NODEKIT)
    find_package(SilverLining QUIET)
endif()

# Sqlite enables the MBTiles format:
IF(SQLITE3_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_MBTILES)
ENDIF(SQLITE3_FOUND)

# DRACO enables geometry compression sometimes used in glTF:
IF(draco_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_DRACO)
ENDIF(draco_FOUND)

# GDAL is the underlying geospatial processing SDK
IF(GDAL_FOUND)
    message(STATUS "Found GDAL ${GDAL_VERSION}" )
ENDIF(GDAL_FOUND)

# Tracy is a profiler
IF(TRACY_FOUND AND OSGEARTH_ENABLE_PROFILING)
    ADD_DEFINITIONS(-DOSGEARTH_PROFILING)
ENDIF()

if(GLEW_FOUND AND EXISTS "${OSGEARTH_SOURCE_DIR}/src/third_party/imgui/imgui.cpp")
    set(OSGEARTH_ENABLE_IMGUI 1)
    add_definitions(-DOSGEARTH_ENABLE_IMGUI)
endif()

if(OSGEARTH_ENABLE_GEOCODER)
    add_definitions(-DOSGEARTH_HAVE_GEOCODER)
endif()

# Protobuf enables the MVT (Mapnik Vector Tiles) format
SET (PROTOBUF_USE_DLLS FALSE CACHE BOOL "Set this to true if Protobuf is compiled as dll")

# Duktape is the JavaScript interpreter
SET (WITH_EXTERNAL_DUKTAPE FALSE CACHE BOOL "Use bundled or system wide version of Duktape")
IF (WITH_EXTERNAL_DUKTAPE)
    find_package(Duktape)
ENDIF (WITH_EXTERNAL_DUKTAPE)

# Whether to install shaders (glsl files).
# If true, shaders install into a resources folder. If false, they are inlined in the
# code and you cannot tweak them after install.
OPTION(OSGEARTH_INSTALL_SHADERS "Whether to deploy GLSL shaders when doing a Make INSTALL" OFF)

# TinyXML is an XML parsing library
SET (WITH_EXTERNAL_TINYXML FALSE CACHE BOOL "Use bundled or system wide version of TinyXML")
IF (WITH_EXTERNAL_TINYXML)
    find_package(TinyXML)
ENDIF (WITH_EXTERNAL_TINYXML)

# postfix settings for various configs
SET(CMAKE_DEBUG_POSTFIX  "d" CACHE STRING "add a Debug postfix, usually d on windows")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a Release postfix, usually empty on windows")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "add a RelWithDebInfo postfix, usually empty on windows")
SET(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a MinSizeRel postfix, usually empty on windows")

INCLUDE(OsgEarthMacroUtils)

# OpenSceneGraph is the 3D rendering toolkit
DETECT_OSG_VERSION()

IF (NOT OPENSCENEGRAPH_VERSION)
	SET(OPENSCENEGRAPH_VERSION ${OPENSCENEGRAPH_MAJOR_VERSION}.${OPENSCENEGRAPH_MINOR_VERSION}.${OPENSCENEGRAPH_PATCH_VERSION})
ENDIF(NOT OPENSCENEGRAPH_VERSION)

if(OPENSCENEGRAPH_VERSION)
    message(STATUS "Found OSG version ${OPENSCENEGRAPH_VERSION}")
endif()

# Make the headers visible to everything
INCLUDE_DIRECTORIES(
    ${OSGEARTH_SOURCE_DIR}/src
    ${OSG_INCLUDE_DIR}
    ${OSG_GEN_INCLUDE_DIR}
)


################################################################################
# Create bin and lib directories if required

IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
   FILE(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin ${CMAKE_BINARY_DIR}/lib)
ENDIF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")



################################################################################
# Installation target

IF(UNIX AND NOT WIN32 AND NOT APPLE AND NOT ANDROID)
  IF(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET(LIB_POSTFIX "64" CACHE STRING "suffix for 32/64 dir placement")
      MARK_AS_ADVANCED(LIB_POSTFIX)
  ENDIF(CMAKE_SIZEOF_VOID_P MATCHES "8")
ENDIF(UNIX AND NOT WIN32 AND NOT APPLE AND NOT ANDROID)
IF(NOT DEFINED LIB_POSTFIX)
    SET(LIB_POSTFIX "")
ENDIF(NOT DEFINED LIB_POSTFIX)

SET(OUTPUT_BINDIR ${PROJECT_BINARY_DIR}/bin)
MAKE_DIRECTORY(${OUTPUT_BINDIR})
SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_BINDIR})

SET(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib)
MAKE_DIRECTORY(${OUTPUT_LIBDIR})
SET(LIBRARY_OUTPUT_PATH ${OUTPUT_LIBDIR})

LINK_DIRECTORIES( ${LINK_DIRECTORIES} ${OUTPUT_LIBDIR} )


# Expose CMAKE_INCLUDE_PATH and CMAKE_LIBARY_PATH to the GUI so users
# may set these values without needing to manipulate the environment.
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} CACHE STRING "Add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} CACHE STRING "Add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH}  CACHE STRING "Add additional search paths here. Use ; to separate multiple paths.")

# Dynamic vs Static Linking
OPTION(OSGEARTH_BUILD_SHARED_LIBS "ON to build shared libraries; OFF to build static libraries." ON)
IF(OSGEARTH_BUILD_SHARED_LIBS)
    SET(OSGEARTH_DYNAMIC_OR_STATIC "SHARED")
ELSE()
    SET(OSGEARTH_DYNAMIC_OR_STATIC "STATIC")
ENDIF()


# OE Libraries
ADD_SUBDIRECTORY(src)

if (OSGEARTH_BUILD_DOCS)
    ADD_SUBDIRECTORY(docs)
endif()
