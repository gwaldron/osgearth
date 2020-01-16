CMAKE_MINIMUM_REQUIRED(VERSION 3.1 FATAL_ERROR)

if(COMMAND cmake_policy)
    # Works around warnings libraries linked against that don't
    # have absolute paths (e.g. -lpthreads)
    cmake_policy(SET CMP0003 NEW)

    # Works around warnings about escaped quotes in ADD_DEFINITIONS
    # statements.
    cmake_policy(SET CMP0005 OLD)

    # Qt5 qt5_use_modules usage was causing "Policy CMP0043 is not set: Ignore COMPILE_DEFINITIONS_<Config> properties." warnings
    cmake_policy(SET CMP0043 NEW)

    # Double colon in target name means IMPORTED (and not ALIAS or IMPORTED).
    # Fixes a warning when cache_rocksdb links to target "RocksDB::rocksdb"
    cmake_policy(SET CMP0028 NEW)
endif(COMMAND cmake_policy)

#
# Set up CMake to use Solution Folders in VS.
#
SET_PROPERTY( GLOBAL PROPERTY USE_FOLDERS ON )
SET_PROPERTY( GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake Targets" )

PROJECT(OSGEARTH)

SET(OSGEARTH_MAJOR_VERSION 2)
SET(OSGEARTH_MINOR_VERSION 99)
SET(OSGEARTH_PATCH_VERSION 0)
SET(OSGEARTH_SOVERSION     0)

SET(OSGEARTH_PLUGIN_PREFIX "")

IF (CYGWIN)
    SET(OSGEARTH_PLUGIN_PREFIX "cygwin_")
ENDIF()

IF(MINGW)
    SET(OSGEARTH_PLUGIN_PREFIX "mingw_")
ENDIF()

# Detect out-of-source build
if (NOT "${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
    SET(OUT_OF_SOURCE_BUILD 1)
endif()

SET(OSGEARTH_VERSION ${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION})


# We want to build SONAMES shared libraries
SET(OSGEARTH_SONAMES TRUE)
SET(OPENTHREADS_SONAMES TRUE)

SET(OpenThreads_SOURCE_DIR ${OSGEARTH_SOURCE_DIR})

# Good place to look for the OSG 3rd party libs
SET(THIRD_PARTY_DIR "" CACHE PATH "Base 3rd-party dependency folder (containing include, lib, bin)")

IF(THIRD_PARTY_DIR)
    LIST(APPEND CMAKE_INCLUDE_PATH ${THIRD_PARTY_DIR}/include)
    LIST(APPEND CMAKE_LIBRARY_PATH ${THIRD_PARTY_DIR}/lib)
ENDIF(THIRD_PARTY_DIR)

# We have some custom .cmake scripts not in the official distribution.
SET(CMAKE_MODULE_PATH "${OSGEARTH_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

# Whether to compile for GLES (mobile/web)
SET(OSGEARTH_USE_GLES FALSE)

# Detect C++11 compiler support and set a variable.
INCLUDE(CXX11)
check_for_cxx11_compiler(CXX11_AVAILABLE)

# Special folder for build-time generated include files
set(OSGEARTH_BUILDTIME_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/build_include")
include_directories(${OSGEARTH_BUILDTIME_INCLUDE_DIR})

# Platform-specific stuff
INCLUDE(oe_ios)
INCLUDE(oe_osx)
INCLUDE(oe_unix)
INCLUDE(oe_win32)

# check if STLport is required
OPTION(USE_STLPORT "Set to ON to build OSGEARTH with stlport instead of the default STL library." OFF)
IF(USE_STLPORT)
	INCLUDE_DIRECTORIES(${STLPORT_DIR}/stlport)
	LINK_DIRECTORIES(${STLPORT_DIR}/lib)
ENDIF()

# don't think this is necessary since we are using OpenThreads
#FIND_PACKAGE(Threads)

IF (OSGEARTH_USE_GLES)
    FIND_PACKAGE(OpenGLES) 
ELSE ()
    FIND_PACKAGE(OpenGL)
ENDIF (OSGEARTH_USE_GLES)

FIND_PACKAGE(CURL)
FIND_PACKAGE(GDAL)
FIND_PACKAGE(GEOS)
FIND_PACKAGE(Sqlite3)
FIND_PACKAGE(Draco)
FIND_PACKAGE(BASISU)
FIND_PACKAGE(Tracy)

IF(SQLITE3_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_MBTILES)
ENDIF(SQLITE3_FOUND)

IF(draco_FOUND)
    ADD_DEFINITIONS(-DOSGEARTH_HAVE_DRACO)
ENDIF(draco_FOUND)

OPTION(ENABLE_PROFILING "Build with support for Tracy profiler" OFF)
IF(TRACY_FOUND AND ENABLE_PROFILING)
    ADD_DEFINITIONS(-DOSGEARTH_PROFILING)
ENDIF(TRACY_FOUND AND ENABLE_PROFILING)

FIND_PACKAGE(Triton QUIET)
FIND_PACKAGE(SilverLining QUIET)

FIND_PACKAGE(protobuf QUIET)
SET (PROTOBUF_USE_DLLS FALSE CACHE BOOL "Set this to true if Protobuf is compiled as dll")

SET (WITH_EXTERNAL_DUKTAPE FALSE CACHE BOOL "Use bundled or system wide version of Duktape")
IF (WITH_EXTERNAL_DUKTAPE)
    FIND_PACKAGE(Duktape)
ENDIF (WITH_EXTERNAL_DUKTAPE)

# option to install shaders:
OPTION(OSGEARTH_INSTALL_SHADERS "Whether to deploy GLSL shaders when doing a Make INSTALL" OFF)

SET (WITH_EXTERNAL_TINYXML FALSE CACHE BOOL "Use bundled or system wide version of TinyXML")
IF (WITH_EXTERNAL_TINYXML)
    FIND_PACKAGE(TinyXML)
ENDIF (WITH_EXTERNAL_TINYXML)

# postfix settings for various configs
SET(CMAKE_DEBUG_POSTFIX  "d" CACHE STRING "add a Debug postfix, usually d on windows")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a Release postfix, usually empty on windows")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "add a RelWithDebInfo postfix, usually empty on windows")
SET(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a MinSizeRel postfix, usually empty on windows")

INCLUDE(OsgEarthMacroUtils)

DETECT_OSG_VERSION()

IF (NOT OPENSCENEGRAPH_VERSION)
	SET(OPENSCENEGRAPH_VERSION ${OPENSCENEGRAPH_MAJOR_VERSION}.${OPENSCENEGRAPH_MINOR_VERSION}.${OPENSCENEGRAPH_PATCH_VERSION})
ENDIF(NOT OPENSCENEGRAPH_VERSION)

FIND_PACKAGE(OSG)

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
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH}  CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")

# Dynamic vs Static Linking
OPTION(DYNAMIC_OSGEARTH "Set to ON to build OSGEARTH for dynamic linking.  Use OFF for static." ON)
IF   (DYNAMIC_OSGEARTH)
    SET(OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC "SHARED")
ELSE (DYNAMIC_OSGEARTH)
    SET(OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC "STATIC")
ENDIF(DYNAMIC_OSGEARTH)

# Whether to copy osgEarth to the OSG_DIR during installation
OPTION(INSTALL_TO_OSG_DIR "Set to ON to install osgEarth to the path defined in the OSG_DIR" OFF)
IF(INSTALL_TO_OSG_DIR)
    SET(OSGEARTH_INSTALL_TO_OSG_DIR TRUE)
ELSE (INSTALL_TO_OSG_DIR)
    SET(OSGEARTH_INSTALL_TO_OSG_DIR FALSE)
ENDIF(INSTALL_TO_OSG_DIR)

# osgEarth Examples
OPTION(BUILD_OSGEARTH_EXAMPLES "Enable to build osgEarth Examples" ON)

# Applications and tests
OPTION(BUILD_APPLICATIONS "Enable build of Applications" ON)
OPTION(BUILD_TESTS "Enable build of Tests" ON)

# OE Core
ADD_SUBDIRECTORY(src)

# Add the docs to the project for convenience
ADD_SUBDIRECTORY(docs)
