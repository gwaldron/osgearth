CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4 FATAL_ERROR)

if(COMMAND cmake_policy)
    # Works around warnings libraries linked against that don't
    # have absolute paths (e.g. -lpthreads)
    cmake_policy(SET CMP0003 NEW)

    # Works around warnings about escaped quotes in ADD_DEFINITIONS
    # statements.
    cmake_policy(SET CMP0005 OLD)
endif(COMMAND cmake_policy)

PROJECT(OSGEARTH)

SET(OSGEARTH_MAJOR_VERSION 2)
SET(OSGEARTH_MINOR_VERSION 0)
SET(OSGEARTH_PATCH_VERSION 0)
SET(OSGEARTH_SOVERSION 1)

SET(OSGEARTH_PLUGIN_PREFIX "")

IF (CYGWIN)
    SET(OSGEARTH_PLUGIN_PREFIX "cygwin_")
ENDIF()

IF(MINGW)
    SET(OSGEARTH_PLUGIN_PREFIX "mingw_")
ENDIF()


SET(OSGEARTH_VERSION ${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION})

# We want to build SONAMES shared librariess
SET(OSGEARTH_SONAMES TRUE)
SET(OPENTHREADS_SONAMES TRUE)

SET(OpenThreads_SOURCE_DIR ${OSGEARTH_SOURCE_DIR})

# We have some custom .cmake scripts not in the official distribution.
# Maybe this can be used override existing behavior if needed?
SET(CMAKE_MODULE_PATH "${OSGEARTH_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

# check if STLport is required
OPTION(USE_STLPORT "Set to ON to build OSGEARTH with stlport instead of the default STL library." OFF)
IF(USE_STLPORT)
	INCLUDE_DIRECTORIES(${STLPORT_DIR}/stlport)
	LINK_DIRECTORIES(${STLPORT_DIR}/lib)
ENDIF()

# Okay, here's the problem: On some platforms, linking against OpenThreads
# is not enough and explicit linking to the underlying thread library
# is also required (e.g. FreeBSD). But OpenThreads may be built with different
# backends (Pthreads, Sproc, Windows) so we don't know what the underlying
# thread library is because some platforms support multiple backends (e.g.
# IRIX supports Sproc and Pthreads). Linking all libraries won't work
# because the libraries may be incompatible.
# So the current solution is to attempt best guess linking and exempt certain
# cases. With IRIX, we're going to hope explicit linking to the underlying
# library is not necessary. We currently don't case for pthreads on Windows
# which might be an issue on things like Cygwin. This may need to be fixed.
FIND_PACKAGE(Threads)
IF(CMAKE_SYSTEM MATCHES IRIX)
    # Erase CMAKE_THREAD_LIBS_INIT and hope it works
    SET(CMAKE_THREAD_LIBS_INIT "" CACHE INTERNAL "")
ENDIF(CMAKE_SYSTEM MATCHES IRIX)

FIND_PACKAGE(OpenGL)

FIND_PACKAGE(CURL)
FIND_PACKAGE(GDAL)
FIND_PACKAGE(LibZip)
FIND_PACKAGE(GEOS)
FIND_PACKAGE(Sqlite3)
FIND_PACKAGE(ZLIB)

IF(UNIX)
    # Not sure what this will do on Cygwin and Msys
    # Also, remember OS X X11 is a user installed option so it may not exist.
    FIND_PACKAGE(X11)
    # Some Unicies need explicit linkage to the Math library or the build fails.
    FIND_LIBRARY(MATH_LIBRARY m)
ENDIF(UNIX)

# Make the headers visible to everything
INCLUDE_DIRECTORIES(
    ${OSGEARTH_SOURCE_DIR}/src
    ${OSG_DIR}/include
    ${OSG_INCLUDE_DIR}
)

# Common global definitions
#ADD_DEFINITIONS(-D)
# Platform specific definitions

IF(WIN32)
  IF(MSVC)
        # This option is to enable the /MP switch for Visual Studio 2005 and above compilers
        OPTION(WIN32_USE_MP "Set to ON to build osgEarth with the /MP option (Visual Studio 2005 and above)." OFF)
        MARK_AS_ADVANCED(WIN32_USE_MP)
        IF(WIN32_USE_MP)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
        ENDIF(WIN32_USE_MP)

        # turn off various warnings
        # foreach(warning 4244 4251 4267 4275 4290 4786 4305 4996)
        #     SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${warning}")
        # endforeach(warning)

        # More MSVC specific compilation flags
        ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
        ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
    ENDIF(MSVC)
ENDIF(WIN32)

OPTION(ENABLE_WALL "Enable the -Wall compiler option." OFF)
IF(ENABLE_WALL)
	IF(CMAKE_COMPILER_IS_GNUCXX)
		ADD_DEFINITIONS(-Wall)
	ENDIF(CMAKE_COMPILER_IS_GNUCXX)
	IF(MSVC)
		ADD_DEFINITIONS(/Wall)
	ENDIF(MSVC)
ENDIF(ENABLE_WALL)

########################################################################################################
##### these were settings located in SetupCommon.cmake used in Luigi builds.... find out what are useful
########################################################################################################
#luigi#SET(CMAKE_VERBOSE_MAKEFILE TRUE)
#luigi#SET(CMAKE_SKIP_RPATH TRUE)
#luigi#SET(CMAKE_SKIP_RULE_DEPENDENCY TRUE)
#luigi#IF(UNIX)
#luigi#    LIST_CONTAINS(contains "g++" ${CMAKE_CXX_COMPILER_LIST})
#luigi#    IF (contains)
#luigi#        MESSAGE(${MY_MESSAGE_DEFAULT} "${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} setting  CMAKE_CXX_COMPILER to g++")
#luigi#        SET(CMAKE_CXX_COMPILER "g++")
#luigi#        SET(CMAKE_CXX_COMPILER_LOADED 2)
#luigi#        SET(CMAKE_CXX_COMPILER_WORKS 2)
#luigi#    ENDIF (contains)
#luigi#    SET(CMAKE_CXX_FLAGS_RELEASE "-O2")
#luigi#    SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -gstabs")
#luigi#ENDIF(UNIX)
########################################################################################################

# Common to all platforms:
SET(OSG_DIR "" CACHE PATH "set to base osg install path")

SET(CMAKE_DEBUG_POSTFIX  "d" CACHE STRING "add a postfix, usually d on windows")

FIND_PACKAGE(OSG)


#FIND_PACKAGE(GDAL)

################################################################################
# Create bin and lib directories if required

IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
   FILE(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin ${CMAKE_BINARY_DIR}/lib)
ENDIF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")


################################################################################
# Installation stuff


IF(UNIX AND NOT WIN32 AND NOT APPLE)
  IF(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET(LIB_POSTFIX "64" CACHE STRING "suffix for 32/64 dir placement")
      MARK_AS_ADVANCED(LIB_POSTFIX)
  ENDIF(CMAKE_SIZEOF_VOID_P MATCHES "8")
ENDIF(UNIX AND NOT WIN32 AND NOT APPLE)
IF(NOT DEFINED LIB_POSTFIX)
    SET(LIB_POSTFIX "")
ENDIF(NOT DEFINED LIB_POSTFIX)

#SET(OUTPUT_BINDIR ${PROJECT_BINARY_DIR}/bin/${CMAKE_SYSTEM_NAME})
SET(OUTPUT_BINDIR ${PROJECT_BINARY_DIR}/bin)
MAKE_DIRECTORY(${OUTPUT_BINDIR})
SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_BINDIR})

#SET(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib/${CMAKE_SYSTEM_NAME})
SET(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib)
MAKE_DIRECTORY(${OUTPUT_LIBDIR})
SET(LIBRARY_OUTPUT_PATH ${OUTPUT_LIBDIR})

LINK_DIRECTORIES( ${LINK_DIRECTORIES} ${OUTPUT_LIBDIR} )


#SET(INSTALL_BINDIR VIRTUALPLANETBUILDER/bin)
#SET(INSTALL_INCDIR VIRTUALPLANETBUILDER/include)
#SET(INSTALL_LIBDIR VIRTUALPLANETBUILDER/lib)
#SET(INSTALL_DOCDIR VIRTUALPLANETBUILDER/doc)

################################################################################
# User Options


# Expose CMAKE_INCLUDE_PATH and CMAKE_LIBARY_PATH to the GUI so users
# may set these values without needing to manipulate the environment.
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
# We are proposing that a new variable called CMAKE_PREFIX_PATH be introduced
# to CMake to compliment CMAKE_INCLUDE_PATH and CMAKE_LIBRARY_PATH.
# A formal feature request has been submited to CMake, Bug #4947.
# It is intended for those users who have common prefixes for their INCLUDE
# and LIBRARY locations. So if users have headers in /usr/local/include
# and libraries in /usr/local/lib, the common prefix is /usr/local.
# It should also cover the case where headers and libraries are
# in the same directory.
# Our proposal expects that FIND_* commands will automatically search for
# CMAKE_PREFIX_PATH right after CMAKE_INCLUDE_PATH or CMAKE_LIBRARY_PATH.
# Obviously, since CMake does not currently support this, we must write
# our Find*.cmake modules to explicitly support this. Otherwise, this variable
# will have no impact.
# This is unofficial so this may be removed or changed at anytime.
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} CACHE STRING "(EXPERIMENTAL) You may add additional search paths here. Use ; to separate multiple paths.")

# Dynamic vs Static Linking
OPTION(DYNAMIC_OSGEARTH "Set to ON to build OSGEARTH for dynamic linking.  Use OFF for static." ON)
IF   (DYNAMIC_OSGEARTH)
    SET(OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC "SHARED")
ELSE (DYNAMIC_OSGEARTH)
    SET(OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC "STATIC")
ENDIF(DYNAMIC_OSGEARTH)

#Whether or not to copy osgEarth to the OSG_DIR during installation
# Dynamic vs Static Linking
OPTION(INSTALL_TO_OSG_DIR "Set to ON to install osgearth to the path defined in the OSG_DIR" OFF)
IF(INSTALL_TO_OSG_DIR)
    SET(OSGEARTH_INSTALL_TO_OSG_DIR TRUE)
ELSE (INSTALL_TO_OSG_DIR)
    SET(OSGEARTH_INSTALL_TO_OSG_DIR FALSE)
ENDIF(INSTALL_TO_OSG_DIR)


INCLUDE(OsgEarthMacroUtils)

DETECT_OSG_VERSION()

SET(OPENSCENEGRAPH_VERSION ${OPENSCENEGRAPH_MAJOR_VERSION}.${OPENSCENEGRAPH_MINOR_VERSION}.${OPENSCENEGRAPH_PATCH_VERSION})

# OE Core
ADD_SUBDIRECTORY(src)

# VPB Applications
# OPTION(BUILD_APPLICATIONS "Enable to build OSGEARTH Applications" ON)
# IF   (BUILD_APPLICATIONS)
    # ADD_SUBDIRECTORY(applications)
# ENDIF(BUILD_APPLICATIONS)



# Set defaults for Universal Binaries. We want 32-bit Intel/PPC on 10.4
# and 32/64-bit Intel/PPC on >= 10.5. Anything <= 10.3 doesn't support.
IF(APPLE)
    # These are just defaults/recommendations, but how we want to build
    # out of the box. But the user needs to be able to change these options.
    # So we must only set the values the first time CMake is run, or we
    # will overwrite any changes the user sets.
    # FORCE is used because the options are not reflected in the UI otherwise.
    # Seems like a good place to add version specific compiler flags too.
    IF(NOT OSGEARTH_CONFIG_HAS_BEEN_RUN_BEFORE)
        # This is really fragile, but CMake doesn't provide the OS system
        # version information we need. (Darwin versions can be changed
        # independently of OS X versions.)
        # It does look like CMake handles the CMAKE_OSX_SYSROOT automatically.
        IF(EXISTS /Developer/SDKs/10.5.sdk)
            SET(CMAKE_OSX_ARCHITECTURES "ppc;i386;ppc64;x86_64" CACHE STRING "Build architectures for OSX" FORCE)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.5 -ftree-vectorize -fvisibility-inlines-hidden" CACHE STRING "Flags used by the compiler during all build types." FORCE)
        ELSE(EXISTS /Developer/SDKs/10.5.sdk)
            IF(EXISTS /Developer/SDKs/MacOSX10.4u.sdk)
                SET(CMAKE_OSX_ARCHITECTURES "ppc;i386" CACHE STRING "Build architectures for OSX" FORCE)
                SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.4 -ftree-vectorize -fvisibility-inlines-hidden" CACHE STRING "Flags used by the compiler during all build types." FORCE)
            ELSE(EXISTS /Developer/SDKs/MacOSX10.4u.sdk)
                # No Universal Binary support
                # Should break down further to set the -mmacosx-version-min,
                # but the SDK detection is too unreliable here.
            ENDIF(EXISTS /Developer/SDKs/MacOSX10.4u.sdk)
        ENDIF(EXISTS /Developer/SDKs/10.5.sdk)
    ENDIF(NOT OSGEARTH_CONFIG_HAS_BEEN_RUN_BEFORE)

    OPTION(OSGEARTH_BUILD_APPLICATION_BUNDLES "Enable the building of applications and examples as OSX Bundles" OFF)
    OPTION(OSGEARTH_BUILD_FRAMEWORKS "Compile frameworks instead of dylibs" OFF)
    SET(OSGEARTH_BUILD_FRAMEWORKS_INSTALL_NAME_DIR "@executable_path/../Frameworks" CACHE STRING "Install name dir for compiled frameworks")

ENDIF(APPLE)


# This needs to be run very last so other parts of the scripts can take
# advantage of this.
IF(NOT OSGEARTH_CONFIG_HAS_BEEN_RUN_BEFORE)
    SET(OSGEARTH_CONFIG_HAS_BEEN_RUN_BEFORE 1 CACHE INTERNAL "Flag to track whether this is the first time running CMake or if CMake has been configured before")
ENDIF(NOT OSGEARTH_CONFIG_HAS_BEEN_RUN_BEFORE)


#-----------------------------------------------------------------------------
### uninstall target
#-----------------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

