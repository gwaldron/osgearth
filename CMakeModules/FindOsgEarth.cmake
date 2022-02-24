# This module defines

# OSGEARTH_LIBRARY
# OSGEARTH_FOUND, if false, do not try to link to osg
# OSGEARTH_INCLUDE_DIRS, where to find the headers
# OSGEARTH_INCLUDE_DIR, where to find the source headers

# to use this module, set variables to point to the osg build
# directory, and source directory, respectively
# OSGEARTHDIR or OSGEARTH_SOURCE_DIR: osg source directory, typically OpenSceneGraph
# OSGEARTH_DIR or OSGEARTH_BUILD_DIR: osg build directory, place in which you've
#    built osg via cmake

# Header files are presumed to be included like
# #include <osg/PositionAttitudeTransform>
# #include <osgUtil/SceneView>

###### headers ######

MACRO( FIND_OSGEARTH_INCLUDE THIS_OSGEARTH_INCLUDE_DIR THIS_OSGEARTH_INCLUDE_FILE )

    FIND_PATH( ${THIS_OSGEARTH_INCLUDE_DIR} ${THIS_OSGEARTH_INCLUDE_FILE}
        PATHS
            ${OSGEARTH_DIR}
            $ENV{OSGEARTH_SOURCE_DIR}
            $ENV{OSGEARTHDIR}
            $ENV{OSGEARTH_DIR}
            /usr/local/
            /usr/
            /sw/ # Fink
            /opt/local/ # DarwinPorts
            /opt/csw/ # Blastwave
            /opt/
            [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSGEARTH_ROOT]/
            ~/Library/Frameworks
            /Library/Frameworks
        PATH_SUFFIXES
            /include/
    )

ENDMACRO( FIND_OSGEARTH_INCLUDE THIS_OSGEARTH_INCLUDE_DIR THIS_OSGEARTH_INCLUDE_FILE )

FIND_OSGEARTH_INCLUDE( OSGEARTH_INCLUDE_DIR osgEarth/Version )

###### libraries ######

MACRO( FIND_OSGEARTH_LIBRARY MYLIBRARY MYLIBRARYNAME )
    
    FIND_LIBRARY(${MYLIBRARY}
        NAMES
            ${MYLIBRARYNAME}
        PATHS
            ${OSGEARTH_DIR}
            $ENV{OSGEARTH_BUILD_DIR}
            $ENV{OSGEARTH_DIR}
            $ENV{OSGEARTHDIR}
            $ENV{OSG_ROOT}
            ~/Library/Frameworks
            /Library/Frameworks
            /usr/local
            /usr
            /sw
            /opt/local
            /opt/csw
            /opt
            [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSGEARTH_ROOT]/lib
            /usr/freeware
        PATH_SUFFIXES
            /lib/
            /lib64/
            /build/lib/
            /build/lib64/
            /Build/lib/
            /Build/lib64/
         )
     
    # set a "HAVE_MYLIBRARYNAME" DEFINITION
    if(MYLIBRARY)
        string(TOUPPER MYLIBRARYNAME TEMP)
        add_definitions(-DHAVE_${TEMP})
    endif()

ENDMACRO(FIND_OSGEARTH_LIBRARY LIBRARY LIBRARYNAME)

FIND_OSGEARTH_LIBRARY( OSGEARTH_LIBRARY osgEarth)

# finds the procedural library and sets "HAVE_OSGEARTHPROCEDURAL" if found
FIND_OSGEARTH_LIBRARY( OSGEARTHPROCEDURAL_LIBRARY osgEarthProcedural)

SET( OSGEARTH_FOUND "NO" )
IF( OSGEARTH_LIBRARY AND OSGEARTH_INCLUDE_DIR )
    SET( OSGEARTH_FOUND "YES" )
    SET( OSGEARTH_INCLUDE_DIRS ${OSGEARTH_INCLUDE_DIR})
    GET_FILENAME_COMPONENT( OSGEARTH_LIBRARIES_DIR ${OSGEARTH_LIBRARY} PATH )
ENDIF( OSGEARTH_LIBRARY AND OSGEARTH_INCLUDE_DIR )
