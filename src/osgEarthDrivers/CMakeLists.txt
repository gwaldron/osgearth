#---------------------------------------------------
# OSG CMAKE SUPPORT
# (C) by Michael Wagner, mtw@shared-reality.com 2005
# (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
#---------------------------------------------------

PROJECT(OSGEARTH_PLUGINS_MASTER)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )

SET(CMAKE_SHARED_MODULE_PREFIX ${OSGEARTH_PLUGIN_PREFIX})

IF(MSVC80)
  IF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
    SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
  ENDIF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
ENDIF(MSVC80)

SET(TARGET_DEFAULT_PREFIX "osgdb_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Plugin")

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgEarth
)

# Folder name for plugins
SET(OSGEARTH_PLUGINS_FOLDER Plugins)

############################################################
#
#  NodeKit/Psudo loader plugins
#


SUBDIRLIST(PLUGIN_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

FOREACH(subdir ${PLUGIN_DIRS})
    # MESSAGE("Adding driver ${subdir}")
    ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()
