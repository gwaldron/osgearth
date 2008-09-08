#---------------------------------------------------
# OSG CMAKE SUPPORT 
# (C) by Michael Wagner, mtw@shared-reality.com 2005
# (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
#---------------------------------------------------

PROJECT(OSGEARTH_PLUGINS_MASTER)

IF(NOT MSVC)
    SET(LIBRARY_OUTPUT_PATH "${LIBRARY_OUTPUT_PATH}/${OSGEARTH_PLUGINS}")
    SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${OSGEARTH_PLUGINS}")
ENDIF(NOT MSVC)

SET(CMAKE_SHARED_MODULE_PREFIX ${OSGEARTH_PLUGIN_PREFIX})

IF(MSVC80)
  IF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
    SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
  ENDIF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
ENDIF(MSVC80)

SET(TARGET_DEFAULT_PREFIX "osgdb_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Plugins")
SET(TARGET_COMMON_LIBRARIES 
    OpenThreads
    osg 
    osgDB 
    osgUtil
    osgEarth
)

############################################################
#
#  NodeKit/Psudo loader plugins
#
ADD_SUBDIRECTORY(earth)
ADD_SUBDIRECTORY(msve)
ADD_SUBDIRECTORY(wcs)
ADD_SUBDIRECTORY(wms)


    
