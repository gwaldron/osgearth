#---------------------------------------------------
# OSG CMAKE SUPPORT 
# (C) by Michael Wagner, mtw@shared-reality.com 2005
# (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
#---------------------------------------------------

PROJECT(OSGEARTH_PLUGINS_MASTER)


SET(CMAKE_SHARED_MODULE_PREFIX ${OSGEARTH_PLUGIN_PREFIX})

IF(MSVC80)
  IF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
    SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
  ENDIF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
ENDIF(MSVC80)

SET(TARGET_DEFAULT_PREFIX "osgdb_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Plugins")

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES 
    osgEarth
)

############################################################
#
#  NodeKit/Psudo loader plugins
#
ADD_SUBDIRECTORY(earth)

#ADD_SUBDIRECTORY(google)
ADD_SUBDIRECTORY(wcs)
ADD_SUBDIRECTORY(wms)
ADD_SUBDIRECTORY(tilecache)
ADD_SUBDIRECTORY(tileservice)
ADD_SUBDIRECTORY(yahoo)
ADD_SUBDIRECTORY(arcgis_map_cache)
ADD_SUBDIRECTORY(arcgis)
ADD_SUBDIRECTORY(tms)
ADD_SUBDIRECTORY(vpb)
ADD_SUBDIRECTORY(osg)

IF(LIBZIP_FOUND)
  ADD_SUBDIRECTORY(zipfs)
ENDIF(LIBZIP_FOUND)

IF(GDAL_FOUND)
  ADD_SUBDIRECTORY(gdal)
  ADD_SUBDIRECTORY(feature_ogr)
  ADD_SUBDIRECTORY(model_feature_overlay)
  ADD_SUBDIRECTORY(model_feature_stencil)
  ADD_SUBDIRECTORY(model_feature_geom)
ENDIF(GDAL_FOUND)

    
