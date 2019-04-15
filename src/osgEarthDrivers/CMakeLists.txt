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

add_subdirectory(bumpmap)
add_subdirectory(cache_filesystem)
add_subdirectory(cache_leveldb)
add_subdirectory(cache_rocksdb)
add_subdirectory(colorramp)
add_subdirectory(detail)
add_subdirectory(earth)
add_subdirectory(engine_mp)
add_subdirectory(engine_rex)
IF(NOT OSGEARTH_BUILD_PLATFORM_IPHONE AND NOT ANDROID)
add_subdirectory(fastdxt)
ENDIF()
add_subdirectory(featurefilter_intersect)
add_subdirectory(featurefilter_join)
add_subdirectory(feature_elevation)
add_subdirectory(kml)
add_subdirectory(label_annotation)
add_subdirectory(mapinspector)
add_subdirectory(model_simple)
add_subdirectory(monitor)
add_subdirectory(script_engine_duktape)
add_subdirectory(sky_gl)
add_subdirectory(sky_silverlining)
add_subdirectory(sky_simple)
add_subdirectory(template)
add_subdirectory(terrainshader)
add_subdirectory(vdatum_egm2008)
add_subdirectory(vdatum_egm84)
add_subdirectory(vdatum_egm96)
add_subdirectory(viewpoints)

# Libraries that require C++11 support
if(CXX11_AVAILABLE)
    add_subdirectory(gltf)
else(CXX11_AVAILABLE)
    message(STATUS "Omitting features that require C++11 support: GLTF")
endif(CXX11_AVAILABLE)
