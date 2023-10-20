#---------------------------------------------------
# OSG CMAKE SUPPORT
# (C) by Michael Wagner, mtw@shared-reality.com 2005
# (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
#---------------------------------------------------

PROJECT(OSGEARTH_PLUGINS_MASTER)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )

SET(CMAKE_SHARED_MODULE_PREFIX ${OSGEARTH_PLUGIN_PREFIX})

SET(TARGET_DEFAULT_PREFIX "osgdb_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Plugin")

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgEarth
)

# Folder name for plugins
SET(OSGEARTH_PLUGINS_FOLDER Plugins)

if(BASISU_FOUND)
    add_subdirectory(basis)
endif()
add_subdirectory(bumpmap)
add_subdirectory(cache_filesystem)
add_subdirectory(colorramp)
add_subdirectory(detail)
add_subdirectory(draco)
add_subdirectory(earth)
add_subdirectory(engine_rex)
add_subdirectory(featurefilter_intersect)
add_subdirectory(featurefilter_join)
add_subdirectory(gltf)
add_subdirectory(kml)
add_subdirectory(mapinspector)
add_subdirectory(monitor)
add_subdirectory(script_engine_duktape)
add_subdirectory(sky_gl)
add_subdirectory(sky_simple)
add_subdirectory(template)
add_subdirectory(terrainshader)
add_subdirectory(webp)
add_subdirectory(vdatum_egm2008)
add_subdirectory(vdatum_egm84)
add_subdirectory(vdatum_egm96)
add_subdirectory(viewpoints)
add_subdirectory(zip)

if(OSGEARTH_BUILD_SILVERLINING_NODEKIT)
    add_subdirectory(sky_silverlining)
endif()

if(OSGEARTH_BUILD_LEVELDB_CACHE)
    add_subdirectory(cache_leveldb)
endif()

add_subdirectory(cache_rocksdb)

IF(NOT APPLE AND NOT ANDROID)
    add_subdirectory(fastdxt)
    add_subdirectory(lerc)
ENDIF()
