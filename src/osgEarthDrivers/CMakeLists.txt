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

# Whether to build the legacy MP terrain engine (default is NO)
OPTION(OSGEARTH_BUILD_LEGACY_MP_TERRAIN_ENGINE "Build the legacy MP terrain engine plugin" OFF)

# Folder name for plugins
SET(OSGEARTH_PLUGINS_FOLDER Plugins)

add_subdirectory(basis)
add_subdirectory(bumpmap)
add_subdirectory(cache_filesystem)
add_subdirectory(cache_leveldb)
add_subdirectory(cache_rocksdb)
add_subdirectory(colorramp)
add_subdirectory(detail)
add_subdirectory(earth)
add_subdirectory(engine_rex)
add_subdirectory(featurefilter_intersect)
add_subdirectory(featurefilter_join)
add_subdirectory(kml)
add_subdirectory(mapinspector)
add_subdirectory(monitor)
add_subdirectory(script_engine_duktape)
add_subdirectory(sky_gl)
add_subdirectory(sky_silverlining)
add_subdirectory(sky_simple)
add_subdirectory(template)
add_subdirectory(terrainshader)
add_subdirectory(webp)
add_subdirectory(lerc)
add_subdirectory(vdatum_egm2008)
add_subdirectory(vdatum_egm84)
add_subdirectory(vdatum_egm96)
add_subdirectory(viewpoints)
add_subdirectory(zip)

if(OSGEARTH_BUILD_LEGACY_MP_TERRAIN_ENGINE)
    add_subdirectory(engine_mp)
endif()

IF(NOT OSGEARTH_BUILD_PLATFORM_IPHONE AND NOT ANDROID)
    add_subdirectory(fastdxt)
ENDIF()

# Libraries that require C++11 support
if(CXX11_AVAILABLE)
    add_subdirectory(gltf)
else(CXX11_AVAILABLE)
    message(STATUS "Omitting features that require C++11 support: GLTF")
endif(CXX11_AVAILABLE)
