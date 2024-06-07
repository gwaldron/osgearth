#include_directories(${OSG_INCLUDE_DIRS})

#set(CMAKE_SHARED_MODULE_PREFIX ${OSGEARTH_PLUGIN_PREFIX})

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
#SET(TARGET_COMMON_LIBRARIES
#    osgEarth
#)

if(BASISU_FOUND)
    add_subdirectory(basis)
endif()
add_subdirectory(bumpmap)
add_subdirectory(cache_filesystem)
add_subdirectory(colorramp)
add_subdirectory(detail)
#add_subdirectory(draco)
add_subdirectory(earth)
add_subdirectory(engine_rex)
add_subdirectory(featurefilter_intersect)
add_subdirectory(featurefilter_join)
add_subdirectory(gltf)
add_subdirectory(kml)
add_subdirectory(script_engine_duktape)
add_subdirectory(sky_gl)
add_subdirectory(sky_simple)
add_subdirectory(template)
add_subdirectory(terrainshader)
add_subdirectory(vdatum_egm2008)
add_subdirectory(vdatum_egm84)
add_subdirectory(vdatum_egm96)
add_subdirectory(viewpoints)
add_subdirectory(webp)
add_subdirectory(zip)

if(OSGEARTH_HAVE_SILVERLINING_NODEKIT)
    add_subdirectory(sky_silverlining)
endif()

# if(OSGEARTH_BUILD_LEVELDB_CACHE)
    # add_subdirectory(cache_leveldb)
# endif()

add_subdirectory(cache_rocksdb)

IF(NOT APPLE AND NOT ANDROID)
    add_subdirectory(fastdxt)
    add_subdirectory(lerc)
ENDIF()
