PROJECT(OSGEARTH_EXTENSIONS)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )

SET(CMAKE_SHARED_MODULE_PREFIX ${OSGEARTH_PLUGIN_PREFIX})

IF(MSVC80)
  IF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
    SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
  ENDIF(NOT OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
ENDIF(MSVC80)

SET(TARGET_DEFAULT_PREFIX "osgdb_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Extension")

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES 
    osgEarth
)

# Folder name for plugins
SET(OSGEARTH_EXTENSIONS_FOLDER Extensions)

add_subdirectory(billboard)
add_subdirectory(bumpmap)
add_subdirectory(graticule)
add_subdirectory(mapinspector)
add_subdirectory(noise)
add_subdirectory(normalmap)
add_subdirectory(splat)
add_subdirectory(terrainshader)
add_subdirectory(viewpoints)
