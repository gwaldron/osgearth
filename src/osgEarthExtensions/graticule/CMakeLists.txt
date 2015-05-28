#
# graticule extension
#

IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_GRATICULE_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_GRATICULE_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)


set(TARGET_GLSL
    Graticule.vert.glsl
    Graticule.frag.glsl)

set(TARGET_IN
    GraticuleShaders.cpp.in)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(LIB_NAME graticule)

configure_shaders(
    GraticuleShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_SRC
	GraticulePlugin.cpp
	GraticuleExtension.cpp
	GraticuleTerrainEffect.cpp
	GraticuleNode.cpp
	${SHADERS_CPP} )
	
set(LIB_PUBLIC_HEADERS
	GraticuleExtension
	GraticuleNode
	GraticuleOptions
	GraticuleTerrainEffect
	Export
	)
	
set(TARGET_H
	${LIB_PUBLIC_HEADERS}
	GraticuleShaders )


set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthAnnotation osgEarthUtil)
	
setup_extension(osgearth_graticule)

#include(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)
