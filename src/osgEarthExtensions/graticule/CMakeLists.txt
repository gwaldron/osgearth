#
# graticule extension
#

set(TARGET_GLSL
    Graticule.vert.glsl
    Graticule.frag.glsl)

set(TARGET_IN
    GraticuleShaders.cpp.in)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    GraticuleShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_SRC
	GraticulePlugin.cpp
	GraticuleExtension.cpp
	GraticuleTerrainEffect.cpp
	${SHADERS_CPP} )
	
set(LIB_PUBLIC_HEADERS
	GraticuleExtension
	GraticuleOptions
	GraticuleTerrainEffect)
	
set(TARGET_H
	${LIB_PUBLIC_HEADERS}
	GraticuleShaders )


set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthAnnotation)
	
setup_extension(osgearth_graticule)

# to install public driver includes:
set(LIB_NAME graticule)

#include(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)
