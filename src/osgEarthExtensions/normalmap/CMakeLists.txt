#
# normal mapping plugin
#

set(TARGET_GLSL
    NormalMap.vert.glsl
    NormalMap.frag.glsl)

set(TARGET_IN
    NormalMapShaders.cpp.in)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    NormalMapShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_SRC
	NormalMapPlugin.cpp
	NormalMapExtension.cpp
	NormalMapTerrainEffect.cpp
	${SHADERS_CPP} )
	
set(LIB_PUBLIC_HEADERS
	NormalMapExtension
	NormalMapOptions
	NormalMapTerrainEffect)
	
set(TARGET_H
	${LIB_PUBLIC_HEADERS}
	NormalMapShaders )


set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil)
	
setup_extension(osgearth_normalmap)

# to install public driver includes:
set(LIB_NAME normalmap)

#include(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)
