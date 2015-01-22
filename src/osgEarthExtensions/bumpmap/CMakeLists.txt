#
# bump mapping plugin
#

set(TARGET_GLSL
    BumpMap.model.vert.glsl
	BumpMap.view.vert.glsl
    BumpMap.simple.frag.glsl
    BumpMap.progressive.frag.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    BumpMapShaders.cpp.in)

configure_shaders(
    BumpMapShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_SRC
	BumpMapPlugin.cpp
	BumpMapExtension.cpp
	BumpMapTerrainEffect.cpp
	${SHADERS_CPP} )
	
set(LIB_PUBLIC_HEADERS
	BumpMapExtension
	BumpMapOptions
	BumpMapTerrainEffect)
	
set(TARGET_H
	${LIB_PUBLIC_HEADERS}
	BumpMapShaders )


set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil)
	
setup_extension(osgearth_bumpmap)

# to install public driver includes:
set(LIB_NAME bumpmap)

#include(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)
