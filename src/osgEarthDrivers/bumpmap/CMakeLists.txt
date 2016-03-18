#
# bump mapping plugin
#

set(TARGET_GLSL
	BumpMap.vert.view.glsl
    BumpMap.frag.simple.glsl
    BumpMap.frag.progressive.glsl
    BumpMap.frag.common.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    BumpMapShaders.cpp.in)

configure_shaders(
    BumpMapShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_SRC
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
	
setup_plugin(osgearth_bumpmap)

# to install public driver includes:
set(LIB_NAME bumpmap)

