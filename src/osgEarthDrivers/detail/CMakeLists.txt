#
# Detail texture generator plugin
#

set(TARGET_GLSL
	Detail.vert.view.glsl
    Detail.frag.glsl
    )

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    DetailShaders.cpp.in)

configure_shaders(
    DetailShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_SRC
	DetailPlugin.cpp
	DetailExtension.cpp
	DetailTerrainEffect.cpp
	${SHADERS_CPP} )
	
set(LIB_PUBLIC_HEADERS
	DetailExtension
	DetailOptions
	DetailTerrainEffect)
	
set(TARGET_H
	${LIB_PUBLIC_HEADERS}
	DetailShaders
	)

set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil)
	
setup_plugin(osgearth_detail)

# to install public driver includes:
set(LIB_NAME detail)
