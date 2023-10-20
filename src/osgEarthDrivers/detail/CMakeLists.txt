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

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY) 
	
setup_plugin(osgearth_detail)

# to install public driver includes:
set(LIB_NAME detail)
