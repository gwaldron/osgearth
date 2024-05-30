#
# Detail texture generator plugin
#

set(TARGET_GLSL
	Detail.vert.view.glsl
    Detail.frag.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

set(TARGET_IN
    DetailShaders.cpp.in)

configure_shaders(
    DetailShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )
    
add_osgearth_plugin(
    TARGET osgdb_osgearth_detail
    SOURCES
        DetailPlugin.cpp
        DetailExtension.cpp
        DetailTerrainEffect.cpp
        ${SHADERS_CPP}
    PUBLIC_HEADERS
        DetailExtension
        DetailOptions
        DetailTerrainEffect
    HEADERS
        DetailShaders
    SHADERS
        ${TARGET_GLSL}
    TEMPLATES
        ${TARGET_IN} )
        
