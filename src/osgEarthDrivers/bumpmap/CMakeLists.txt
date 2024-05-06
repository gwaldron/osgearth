
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

add_osgearth_plugin(TARGET osgdb_osgearth_bumpmap
    PUBLIC_HEADERS
        BumpMapExtension
        BumpMapOptions
        BumpMapTerrainEffect
    HEADERS
        BumpMapShaders
    SOURCES 
        BumpMapExtension.cpp
        BumpMapTerrainEffect.cpp
        ${SHADERS_CPP}
    TEMPLATES
        ${TARGET_IN}
    SHADERS
        ${TARGET_GLSL} )
        
