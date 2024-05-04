
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

# set(TARGET_SRC
	# BumpMapExtension.cpp
	# BumpMapTerrainEffect.cpp
	# ${SHADERS_CPP} )
	
# set(TARGET_H_PUBLIC
	# BumpMapExtension
	# BumpMapOptions
	# BumpMapTerrainEffect)
	
# set(TARGET_H
	# ${TARGET_H_PUBLIC}
	# BumpMapShaders )

# add_osgearth_library_as_plugin(osgdb_osgearth_bumpmap)

add_osgearth_library_as_plugin(TARGET osgdb_osgearth_bumpmap
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
        
        
