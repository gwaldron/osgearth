#
# SimpleSky plugin
#

set(TARGET_GLSL
    SimpleSky.Atmosphere.frag.glsl
    SimpleSky.Atmosphere.vert.glsl
    SimpleSky.Ground.ONeil.frag.glsl
    SimpleSky.Ground.ONeil.vert.glsl
    SimpleSky.Moon.frag.glsl
    SimpleSky.Moon.vert.glsl
    SimpleSky.Stars.frag.glsl
    SimpleSky.Stars.vert.glsl
    SimpleSky.Stars.GLES.frag.glsl
    SimpleSky.Stars.GLES.vert.glsl
    SimpleSky.Sun.frag.glsl
    SimpleSky.Sun.vert.glsl )

set(TARGET_IN
    SimpleSkyShaders.cpp.in)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    SimpleSkyShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )
    
set(TARGET_SRC 
    SimpleSkyExtension.cpp
    SimpleSkyNode.cpp
    ${SHADERS_CPP} )

set(TARGET_H
    SimpleSkyOptions
	SimpleSkyNode
    SimpleSkyShaders )

set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil )

setup_plugin(osgearth_sky_simple)


# to install public driver includes:
set(LIB_NAME sky_simple)
set(LIB_PUBLIC_HEADERS SimpleSkyOptions)
include(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
