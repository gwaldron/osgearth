
set(TARGET_GLSL
    SimpleOcean.VS.glsl
    SimpleOcean.FS.glsl)

set(TARGET_IN
    Shaders.cpp.in)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(TARGET_SRC
    ElevationProxyImageLayer.cpp
    SimpleOceanDriver.cpp
    SimpleOceanNode.cpp
    ${SHADERS_CPP}
)
               
SET(TARGET_H   
    ElevationProxyImageLayer
    SimpleOceanOptions
    SimpleOceanNode
    SimpleOceanShaders
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil
    osgEarthSymbology
)

SETUP_PLUGIN(osgearth_ocean_simple)

# to install public driver includes:
SET(LIB_NAME ocean_simple)

SET(LIB_PUBLIC_HEADERS SimpleOceanOptions)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
