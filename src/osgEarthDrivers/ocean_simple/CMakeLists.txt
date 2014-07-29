
SET(TARGET_SRC ElevationProxyImageLayer.cpp
               SimpleOceanDriver.cpp
               SimpleOceanNode.cpp
)
               
SET(TARGET_H   ElevationProxyImageLayer
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
