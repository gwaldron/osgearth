SET(TARGET_SRC 
    SimpleSkyDriver.cpp
    SimpleSkyNode.cpp
)
SET(TARGET_H
    SimpleSkyOptions
	SimpleSkyNode
    SimpleSkyShaders
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil
)

SETUP_PLUGIN(osgearth_sky_simple)


# to install public driver includes:
SET(LIB_NAME sky_simple)
SET(LIB_PUBLIC_HEADERS SimpleSkyOptions)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
