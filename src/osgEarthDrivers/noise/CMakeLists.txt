
SET(TARGET_SRC NoiseDriver.cpp)
SET(TARGET_H   NoiseOptions)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil
)

SETUP_PLUGIN(osgearth_noise)

# to install public driver includes:
SET(LIB_NAME noise)
SET(LIB_PUBLIC_HEADERS NoiseOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
