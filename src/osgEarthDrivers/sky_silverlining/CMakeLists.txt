IF(SILVERLINING_FOUND)

SET(TARGET_SRC 
    SilverLiningDriver.cpp
)
SET(TARGET_H
)

INCLUDE_DIRECTORIES( 
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil
    osgEarthSilverLining
)

SETUP_PLUGIN(osgearth_sky_silverlining)


# to install public driver includes:
SET(LIB_NAME sky_silverlining)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(SILVERLINING_FOUND)
