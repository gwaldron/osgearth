SET(TARGET_SRC 
    SilverLiningDriver.cpp
    SilverLiningNode.cpp
    SilverLiningContext.cpp
    SilverLiningSkyDrawable.cpp
    SilverLiningCloudsDrawable.cpp
)
SET(TARGET_H
    SilverLiningOptions
    SilverLiningNode
    SilverLiningContext
    SilverLiningSkyDrawable
    SilverLiningCloudsDrawable
)

INCLUDE_DIRECTORIES( 
    ${SILVERLINING_INCLUDE_DIR}
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
	${SILVERLINING_LIBRARY}
    osgEarthUtil
)

SETUP_PLUGIN(osgearth_sky_silverlining)


# to install public driver includes:
SET(LIB_NAME sky_silverlining)
SET(LIB_PUBLIC_HEADERS SilverLiningOptions)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
