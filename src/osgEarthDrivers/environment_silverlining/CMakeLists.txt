SET(TARGET_SRC 
    SilverLiningDriver.cpp
    SilverLiningNode.cpp
    SilverLiningContext.cpp
    SilverLiningSkyDrawable.cpp
)
SET(TARGET_H
    SilverLiningOptions
    SilverLiningNode
    SilverLiningContext
    SilverLiningSkyDrawable
)

INCLUDE_DIRECTORIES( 
    ${SILVERLINING_INCLUDE_DIR}
)

SET(SILVERLINING_LIBRARY_VAR       ${SILVERLINING_LIBRARY_RELEASE})
SET(SILVERLINING_LIBRARY_VAR_DEBUG ${SILVERLINING_LIBRARY_DEBUG})

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    ${SILVERLINING_LIBRARY_VAR}
    osgEarthUtil
)

SETUP_PLUGIN(osgearth_environment_silverlining)


# to install public driver includes:
SET(LIB_NAME environment_silverlining)
SET(LIB_PUBLIC_HEADERS SilverLiningOptions)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
