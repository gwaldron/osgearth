
INCLUDE_DIRECTORIES( ${DUKTAPE_INCLUDE_DIR} )

SET(TARGET_SRC
    Plugin.cpp
    DuktapeEngine.cpp
    JSWrappers.cpp
)

SET(TARGET_H
    DuktapeEngine
    JSWrappers
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} ${DUKTAPE_LIBRARY} osgEarthFeatures osgEarthSymbology)

SETUP_PLUGIN(osgearth_scriptengine_duktape)

# to install public driver includes:
SET(LIB_NAME scriptengine_duktape)
SET(LIB_PUBLIC_HEADERS ${TARGET_H} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

