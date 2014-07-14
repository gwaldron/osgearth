
INCLUDE_DIRECTORIES( ${DUKTAPE_INCLUDE_DIR} )

SET(TARGET_H
    duktape.h
    DuktapeEngine
)

SET(TARGET_SRC
    Plugin.cpp
    duktape.c
    DuktapeEngine.cpp
)

SET(TARGET_COMMON_LIBRARIES osgEarthFeatures osgEarthSymbology)

SETUP_PLUGIN(osgearth_scriptengine_javascript)

# to install public driver includes:
SET(LIB_NAME scriptengine_javascript)
SET(LIB_PUBLIC_HEADERS ${TARGET_H} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

