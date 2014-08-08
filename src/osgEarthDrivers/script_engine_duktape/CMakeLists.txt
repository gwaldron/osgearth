
IF(WITH_EXTERNAL_DUKTAPE)

    INCLUDE_DIRECTORIES( ${DUKTAPE_INCLUDE_DIR} )

    SET(TARGET_H
        DuktapeEngine
		JSGeometry
    )

    SET(TARGET_SRC
        Plugin.cpp
        DuktapeEngine.cpp
    )

    SET(TARGET_COMMON_LIBRARIES ${DUKTAPE_LIBRARY} osgEarthFeatures osgEarthSymbology)

ELSE(WITH_EXTERNAL_DUKTAPE)

    SET(TARGET_H
        duktape.h
        DuktapeEngine
		JSGeometry
    )

    SET(TARGET_SRC
        Plugin.cpp
        duktape.c
        DuktapeEngine.cpp
    )

    SET(TARGET_COMMON_LIBRARIES osgEarthFeatures osgEarthSymbology)

ENDIF(WITH_EXTERNAL_DUKTAPE)

SETUP_PLUGIN(osgearth_scriptengine_javascript)

# to install public driver includes:
SET(LIB_NAME scriptengine_javascript)
SET(LIB_PUBLIC_HEADERS ${TARGET_H} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

