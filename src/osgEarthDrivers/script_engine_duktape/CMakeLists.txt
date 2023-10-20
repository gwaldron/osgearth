IF(NOT WITH_EXTERNAL_DUKTAPE OR DUKTAPE_FOUND)

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

ELSE(WITH_EXTERNAL_DUKTAPE)

    SET(TARGET_H
        duktape.h
        duk_config.h
        DuktapeEngine
		JSGeometry
    )

    SET(TARGET_SRC
        Plugin.cpp
        duktape.c
        DuktapeEngine.cpp
    )

ENDIF(WITH_EXTERNAL_DUKTAPE)

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY) 

SETUP_PLUGIN(osgearth_scriptengine_javascript)

# to install public driver includes:
SET(LIB_NAME scriptengine_javascript)
SET(LIB_PUBLIC_HEADERS ${TARGET_H} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(NOT WITH_EXTERNAL_DUKTAPE OR DUKTAPE_FOUND)
