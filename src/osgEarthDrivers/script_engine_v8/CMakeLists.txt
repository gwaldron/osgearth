
INCLUDE_DIRECTORIES( ${V8_INCLUDE_DIR} )

SET(TARGET_SRC
    JavascriptEngineV8Factory.cpp
    JavascriptEngineV8.cpp
    JSWrappers.cpp
)

SET(TARGET_H
    JavascriptEngineV8
    JSWrappers
    V8Util
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)

SET(TARGET_LIBRARIES_VARS ${TARGET_LIBRARIES_VARS} V8_BASE_LIBRARY V8_SNAPSHOT_LIBRARY V8_ICUUC_LIBRARY V8_ICUI18N_LIBRARY)

SETUP_PLUGIN(osgearth_scriptengine_javascript)

# to install public driver includes:
SET(LIB_NAME scriptengine_javascript)
SET(LIB_PUBLIC_HEADERS ${TARGET_H} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

