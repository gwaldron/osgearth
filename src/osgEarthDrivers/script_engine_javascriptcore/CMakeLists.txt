
INCLUDE_DIRECTORIES( ${JAVASCRIPTCORE_INCLUDE_DIR} )

SET(TARGET_SRC
    JavaScriptCoreEngineFactory.cpp
    JavaScriptCoreEngine.cpp
    JSWrappers.cpp
)

SET(TARGET_H
    JavaScriptCoreEngine
    JSWrappers
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} ${JAVASCRIPTCORE_LIBRARY} osgEarthFeatures)

SETUP_PLUGIN(osgearth_scriptengine_javascriptcore)

# to install public driver includes:
SET(LIB_NAME scriptengine_javascriptcore)
SET(LIB_PUBLIC_HEADERS ${TARGET_H} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

