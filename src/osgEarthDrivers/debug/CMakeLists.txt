SET(TARGET_SRC DebugTileSource.cpp)
SET(TARGET_H DebugOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)
SETUP_PLUGIN(osgearth_debug)

# to install public driver includes:
SET(LIB_NAME debug)
SET(LIB_PUBLIC_HEADERS DebugOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

