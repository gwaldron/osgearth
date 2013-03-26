
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SET(TARGET_SRC
    BYOTerrainEngineNode.cpp
    BYOTerrainEngineDriver.cpp
)

SET(TARGET_H
    Common
    BYOTerrainEngineNode
    BYOTerrainEngineOptions
)

SETUP_PLUGIN(osgearth_engine_byo)

# to install public driver includes:
SET(LIB_NAME engine_byo)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
