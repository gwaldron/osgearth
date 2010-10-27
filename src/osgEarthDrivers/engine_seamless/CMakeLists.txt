SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
        QSC.cpp
        Euler.cpp
        Patch.cpp
        PatchGroup.cpp
        PatchSet.cpp
        Projected.cpp
        Geographic.cpp
        GeoPatch.cpp
        SeamlessEngineNode.cpp
        SeamlessPlugin.cpp
)

SET(TARGET_H
        QSC
        Euler
        Patch
        PatchInfo
        PatchGroup
        PatchSet
        Projected
        Geographic
        GeoPatch
        SeamlessEngineNode
        SeamlessOptions
)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR})

# to install public driver includes:
SETUP_PLUGIN(osgearth_engine_seamless)

SET(LIB_NAME engine_seamless)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
