
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
    Diamond.cpp
    Manifold.cpp
    CubeManifold.cpp
    DRoamNode.cpp
    MeshManager.cpp
    Plugin.cpp)

SET(TARGET_H
    Common
    Diamond
    DRoamNode
    MeshManager
    Manifold
    CubeManifold
)

SETUP_PLUGIN(osgearth_engine_droam)

# to install public driver includes:
SET(LIB_NAME engine_droam)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

