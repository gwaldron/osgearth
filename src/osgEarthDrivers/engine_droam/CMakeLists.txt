
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
    CubeManifold.cpp
    Diamond.cpp
    DRoamNode.cpp
    GeodeticManifold.cpp
    Manifold.cpp
    MeshManager.cpp
    Plugin.cpp
    AMRGeometry.cpp
)

SET(TARGET_H
    Common
    CubeManifold
    Diamond
    DRoamNode
    GeodeticManifold
    Manifold
    MeshManager
    AMRGeometry
    AMRShaders.h
)

SETUP_PLUGIN(osgearth_engine_droam)

# to install public driver includes:
SET(LIB_NAME engine_droam)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

