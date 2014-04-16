
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SET(TARGET_SRC
    KeyNodeFactory.cpp
    MPGeometry.cpp
    MPTerrainEngineNode.cpp
    MPTerrainEngineDriver.cpp
    SingleKeyNodeFactory.cpp
    TerrainNode.cpp
    TileGroup.cpp
    TileModel.cpp
    TileModelCompiler.cpp
    TileNode.cpp
    TileNodeRegistry.cpp
    TileModelFactory.cpp
    TilePagedLOD.cpp
)

SET(TARGET_H
    Common
    DynamicLODScaleCallback
    FileLocationCallback
    KeyNodeFactory
    MPGeometry
    MPTerrainEngineNode
    MPTerrainEngineOptions
    QuickReleaseGLObjects
    SingleKeyNodeFactory
    TerrainNode
    TileGroup
    TileModel
    TileModelCompiler
    TileNode
    TileNodeRegistry
    TileModelFactory
    TilePagedLOD
)

SETUP_PLUGIN(osgearth_engine_mp)

# to install public driver includes:
SET(LIB_NAME engine_mp)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

