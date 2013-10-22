
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SET(TARGET_SRC
    CustomPagedLOD.cpp
    KeyNodeFactory.cpp
    LODFactorCallback.cpp
    QuadTreeTerrainEngineNode.cpp
    QuadTreeTerrainEngineDriver.cpp
    SerialKeyNodeFactory.cpp
    TerrainNode.cpp
    TileModelCompiler.cpp
    TileNode.cpp
    TileNodeRegistry.cpp
    TileModelFactory.cpp
)

SET(TARGET_H
    Common
    CustomPagedLOD
    DynamicLODScaleCallback
    FileLocationCallback
    KeyNodeFactory
    LODFactorCallback
    QuadTreeTerrainEngineNode
    QuadTreeTerrainEngineOptions
    QuickReleaseGLObjects
    SerialKeyNodeFactory
    TerrainNode
    TileModel
    TileModelCompiler
    TileNode
    TileNodeRegistry
    TileModelFactory
)

SETUP_PLUGIN(osgearth_engine_quadtree)

# to install public driver includes:
SET(LIB_NAME engine_quadtree)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

