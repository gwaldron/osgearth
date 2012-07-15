
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SET(TARGET_SRC
    KeyNodeFactory.cpp
    LODFactorCallback.cpp
    QuadTreeTerrainEngineNode.cpp
    QuadTreeTerrainEngineDriver.cpp
    SerialKeyNodeFactory.cpp
    TerrainNode.cpp
    TileModelCompiler.cpp
    TileNode.cpp
    TileModelFactory.cpp
)

SET(TARGET_H
    Common
    DynamicLODScaleCallback
    FileLocationCallback
    KeyNodeFactory
    LODFactorCallback
    QuadTreeTerrainEngineNode
    QuadTreeTerrainEngineOptions
    SerialKeyNodeFactory
    TerrainNode
    TileModel
    TileModelCompiler
    TileNode
    TileModelFactory
)

SETUP_PLUGIN(osgearth_engine_quadtree)

# to install public driver includes:
SET(LIB_NAME engine_quadtree)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

