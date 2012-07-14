
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SET(TARGET_SRC
    KeyNodeFactory.cpp
    LODFactorCallback.cpp
    MultiPassTerrainTechnique.cpp
    QuadTreeTerrainEngineNode.cpp
    QuadTreeTerrainEngineDriver.cpp
    SerialKeyNodeFactory.cpp
    SinglePassTerrainTechnique.cpp
    TerrainNode.cpp
    Tile.cpp
    TileBuilder.cpp
)

SET(TARGET_H
    Common
    CustomTerrainTechnique
    DynamicLODScaleCallback   
    FileLocationCallback
    KeyNodeFactory
    LODFactorCallback
    MultiPassTerrainTechnique
    QuadTreeTerrainEngineNode
    QuadTreeTerrainEngineOptions
    SerialKeyNodeFactory
    SinglePassTerrainTechnique
    TerrainNode
    Tile
    TileBuilder
    TransparentLayer
)

SETUP_PLUGIN(osgearth_engine_quadtree)

# to install public driver includes:
SET(LIB_NAME engine_quadtree)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

