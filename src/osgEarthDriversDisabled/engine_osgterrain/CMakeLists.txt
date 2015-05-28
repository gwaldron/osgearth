
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SET(TARGET_SRC
    KeyNodeFactory.cpp
    LODFactorCallback.cpp
    MultiPassTerrainTechnique.cpp
    OSGTerrainEngineNode.cpp    
    OSGTileFactory.cpp
    ParallelKeyNodeFactory.cpp
    Plugin.cpp
    SerialKeyNodeFactory.cpp
    SinglePassTerrainTechnique.cpp    
    StreamingTerrainNode.cpp
    StreamingTile.cpp
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
    ParallelKeyNodeFactory
    OSGTerrainEngineNode
    OSGTerrainOptions
    OSGTileFactory
    SerialKeyNodeFactory
    SinglePassTerrainTechnique
    StreamingTerrainNode
    StreamingTile
    TerrainNode
    Tile
    TileBuilder
    TransparentLayer
)

SETUP_PLUGIN(osgearth_engine_osgterrain)

# to install public driver includes:
SET(LIB_NAME engine_osgterrain)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

