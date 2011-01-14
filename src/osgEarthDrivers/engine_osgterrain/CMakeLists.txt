
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
    CustomTerrain.cpp
    CustomTile.cpp
    MultiPassTerrainTechnique.cpp
    OSGTerrainEngineNode.cpp    
    OSGTileFactory.cpp
    QuadTileNode.cpp
    ParallelKeyNodeFactory.cpp
    Plugin.cpp
    SerialKeyNodeFactory.cpp
    SinglePassTerrainTechnique.cpp
    TerrainTileEdgeNormalizerUpdateCallback.cpp
    TileDataLoader.cpp
    TileBuilder.cpp
)

SET(TARGET_H
    Common
    CustomTerrain
    CustomTile
    CustomTerrainTechnique
    FileLocationCallback
    KeyNodeFactory
    MultiPassTerrainTechnique
    ParallelKeyNodeFactory
    OSGTerrainEngineNode
    OSGTerrainOptions
    OSGTileFactory
    QuadTileNode
    SerialKeyNodeFactory
    SinglePassTerrainTechnique
    TerrainTileEdgeNormalizerUpdateCallback
    TileDataLoader
    TileBuilder
    TransparentLayer
)

SETUP_PLUGIN(osgearth_engine_osgterrain)

# to install public driver includes:
SET(LIB_NAME engine_osgterrain)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

