
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
    CustomTerrain.cpp
    CustomTile.cpp
    MultiPassTerrainTechnique.cpp
    OSGTerrainEngineNode.cpp    
    OSGTileFactory.cpp
    QuadTileNode.cpp
    Plugin.cpp
    TerrainTileEdgeNormalizerUpdateCallback.cpp
    TileDataLoader.cpp
    SinglePassTerrainTechnique.cpp
)

SET(TARGET_H
    Common
    CustomTerrain
    CustomTile
    CustomTerrainTechnique
    FileLocationCallback
    MultiPassTerrainTechnique
    OSGTerrainEngineNode
    OSGTerrainOptions
    OSGTileFactory
    QuadTileNode
    TerrainTileEdgeNormalizerUpdateCallback
    TileDataLoader
    TransparentLayer
    SinglePassTerrainTechnique
)

SETUP_PLUGIN(osgearth_engine_osgterrain)

# to install public driver includes:
SET(LIB_NAME engine_osgterrain)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

