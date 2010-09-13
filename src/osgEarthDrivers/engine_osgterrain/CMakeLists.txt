
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
    CompositingTerrainTechnique.cpp
    CustomTerrain.cpp
    MultiPassTerrainTechnique.cpp
    MultiTextureTerrainTechnique.cpp
    OSGTerrainEngineNode.cpp    
    OSGTerrainTileLoader.cpp
    OSGTileFactory.cpp
    Plugin.cpp
    TerrainTileEdgeNormalizerUpdateCallback.cpp
)

SET(TARGET_H
    Common
    CompositingTerrainTechnique
    CustomTerrain
    CustomTerrainTechnique
    FileLocationCallback
    MultiPassTerrainTechnique
    MultiTextureTerrainTechnique
    OSGTerrainEngineNode
    OSGTerrainOptions
    OSGTileFactory
    TerrainTileEdgeNormalizerUpdateCallback
)

SETUP_PLUGIN(osgearth_engine_osgterrain)

# to install public driver includes:
SET(LIB_NAME engine_osgterrain)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

