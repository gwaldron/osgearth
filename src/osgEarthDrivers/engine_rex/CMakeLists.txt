
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

set(TARGET_GLSL
    RexEngine.vert.glsl
    RexEngine.vert.view.glsl
    RexEngine.tcs.glsl
    RexEngine.tes.glsl
    RexEngine.gs.glsl
    RexEngine.frag.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(TARGET_SRC
    GeometryPool.cpp
    RexTerrainEngineNode.cpp
    RexTerrainEngineDriver.cpp
    LoadTileData.cpp
    ExpireTiles.cpp
    MaskGenerator.cpp
    SurfaceNode.cpp
    SurfaceNodeFactory.cpp
	TileDrawable.cpp
    TileGroup.cpp
    TileGroupFactory.cpp
    TileNode.cpp
    TileNodeRegistry.cpp
    Loader.cpp
    ${SHADERS_CPP}
)

SET(TARGET_H
    Common
    FileLocationCallback
    GeometryPool
    Shaders
    RexTerrainEngineNode
    RexTerrainEngineOptions
    LoadTileData
    ExpireTiles
    MaskGenerator
    QuickReleaseGLObjects
    RenderBindings
    SurfaceNode
	TileDrawable
    TileGroup
    TileGroupFactory
    TileNode
    SurfaceNodeFactory
    TileNodeRegistry
    Loader
)

setup_plugin(osgearth_engine_rex)

# to install public driver includes:
SET(LIB_NAME engine_rex)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
