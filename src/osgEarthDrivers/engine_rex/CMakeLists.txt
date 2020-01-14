
set(TARGET_GLSL
    RexEngine.vert.glsl
    RexEngine.elevation.glsl
    RexEngine.vert.view.glsl
    RexEngine.tcs.glsl
    RexEngine.tes.glsl
    RexEngine.gs.glsl
    RexEngine.frag.glsl
    RexEngine.NormalMap.vert.glsl
    RexEngine.NormalMap.frag.glsl
    RexEngine.Morphing.vert.glsl
    RexEngine.SDK.vert.glsl)

set(TARGET_IN
    Shaders.cpp.in)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(TARGET_SRC
    CreateTileImplementation.cpp
    DrawState.cpp
    DrawTileCommand.cpp
    GeometryPool.cpp
    RexTerrainEngineNode.cpp
    RexTerrainEngineDriver.cpp
    LayerDrawable.cpp
    LoadTileData.cpp
    MaskGenerator.cpp
	SelectionInfo.cpp
    SurfaceNode.cpp
    TerrainCuller.cpp
    TerrainRenderData.cpp
	TileDrawable.cpp
    EngineContext.cpp
    TileNode.cpp
    TileNodeRegistry.cpp
    Loader.cpp
    Unloader.cpp
    ${SHADERS_CPP}
)

SET(TARGET_H
    Common
    CreateTileImplementation
    DrawState
    DrawTileCommand
    GeometryPool
    Shaders
    RexTerrainEngineNode
    LayerDrawable
    LoadTileData
    MaskGenerator
    RenderBindings
    SurfaceNode
    TerrainCuller
    TerrainRenderData
	TileDrawable
    TileRenderModel
    EngineContext
    TileNode
    TileNodeRegistry
    Loader
    Unloader
	SelectionInfo
)

IF(TRACY_FOUND)
    INCLUDE_DIRECTORIES(${TRACY_INCLUDE_DIR})
    LIST(APPEND TARGET_LIBRARIES_VARS TRACY_LIBRARY )
ENDIF(TRACY_FOUND)

setup_plugin(osgearth_engine_rex)

# to install public driver includes:
SET(LIB_NAME engine_rex)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
