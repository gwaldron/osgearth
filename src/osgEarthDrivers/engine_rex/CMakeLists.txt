
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

set(TARGET_GLSL
    RexEngine.vert.glsl
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
    GeometryPool.cpp
    RexTerrainEngineNode.cpp
    RexTerrainEngineDriver.cpp
    LoadTileData.cpp
    MaskGenerator.cpp
    MPTexture.cpp
	SelectionInfo.cpp
    SurfaceNode.cpp
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
    GeometryPool
    Shaders
    RexTerrainEngineNode
    RexTerrainEngineOptions
    LoadTileData
    MaskGenerator
    MPTexture
    RenderBindings
    SurfaceNode
	TileDrawable
    EngineContext
    TileNode
    TileNodeRegistry
    Loader
    Unloader
	SelectionInfo
)

setup_plugin(osgearth_engine_rex)

# to install public driver includes:
SET(LIB_NAME engine_rex)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
