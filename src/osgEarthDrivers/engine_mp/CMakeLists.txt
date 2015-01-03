
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

set(TARGET_GLSL
    MPEngine.vert.glsl
    MPEngine.vert.pooled.glsl
    MPEngine.frag.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    MPShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(TARGET_SRC
    GeometryPool.cpp
    MPGeometry.cpp
    MPTerrainEngineNode.cpp
    MPTerrainEngineDriver.cpp
    MaskGenerator.cpp
    SurfaceNodeFactory.cpp
    TerrainNode.cpp
	TileDrawable.cpp
    TileGroup.cpp
    TileGroupFactory.cpp
    TileNode.cpp
    TileNodeRegistry.cpp
    TilePagedLOD.cpp
    ${SHADERS_CPP}
)

SET(TARGET_H
    Common
    FileLocationCallback
    GeometryPool
    MPGeometry
    MPShaders
    MPTerrainEngineNode
    MPTerrainEngineOptions
    MaskGenerator
    QuickReleaseGLObjects
    RenderBindings
    SurfaceNode
    TerrainNode
	TileDrawable
    TileGroup
    TileGroupFactory
    TileNode
    SurfaceNodeFactory
    TileNodeRegistry
    TilePagedLOD
)

setup_plugin(osgearth_engine_mp)

# to install public driver includes:
SET(LIB_NAME engine_mp)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

