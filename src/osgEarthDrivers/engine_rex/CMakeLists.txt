
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

set(TARGET_GLSL
    RexEngine.vert.glsl
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
    MaskGenerator.cpp
    SurfaceNodeFactory.cpp
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
    Shaders
    RexTerrainEngineNode
    RexTerrainEngineOptions
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
    TilePagedLOD
)

setup_plugin(osgearth_engine_rex)

# to install public driver includes:
SET(LIB_NAME engine_rex)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
