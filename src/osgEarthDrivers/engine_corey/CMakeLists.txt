
set(TARGET_GLSL
    Corey.vert.glsl
    Corey.elevation.glsl
    Corey.gs.glsl
    Corey.ImageLayer.glsl
    Corey.NormalMap.glsl
    Corey.Morphing.glsl
    Corey.Tessellation.glsl
    Corey.SDK.glsl    
    Corey.vert.GL4.glsl
    Corey.ImageLayer.GL4.glsl
    Corey.NormalMap.GL4.glsl
    Corey.Tessellation.GL4.glsl
    Corey.SDK.GL4.glsl
    Corey.GL4.glsl)

set(TARGET_IN
    Shaders.cpp.in)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    Shaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGTEXT_LIBRARY OPENTHREADS_LIBRARY OPENGL_gl_LIBRARY) 

SET(TARGET_SRC
    CreateTileImplementation.cpp
    DrawState.cpp
    DrawTileCommand.cpp
    CoreyTerrainEngineNode.cpp
    CoreyTerrainEngineDriver.cpp
    LayerDrawable.cpp
	SelectionInfo.cpp
    TerrainCuller.cpp
    TerrainRenderData.cpp
    TileGeometry.cpp
    TileNode.cpp
    ${SHADERS_CPP}
)

SET(TARGET_H
    Common
    CoreyTerrainEngineNode
    CreateTileImplementation
    DrawState
    DrawTileCommand
    EngineData
    LayerDrawable
    RenderBindings
	SelectionInfo
    Shaders
    TerrainCuller
    TerrainRenderData
    TileRenderModel
    TileGeometry
    TileNode
)

IF(TRACY_FOUND)
    INCLUDE_DIRECTORIES(${TRACY_INCLUDE_DIR})
    LIST(APPEND TARGET_LIBRARIES_VARS TRACY_LIBRARY )
ENDIF(TRACY_FOUND)

setup_plugin(osgearth_engine_corey)

# to install public driver includes:
SET(LIB_NAME engine_corey)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
