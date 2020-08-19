#
# osgEarth Procedural NodeKit
#

IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHPROCEDURAL_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTHPROCEDURAL_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

SET(LIB_NAME osgEarthProcedural)

set(TARGET_GLSL
    Splat.types.glsl
    Splat.Noise.glsl
    Splat.vert.model.glsl
    Splat.vert.view.glsl
    Splat.frag.glsl
    Splat.util.glsl
    GroundCover.CS.glsl
    GroundCover.Render.glsl
    GroundCover.Types.glsl
    Grass.glsl
    TextureSplatting.glsl
    TextureSplatting2.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    ProceduralShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_IN    
    ProceduralShaders.cpp.in)

set(TARGET_SRC
    AssetCatalog.cpp
    Biome.cpp
    BiomeLayer.cpp
    GrassLayer.cpp
    #GroundCoverFeatureGenerator.cpp
    #GroundCoverFeatureSource.cpp
    GroundCoverLayer.cpp
    NoiseTextureFactory.cpp
    RoadSurfaceLayer.cpp
    TextureSplattingLayer.cpp
    LifeMapLayer.cpp
    ${SHADERS_CPP})
	
set(LIB_PUBLIC_HEADERS
    AssetCatalog
    Biome
    BiomeLayer
	Export
    GrassLayer
    #GroundCoverFeatureGenerator
    #GroundCoverFeatureSource
    GroundCoverLayer
    NoiseTextureFactory
    RoadSurfaceLayer
    ProceduralShaders
    TextureSplattingLayer
    LifeMapLayer)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	ProceduralShaders)

ADD_LIBRARY(${LIB_NAME} ${OSGEARTH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TARGET_SRC}
    ${TARGET_GLSL}
    ${TARGET_IN}
)

IF (WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ELSE(WIN32)
  LINK_EXTERNAL(${LIB_NAME} ${TARGET_EXTERNAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})
ENDIF(WIN32)

LINK_INTERNAL(${LIB_NAME}
    osgEarth
)

IF (TRACY_FOUND)
    INCLUDE_DIRECTORIES(${TRACY_INCLUDE_DIR})
    LINK_WITH_VARIABLES(${LIB_NAME} TRACY_LIBRARY)
ENDIF(TRACY_FOUND)

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGSIM_LIBRARY OSGDB_LIBRARY OSGVIEWER_LIBRARY OSGTEXT_LIBRARY OSGGA_LIBRARY OSGSHADOW_LIBRARY OPENTHREADS_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

# Add this project to the NodeKits solution folder
set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

INCLUDE(ModuleInstall OPTIONAL)