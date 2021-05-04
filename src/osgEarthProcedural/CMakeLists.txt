#
# osgEarth Procedural NodeKit
#
IF(OSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DOSGEARTHPROCEDURAL_LIBRARY)
ELSE()
    ADD_DEFINITIONS(-DOSGEARTHPROCEDURAL_LIBRARY_STATIC)
ENDIF()

SET(LIB_NAME osgEarthProcedural)

set(TARGET_GLSL
    Procedural.GroundCover.CS.glsl
    Procedural.GroundCover.Render.glsl
    Procedural.GroundCover.Types.glsl
    Procedural.Grass.glsl
    Procedural.TextureSplatting.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    ProceduralShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_IN    
    ProceduralShaders.cpp.in)

set(TARGET_SRC
    Biome.cpp
    BiomeLayer.cpp
    BiomeManager.cpp
    GrassLayer.cpp
    GroundCoverFeatureGenerator.cpp
    #GroundCoverFeatureSource.cpp
    GroundCoverLayer.cpp
    NoiseTextureFactory.cpp
    RoadSurfaceLayer.cpp
    TextureSplattingLayer.cpp
    TextureSplattingMaterials.cpp
    LifeMapLayer.cpp
    ${SHADERS_CPP})
	
set(LIB_PUBLIC_HEADERS
    Biome
    BiomeLayer
    BiomeManager
	Export
    GrassLayer
    GroundCoverFeatureGenerator
    #GroundCoverFeatureSource
    GroundCoverLayer
    NoiseTextureFactory
    RoadSurfaceLayer
    ProceduralShaders
    TextureSplattingLayer
    TextureSplattingMaterials
    LifeMapLayer)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	ProceduralShaders)

ADD_LIBRARY(
    ${LIB_NAME}
    ${OSGEARTH_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TARGET_SRC}
    ${TARGET_GLSL}
    ${TARGET_IN}
)

LINK_EXTERNAL(
    ${LIB_NAME} 
    ${TARGET_EXTERNAL_LIBRARIES} 
    ${CMAKE_THREAD_LIBS_INIT} 
    ${MATH_LIBRARY} )

LINK_INTERNAL(
    ${LIB_NAME}
    osgEarth
)

IF (TRACY_FOUND)
    INCLUDE_DIRECTORIES(${TRACY_INCLUDE_DIR})
    LINK_WITH_VARIABLES(
        ${LIB_NAME}
        TRACY_LIBRARY )
ENDIF(TRACY_FOUND)

LINK_WITH_VARIABLES(
    ${LIB_NAME}
    OSG_LIBRARY
    OSGUTIL_LIBRARY
    OSGDB_LIBRARY)

LINK_CORELIB_DEFAULT(
    ${LIB_NAME}
    ${CMAKE_THREAD_LIBS_INIT}
    ${MATH_LIBRARY} )

# Add this project to the NodeKits solution folder
set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

INCLUDE(ModuleInstall OPTIONAL)