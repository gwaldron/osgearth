#
# texture splatting plugin
#
SET(LIB_NAME osgEarthSplat)

IF(OSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DOSGEARTHSPLAT_LIBRARY)
ELSE ()
    ADD_DEFINITIONS(-DOSGEARTHSPLAT_LIBRARY_STATIC)
ENDIF()

set(TARGET_GLSL
    Splat.types.glsl
    Splat.Noise.glsl
    Splat.vert.model.glsl
    Splat.vert.view.glsl
    Splat.frag.glsl
    Splat.util.glsl
    Splat.GroundCover.CS.glsl
    Splat.GroundCover.Sort.CS.glsl
    Splat.GroundCover.Billboard.glsl
    Splat.GroundCover.Model.glsl
    Splat.GroundCover.Generate.CS.glsl
    Splat.GroundCover.Render.glsl
    Splat.GroundCover.Types.glsl
    Splat.Grass.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    SplatShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_IN    
    SplatShaders.cpp.in)

set(TARGET_SRC
    Biome.cpp
    GrassLayer.cpp
    GroundCoverFeatureGenerator.cpp
    GroundCoverFeatureSource.cpp
    GroundCoverLayer.cpp
    NoiseTextureFactory.cpp
    RoadSurfaceLayer.cpp
	SplatCatalog.cpp
    SplatLayer.cpp
    Surface.cpp
    Zone.cpp
    OldInstanceCloud.cpp
    ${SHADERS_CPP})
	
set(LIB_PUBLIC_HEADERS
    Biome
	Export
    GrassLayer
    GroundCoverFeatureGenerator
    GroundCoverFeatureSource
    GroundCoverLayer
    NoiseTextureFactory
    RoadSurfaceLayer
	SplatCatalog
    SplatShaders
    SplatLayer
    Surface
    Zone
    OldInstanceCloud)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	SplatShaders)

ADD_LIBRARY(${LIB_NAME}
    ${OSGEARTH_DYNAMIC_OR_STATIC}
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

LINK_WITH_VARIABLES(${LIB_NAME} OSG_LIBRARY OSGUTIL_LIBRARY OSGDB_LIBRARY)
LINK_CORELIB_DEFAULT(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIBRARY})

# Add this project to the NodeKits solution folder
set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

INCLUDE(ModuleInstall OPTIONAL)