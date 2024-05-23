#
# texture splatting plugin
#
SET(LIB_NAME osgEarthSplat)

set(OSGEARTH_HAVE_LEGACY_SPLAT_NODEKIT ON PARENT_SCOPE) # for BuildConfig

if(OSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DOSGEARTHSPLAT_LIBRARY)
else()
    ADD_DEFINITIONS(-DOSGEARTHSPLAT_LIBRARY_STATIC)
endif()

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
	
set(TARGET_H
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
	
add_osgearth_library(
    TARGET ${LIB_NAME}
    SOURCES ${TARGET_SRC}
    PUBLIC_HEADERS ${TARGET_H}
    SHADERS ${TARGET_GLSL}
    TEMPLATES ${TARGET_IN}
    FOLDER "NodeKits")

target_link_libraries(${LIB_NAME} PRIVATE OpenGL::GL)



