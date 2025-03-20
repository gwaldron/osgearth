#
# osgEarth Procedural NodeKit
#
if(OSGEARTH_BUILD_SHARED_LIBS)
    add_definitions(-DOSGEARTHPROCEDURAL_LIBRARY)
endif()

SET(LIB_NAME osgEarthProcedural)

set(OSGEARTH_HAVE_PROCEDURAL_NODEKIT ON PARENT_SCOPE) # for BuildConfig

# temporary... for devel only...
if(CLIPPER2_FOUND)
    set(CLIPPER2_INCLUDE_DIR "H:/devel/clipper2/install/include")
    if (EXISTS ${CLIPPER2_INCLUDE_DIR})
        include_directories(${CLIPPER2_INCLUDE_DIR})
        SET(CLIPPER2_LIB "H:/devel/clipper2/install/lib/clipper2.lib")
        set(OSGEARTH_HAVE_CLIPPER2)
        add_definitions(-DOSGEARTH_HAVE_CLIPPER2)
    endif()
endif()

# shaders:
set(TARGET_GLSL
    Procedural.Vegetation.glsl
    Procedural.TextureSplatting.glsl
    Procedural.FeatureSplatting.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    ProceduralShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_IN    
    ProceduralShaders.cpp.in)

# sources:
set(TARGET_SRC
    Biome.cpp
    BiomeLayer.cpp
    BiomeManager.cpp
    BridgeLayer.cpp
    RoadNetwork.cpp
    RoadSurfaceLayer.cpp
    TextureSplattingLayer.cpp
    TextureSplattingMaterials.cpp
    LifeMapLayer.cpp
    NodeGraph.cpp
    ProceduralTiledModelLayer.cpp
    VegetationFeatureGenerator.cpp
    VegetationLayer.cpp
    RoadLayer.cpp
    FeatureSplattingLayer.cpp
    ${SHADERS_CPP} )
	
# headers:
set(TARGET_H
    Biome
    BiomeLayer
    BiomeManager
    BridgeLayer
	Export
    NodeGraph
    RoadNetwork
    RoadSurfaceLayer
    ProceduralShaders
    ProceduralTiledModelLayer
    TextureSplattingLayer
    TextureSplattingMaterials
    LifeMapLayer
    VegetationFeatureGenerator
    VegetationLayer
    RoadLayer
    FeatureSplattingLayer)
    
add_osgearth_library(
    TARGET ${LIB_NAME}
    SOURCES ${TARGET_SRC}
    PUBLIC_HEADERS ${TARGET_H}
    SHADERS ${TARGET_GLSL}
    TEMPLATES ${TARGET_IN}
    FOLDER "NodeKits")

if(CLIPPER2_FOUND)
    target_link_libraries(${LIB_NAME} PRIVATE ${CLIPPER2_LIB})
endif()
