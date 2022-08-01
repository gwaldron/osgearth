#
# osgEarth Procedural NodeKit
#
IF(OSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DOSGEARTHPROCEDURAL_LIBRARY)
ELSE()
    ADD_DEFINITIONS(-DOSGEARTHPROCEDURAL_LIBRARY_STATIC)
ENDIF()

SET(LIB_NAME osgEarthProcedural)
set(OSGEARTHPROCEDURAL_LIBRARY ${LIB_NAME})

# shaders:
set(TARGET_GLSL
    Procedural.Vegetation.glsl
    Procedural.TextureSplatting.glsl
    Procedural.HexTiling.glsl )

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
    NoiseTextureFactory.cpp
    RoadSurfaceLayer.cpp
    TextureSplattingLayer.cpp
    TextureSplattingMaterials.cpp
    LifeMapLayer.cpp
    VegetationFeatureGenerator.cpp
    VegetationLayer.cpp
    ${SHADERS_CPP}
    )
	
# headers:
set(LIB_PUBLIC_HEADERS
    Biome
    BiomeLayer
    BiomeManager
	Export
    NoiseTextureFactory
    RoadSurfaceLayer
    ProceduralShaders
    TextureSplattingLayer
    TextureSplattingMaterials
    LifeMapLayer
    VegetationFeatureGenerator
    VegetationLayer
    )
    
# ImGui headers:
if(OSGEARTH_ENABLE_IMGUI)
    set(LIB_PUBLIC_HEADERS_IMGUI
        ImGui/LifeMapLayerGUI
        ImGui/VegetationLayerGUI
        ImGui/TextureSplattingLayerGUI
        )    
endif()

add_library(
    ${LIB_NAME}
    ${OSGEARTH_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${LIB_PUBLIC_HEADERS_IMGUI}
    ${TARGET_SRC}
    ${TARGET_GLSL}
    ${TARGET_IN}
)

LINK_INTERNAL(
    ${LIB_NAME} 
    ${TARGET_EXTERNAL_LIBRARIES} 
    ${CMAKE_THREAD_LIBS_INIT} 
    ${MATH_LIBRARY} )

LINK_INTERNAL(
    ${LIB_NAME}
    osgEarth
)

IF (TRACY_FOUND)
    include_directories(${TRACY_INCLUDE_DIR})
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

# Place it in the NodeKits solution folder
set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

INCLUDE(ModuleInstall OPTIONAL)