#
# texture splatting plugin
#

set(TARGET_GLSL
    Splat.types.glsl
    Splat.Noise.glsl
    Splat.vert.model.glsl
    Splat.vert.view.glsl
    Splat.frag.glsl
    Splat.frag.common.glsl
    Splat.frag.getRenderInfo.glsl
    Splat.util.glsl )

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    SplatShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

set(TARGET_IN    
    SplatShaders.cpp.in)

set(TARGET_SRC
    Biome.cpp
	BiomeSelector.cpp
    LandUseTileSource.cpp
	SplatPlugin.cpp
	SplatExtension.cpp
	SplatCatalog.cpp
	SplatTerrainEffect.cpp
	SplatCoverageLegend.cpp
    ${SHADERS_CPP})
	
set(LIB_PUBLIC_HEADERS
    Biome
	BiomeSelector
    LandUseTileSource
	SplatCoverageLegend
	SplatCatalog
	SplatExtension
	SplatOptions
	SplatTerrainEffect)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	SplatExport
	SplatShaders)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil)
	
setup_extension(osgearth_splat)

# to install public driver includes:
set(LIB_NAME splat)

include(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

