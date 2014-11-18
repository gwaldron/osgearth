#
# texture splatting plugin
#

set(TARGET_GLSL
    Noise.glsl
    Splat.vert.model.glsl
    Splat.vert.view.glsl
    Splat.frag.glsl)

set(SHADERS_CPP "${CMAKE_CURRENT_BINARY_DIR}/AutoGenShaders.cpp")

configure_shaders(
    SplatShaders.cpp.in
    ${SHADERS_CPP}
    ${TARGET_GLSL} )

SET(TARGET_SRC
	SplatPlugin.cpp
	SplatExtension.cpp
	SplatCatalog.cpp
	SplatTerrainEffect.cpp
	SplatCoverageLegend.cpp
	ModelSplatter.cpp
    ${SHADERS_CPP})
	
SET(LIB_PUBLIC_HEADERS
	SplatCoverageLegend
	SplatCatalog
	SplatExtension
	SplatOptions
	SplatTerrainEffect
	ModelSplatter)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	SplatExport
	SplatShaders
	NoiseShaders)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil)
	
SETUP_EXTENSION(osgearth_splat)

# to install public driver includes:
SET(LIB_NAME splat)

INCLUDE(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

