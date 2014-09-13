#
# texture splatting plugin
#

SET(TARGET_SRC
	SplatPlugin.cpp
	SplatExtension.cpp
	SplatCatalog.cpp
	SplatTerrainEffect.cpp
	SplatCoverageLegend.cpp)
	
SET(LIB_PUBLIC_HEADERS
	SplatCoverageLegend
	SplatCatalog
	SplatExtension
	SplatOptions
	SplatTerrainEffect)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	SplatExport
	SplatShaders)
	
SETUP_EXTENSION(osgearth_splat)

# to install public driver includes:
SET(LIB_NAME splat)

INCLUDE(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

