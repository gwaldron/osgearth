#
# texture splatting plugin
#

SET(TARGET_SRC
	SplatPlugin.cpp
	SplatExtension.cpp
	SplatCatalog.cpp
	SplatClassificationMap.cpp)
	
SET(LIB_PUBLIC_HEADERS
	SplatOptions)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	SplatExport
	SplatExtension
	SplatCatalog
	SplatClassificationMap)
	
SETUP_EXTENSION(osgearth_splat)

# to install public driver includes:
SET(LIB_NAME splat)

INCLUDE(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

