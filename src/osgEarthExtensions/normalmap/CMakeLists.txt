#
# normal mapping plugin
#

SET(TARGET_SRC
	NormalMapPlugin.cpp
	NormalMapExtension.cpp
	NormalMapTerrainEffect.cpp)
	
SET(LIB_PUBLIC_HEADERS
	NormalMapExtension
	NormalMapOptions
	NormalMapTerrainEffect)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	NormalMapShaders)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil)
	
SETUP_EXTENSION(osgearth_normalmap)

# to install public driver includes:
SET(LIB_NAME normalmap)

INCLUDE(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

