#
# image billboarding plugin
#

SET(TARGET_SRC
        BillboardPlugin.cpp
	BillboardExtension.cpp)
	
SET(LIB_PUBLIC_HEADERS
	BillboardExtension
	BillboardOptions)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS}
	BillboardShaders)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil
    osgEarthFeatures)
	
SETUP_EXTENSION(osgearth_billboard)

# to install public driver includes:
SET(LIB_NAME billboard)

INCLUDE(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

