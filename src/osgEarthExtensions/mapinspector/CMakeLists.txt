#
# MapInspector Extension
#

SET(TARGET_SRC
	MapInspectorPlugin.cpp
	MapInspectorExtension.cpp
	MapInspectorUI.cpp)
	
SET(LIB_PUBLIC_HEADERS
	MapInspectorExtension
	MapInspectorUI)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS} )

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil
	osgEarthFeatures
	osgEarthSymbology)
	
SETUP_EXTENSION(osgearth_mapinspector)

SET(LIB_NAME mapinspector)

INCLUDE(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

