#
# MapInspector Extension
#

SET(TARGET_SRC
	MapInspectorExtension.cpp
	MapInspectorUI.cpp)
	
SET(LIB_PUBLIC_HEADERS
	MapInspectorExtension
	MapInspectorUI)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS} )
	
SETUP_PLUGIN(osgearth_mapinspector)

SET(LIB_NAME mapinspector)

INCLUDE(ModuleInstallOsgEarthPluginIncludes OPTIONAL)

