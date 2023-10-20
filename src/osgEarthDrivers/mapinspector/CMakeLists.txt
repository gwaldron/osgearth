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
	
SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY) 

SETUP_PLUGIN(osgearth_mapinspector)

SET(LIB_NAME mapinspector)

INCLUDE(ModuleInstallOsgEarthPluginIncludes OPTIONAL)
