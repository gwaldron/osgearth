#
# Monitor Extension
#

SET(TARGET_SRC
	MonitorPlugin.cpp
	MonitorExtension.cpp
	MonitorUI.cpp)
	
SET(LIB_PUBLIC_HEADERS
	MonitorExtension
	MonitorUI)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS} )

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGGA_LIBRARY OPENTHREADS_LIBRARY) 
SETUP_PLUGIN(osgearth_monitor)

SET(LIB_NAME monitor)

INCLUDE(ModuleInstallOsgEarthPluginIncludes OPTIONAL)
