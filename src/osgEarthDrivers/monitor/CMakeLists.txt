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
	
SETUP_PLUGIN(osgearth_monitor)

SET(LIB_NAME monitor)

INCLUDE(ModuleInstallOsgEarthPluginIncludes OPTIONAL)

