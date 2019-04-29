#
# Viewpoints plugin
#

SET(TARGET_SRC
	ViewpointsPlugin.cpp
	ViewpointsExtension.cpp)
	
SET(LIB_PUBLIC_HEADERS
	ViewpointsExtension)
	
SET(TARGET_H
	${LIB_PUBLIC_HEADERS} )
	
setup_plugin(osgearth_viewpoints)

# to install public driver includes:
SET(LIB_NAME viewpoints)

#INCLUDE(ModuleInstallOsgEarthExtensionIncludes OPTIONAL)

