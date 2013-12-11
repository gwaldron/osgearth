
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)

SET(TARGET_SRC
	TemplateMatClassDriver.cpp)
	
SET(TARGET_H
	TemplateMatClassOptions
)

SETUP_PLUGIN(osgearth_template_matclass)

# to install public driver includes:
SET(LIB_NAME template_matclass)
SET(LIB_PUBLIC_HEADERS TemplateMatClassOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
