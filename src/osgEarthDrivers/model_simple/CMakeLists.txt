SET(TARGET_SRC SimpleModelSource.cpp)
SET(TARGET_H SimpleModelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SETUP_PLUGIN(osgearth_model_simple)


# to install public driver includes:
SET(LIB_NAME model_simple)
SET(LIB_PUBLIC_HEADERS SimpleModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
