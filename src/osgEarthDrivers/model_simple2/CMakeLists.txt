SET(TARGET_SRC SimpleModelSource.cpp)
SET(TARGET_H SimpleModelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures2 osgEarthSymbology)
SETUP_PLUGIN(osgearth_model_simple2)


# to install public driver includes:
SET(LIB_NAME model_simple2)
SET(LIB_PUBLIC_HEADERS SimpleModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

