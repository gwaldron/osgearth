SET(TARGET_SRC FeatureLabelModelSource.cpp)
SET(TARGET_H FeatureLabelModelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SETUP_PLUGIN(osgearth_model_feature_label)


# to install public driver includes:
SET(LIB_NAME model_feature_label)
SET(LIB_PUBLIC_HEADERS FeatureLabelModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

