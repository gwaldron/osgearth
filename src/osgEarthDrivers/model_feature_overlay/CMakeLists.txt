SET(TARGET_SRC FeatureOverlayModelSource.cpp)
SET(TARGET_H FeatureOverlayModelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)
SETUP_PLUGIN(osgearth_model_feature_overlay)

# to install public driver includes:
SET(LIB_NAME model_feature_overlay)
SET(LIB_PUBLIC_HEADERS FeatureOverlayModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

