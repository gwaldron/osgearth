SET(TARGET_SRC FeatureGeomModelSource.cpp)
SET(TARGET_H FeatureGeomModelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures2 osgEarthSymbology)
SETUP_PLUGIN(osgearth_model_feature_geom2)


# to install public driver includes:
SET(LIB_NAME model_feature_geom2)
SET(LIB_PUBLIC_HEADERS FeatureGeomModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

