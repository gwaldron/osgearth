SET(TARGET_SRC FeatureGeomModelSource.cpp)
SET(TARGET_H FeatureGeomModelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)
SETUP_PLUGIN(osgearth_model_feature_geom)


# to install public driver includes:
SET(LIB_NAME model_feature_geom)
SET(LIB_PUBLIC_HEADERS FeatureGeomModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

