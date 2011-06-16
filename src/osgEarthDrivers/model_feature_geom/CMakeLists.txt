SET(TARGET_SRC
    FeatureGeomModelSource.cpp )
    
SET(TARGET_H
    FeatureGeomModelOptions )
    
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)

SETUP_PLUGIN(osgearth_model_feature_geom)


# to install public driver includes:
SET(LIB_NAME model_feature_geom)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
