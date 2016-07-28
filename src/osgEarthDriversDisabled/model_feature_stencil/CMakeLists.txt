SET(TARGET_SRC
    FeatureStencilModelSource.cpp
)

SET(TARGET_H
    FeatureStencilModelOptions
)
    
SET(TARGET_COMMON_LIBRARIES
    ${TARGET_COMMON_LIBRARIES}
    osgEarthFeatures
    osgEarthSymbology
)

SETUP_PLUGIN(osgearth_model_feature_stencil)


# to install public driver includes:
SET(LIB_NAME model_feature_stencil)
SET(LIB_PUBLIC_HEADERS FeatureStencilModelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
