SET(TARGET_SRC
    FeatureStencilModelSource.cpp
)
    
SET(TARGET_COMMON_LIBRARIES
    ${TARGET_COMMON_LIBRARIES}
    osgEarthFeatures
)

SETUP_PLUGIN(osgearth_model_feature_stencil)
