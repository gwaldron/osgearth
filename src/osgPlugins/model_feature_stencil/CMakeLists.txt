SET(TARGET_SRC ReaderWriterFeatureStencil.cpp)
SET(TARGET_H StencilUtils.h)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)
SETUP_PLUGIN(osgearth_model_feature_stencil)
