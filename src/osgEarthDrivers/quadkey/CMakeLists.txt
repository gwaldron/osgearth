SET(TARGET_SRC
  ReaderWriterQuadKey.cpp
)
SET(TARGET_H
  QuadKeyOptions
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthUtil)

SETUP_PLUGIN(osgearth_quadkey)

# to install public driver includes:
SET(LIB_NAME quadkey)
SET(LIB_PUBLIC_HEADERS QuadKeyOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
