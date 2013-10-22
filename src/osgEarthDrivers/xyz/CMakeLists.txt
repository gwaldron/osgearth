SET(TARGET_SRC
  ReaderWriterXYZ.cpp
)
SET(TARGET_H
  XYZOptions
)
    
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthUtil)

SETUP_PLUGIN(osgearth_xyz)

# to install public driver includes:
SET(LIB_NAME xyz)
SET(LIB_PUBLIC_HEADERS XYZOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
