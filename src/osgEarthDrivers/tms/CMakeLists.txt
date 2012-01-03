SET(TARGET_SRC
  ReaderWriterTMS.cpp
)
SET(TARGET_H
  TMSOptions
)
    
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthUtil)

SETUP_PLUGIN(osgearth_tms)

# to install public driver includes:
SET(LIB_NAME tms)
SET(LIB_PUBLIC_HEADERS TMSOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
