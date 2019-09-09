SET(TARGET_SRC
  ReaderWriterCesiumIon.cpp
)
SET(TARGET_H
  CesiumIonOptions
)
    
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthUtil)

SETUP_PLUGIN(osgearth_cesiumion)

# to install public driver includes:
SET(LIB_NAME cesiumion)
SET(LIB_PUBLIC_HEADERS CesiumIonOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
