
SET(TARGET_SRC
    Capabilities.cpp
    ReaderWriterWMS.cpp
    TileService.cpp)

SET(TARGET_H
    Capabilities
    TileService
    WMSOptions
)

SETUP_PLUGIN(osgearth_wms)

# to install public driver includes:
SET(LIB_NAME wms)
SET(LIB_PUBLIC_HEADERS WMSOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

