SET(TARGET_SRC
    ReaderWriterVPB.cpp
)

SET(TARGET_H
    VPBOptions
)

SETUP_PLUGIN(osgearth_vpb)


# to install public driver includes:
SET(LIB_NAME vpb)
SET(LIB_PUBLIC_HEADERS VPBOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
