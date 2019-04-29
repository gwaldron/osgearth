SET(TARGET_H
    ColorRampOptions
)
SET(TARGET_SRC 
    ColorRampTileSource.cpp
)
SETUP_PLUGIN(osgearth_colorramp)


# to install public driver includes:
SET(LIB_NAME colorramp)
SET(LIB_PUBLIC_HEADERS ColorRampOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

