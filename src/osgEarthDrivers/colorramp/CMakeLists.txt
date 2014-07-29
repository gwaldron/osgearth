SET(TARGET_H
    ColorRampOptions
)
SET(TARGET_SRC 
    ColorRampTileSource.cpp
)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)
SETUP_PLUGIN(osgearth_colorramp)


# to install public driver includes:
SET(LIB_NAME osg)
SET(LIB_PUBLIC_HEADERS ColorRampOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

