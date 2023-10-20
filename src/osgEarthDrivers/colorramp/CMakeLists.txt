SET(TARGET_H
    ColorRampOptions
)
SET(TARGET_SRC 
    ColorRampTileSource.cpp
)

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY) 
SETUP_PLUGIN(osgearth_colorramp)

# to install public driver includes:
SET(LIB_NAME colorramp)
SET(LIB_PUBLIC_HEADERS ColorRampOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
