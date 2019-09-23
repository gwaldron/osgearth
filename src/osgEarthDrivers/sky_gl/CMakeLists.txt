SET(TARGET_SRC 
    GLSkyDriver.cpp
    GLSkyNode.cpp
    GLSkyExtension.cpp
)
SET(TARGET_H
    GLSkyOptions
	GLSkyNode
)

SETUP_PLUGIN(osgearth_sky_gl)

# to install public driver includes:
SET(LIB_NAME sky_gl)
SET(LIB_PUBLIC_HEADERS GLSkyOptions)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
