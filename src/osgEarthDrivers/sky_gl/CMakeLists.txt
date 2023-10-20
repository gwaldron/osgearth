SET(TARGET_SRC 
    GLSkyDriver.cpp
    GLSkyNode.cpp
    GLSkyExtension.cpp
)
SET(TARGET_H
    GLSkyOptions
	GLSkyNode
)

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY OPENGL_gl_LIBRARY) 

SETUP_PLUGIN(osgearth_sky_gl)

# to install public driver includes:
SET(LIB_NAME sky_gl)
SET(LIB_PUBLIC_HEADERS GLSkyOptions)


INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
