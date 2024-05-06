add_osgearth_plugin(
    TARGET osgdb_osgearth_sky_gl
    HEADERS
        GLSkyNode
    PUBLIC_HEADERS
        GLSkyOptions
    SOURCES
        GLSkyDriver.cpp
        GLSkyNode.cpp
        GLSkyExtension.cpp )