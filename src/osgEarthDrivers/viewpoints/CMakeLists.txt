add_osgearth_plugin(
    TARGET osgdb_osgearth_viewpoints
    PUBLIC_HEADERS
        ViewpointsExtension
    SOURCES
        ViewpointsPlugin.cpp
        ViewpointsExtension.cpp )
    