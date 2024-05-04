add_osgearth_library_as_plugin(
    TARGET osgdb_osgearth_viewpoints
    PUBLIC_HEADERS
        ViewpointsExtension
    SOURCES
        ViewpointsPlugin.cpp
        ViewpointsExtension.cpp )
    