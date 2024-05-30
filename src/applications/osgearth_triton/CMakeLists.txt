add_osgearth_app(
    TARGET osgearth_triton
    SOURCES osgearth_triton.cpp
    FOLDER Tests )

target_link_libraries(osgearth_triton PRIVATE osgEarthTriton)
