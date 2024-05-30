add_osgearth_app(
    TARGET osgearth_silverlining
    SOURCES osgearth_silverlining.cpp
    FOLDER Tests )

target_link_libraries(osgearth_silverlining PRIVATE osgEarthSilverLining)
