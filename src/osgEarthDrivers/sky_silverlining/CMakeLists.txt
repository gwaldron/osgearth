if(SILVERLINING_FOUND)
    add_osgearth_library_as_plugin(
        TARGET osgdb_osgearth_sky_silverlining
        SOURCES
            SilverLiningDriver.cpp
        LIBRARIES
            osgEarthSilverLining )
endif()
