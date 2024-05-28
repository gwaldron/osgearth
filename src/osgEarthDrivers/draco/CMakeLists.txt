find_package(draco QUIET)

if(draco_FOUND)
    message(STATUS "Found draco")
    add_osgearth_plugin(
        TARGET osgdb_draco
        SOURCES ReaderWriterDraco.cpp
        LIBRARIES draco::draco)
endif()
