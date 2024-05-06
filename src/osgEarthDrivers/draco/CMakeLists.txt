find_package(draco)

if(draco_FOUND)
    add_osgearth_plugin(
        TARGET osgdb_draco
        SOURCES ReaderWriterDraco.cpp
        LIBRARIES draco::draco)
endif()
