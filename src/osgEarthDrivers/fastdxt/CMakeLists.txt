option(OSGEARTH_ENABLE_FASTDXT "Set to ON to build optional FastDXT image compressor." ON)
if(OSGEARTH_ENABLE_FASTDXT)

    add_osgearth_plugin(
        TARGET osgdb_fastdxt
        HEADERS 
            dxt.h
            libdxt.h
            util.h
        SOURCES
            FastDXTImageProcessor.cpp
            dxt.cpp
            util.cpp
            libdxt.cpp
            intrinsic.cpp )

ENDIF()


