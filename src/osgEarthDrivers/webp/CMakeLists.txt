if(WEBP_FOUND)
    include_directories( ${WEBP_INCLUDE_DIR} )
    add_osgearth_library_as_plugin(
        TARGET osgdb_webp
        SOURCES ReaderWriterWebP.cpp )
endif(WEBP_FOUND)

