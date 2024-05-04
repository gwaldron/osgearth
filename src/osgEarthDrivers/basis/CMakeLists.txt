
#find_package(BASISU)

if(BASISU_FOUND)

    add_osgearth_library_as_plugin(TARGET osgdb_basis
        INCLUDE_DIRECTORIES ${BASISU_INCLUDE_DIR}
        SOURCES ReaderWriterBasis.cpp
        LIBRARIES ${BASISU_TRANSCODER_LIBRARY})

endif()
