#find_package(BASISU)

if(BASISU_FOUND)
    add_osgearth_plugin(
        TARGET osgdb_basis
        SOURCES ReaderWriterBasis.cpp
        INCLUDE_DIRECTORIES ${BASISU_INCLUDE_DIR}
        LIBRARIES ${BASISU_TRANSCODER_LIBRARY})
endif()
