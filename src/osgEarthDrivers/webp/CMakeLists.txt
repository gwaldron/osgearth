find_package(WEBP)

if (WEBP_FOUND)
    message(STATUS "Found WebP ${WEBP_VERSION}")
    include_directories(${WEBP_INCLUDE_DIRS})
    add_osgearth_plugin(
        TARGET osgdb_webp
        SOURCES ReaderWriterWebP.cpp
        LIBRARIES ${WEBP_LIBRARIES})
endif()

