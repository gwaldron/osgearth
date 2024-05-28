find_package(WebP QUIET)

if (WebP_FOUND)
    message(STATUS "Found WebP ${WEBP_VERSION}")
    add_osgearth_plugin(
        TARGET osgdb_webp
        SOURCES ReaderWriterWebP.cpp)
    target_link_libraries(osgdb_webp PRIVATE WebP::webp)
endif()
