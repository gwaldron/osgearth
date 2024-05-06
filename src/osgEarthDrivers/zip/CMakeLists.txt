if(LIBZIP_FOUND AND OSGEARTH_BUILD_ZIP_PLUGIN)

    include_directories( ${LIBZIP_INCLUDE_DIRS} )
    
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-implicit-fallthrough")
    endif()
    
    add_osgearth_plugin(
        TARGET osgdb_zip
        HEADERS
            ZipArchive.h
        SOURCES
            ZipArchive.cpp
            ReaderWriterZIP.cpp )
    
endif()
