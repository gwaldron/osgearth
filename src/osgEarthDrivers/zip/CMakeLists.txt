if(OSGEARTH_BUILD_ZIP_PLUGIN)

    find_package(libzip)

    if(libzip_FOUND)

        if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-implicit-fallthrough")
        endif()
        
        add_osgearth_plugin(
            TARGET osgdb_zip
            HEADERS
                ZipArchive.h
            SOURCES
                ZipArchive.cpp
                ReaderWriterZIP.cpp)
                
        target_link_libraries(osgdb_zip PRIVATE libzip::zip)
        
    endif()

endif()