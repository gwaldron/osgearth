add_osgearth_app(
    TARGET osgearth_server
    SOURCES osgearth_server.cpp
    FOLDER Tools

    LIBRARIES
        $<$<PLATFORM_ID:Windows>:ws2_32> 
 )



