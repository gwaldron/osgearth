# Windows stuff

if(WIN32 AND NOT ANDROID)
    if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
        add_definitions(-D_SCL_SECURE_NO_WARNINGS)
        add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    endif(MSVC)    
endif(WIN32 AND NOT ANDROID)

# Custom plugin prefixes
if (CYGWIN)
    SET(OSGEARTH_PLUGIN_PREFIX "cygwin_")
endif()

if(MINGW)
    SET(OSGEARTH_PLUGIN_PREFIX "mingw_")
endif()
