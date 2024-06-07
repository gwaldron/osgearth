# Windows stuff
if(WIN32 AND NOT ANDROID)
    if(MSVC)
        set(CMAKE_DEBUG_POSTFIX  "d" CACHE STRING "add a Debug postfix, usually d on windows")
        set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a Release postfix, usually empty on windows")
        set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "add a RelWithDebInfo postfix, usually empty on windows")
        set(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a MinSizeRel postfix, usually empty on windows")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj") #MSVC 2017/2019
        add_definitions(-D_SCL_SECURE_NO_WARNINGS)
        add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    endif(MSVC)    
endif(WIN32 AND NOT ANDROID)
