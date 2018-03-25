OPTION(OSGEARTH_ENABLE_FASTDXT "Set to ON to build optional FastDXT image compressor." ON)
IF(OSGEARTH_ENABLE_FASTDXT)

OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

IF(MINGW)
    # required by gcc 7.2.0
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
ENDIF(MINGW)

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
    IF(CURL_IS_STATIC)
        ADD_DEFINITIONS(-DCURL_STATICLIB)
        SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm)
    ENDIF(CURL_IS_STATIC)
ENDIF(WIN32)

SET(TARGET_H
    dxt.h
    libdxt.h
    util.h
)

SET(TARGET_SRC
    FastDXTImageProcessor.cpp
    dxt.cpp
    util.cpp
    libdxt.cpp
    intrinsic.cpp
)

SETUP_PLUGIN(fastdxt)

ENDIF(OSGEARTH_ENABLE_FASTDXT)
