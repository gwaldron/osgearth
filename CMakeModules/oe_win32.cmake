# Windows stuff

IF(WIN32 AND NOT ANDROID)

  # GL CORE Profile build (OSG must also be built with it)
  FIND_PACKAGE(GLCORE)
  IF(GLCORE_FOUND)
      INCLUDE_DIRECTORIES( ${GLCORE_INCLUDE_DIR} )
  ENDIF()    

  IF(MSVC)
        # This option is to enable the /MP switch for Visual Studio 2005 and above compilers
        OPTION(WIN32_USE_MP "Set to ON to build osgEarth with the /MP option (Visual Studio 2005 and above)." OFF)
        MARK_AS_ADVANCED(WIN32_USE_MP)
        IF(WIN32_USE_MP)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
        ENDIF(WIN32_USE_MP)

        # More MSVC specific compilation flags
        ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
        ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
    ENDIF(MSVC)
ENDIF(WIN32 AND NOT ANDROID)
