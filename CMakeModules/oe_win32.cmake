# Windows stuff

IF(WIN32 AND NOT ANDROID)

  # GW: no longer required (keep for posterity)
  # GL CORE Profile build (OSG must also be built with it)
  # FIND_PACKAGE(GLCORE)
  #IF(GLCORE_FOUND)
  #    INCLUDE_DIRECTORIES( ${GLCORE_INCLUDE_DIR} )
  #    message(status "Found GLCORE headers at ${GLCORE_INCLUDE_DIR}")
  #ENDIF()    

  IF(MSVC)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
        ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
        ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
    ENDIF(MSVC)
    
ENDIF(WIN32 AND NOT ANDROID)
