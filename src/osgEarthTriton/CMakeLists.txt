IF(TRITON_FOUND)

    SET(LIB_NAME osgEarthTriton)

    IF(OSGEARTH_BUILD_SHARED_LIBS)
        ADD_DEFINITIONS(-DOSGEARTHTRITON_LIBRARY)
    ELSE ()
        ADD_DEFINITIONS(-DOSGEARTHTRITON_LIBRARY_STATIC)
    ENDIF()

    SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

    SET(LIB_PUBLIC_HEADERS
    #   header files go here
        Common
        Export
        TritonContext
        TritonDrawable
        TritonAPIWrapper
        TritonCallback
        TritonLayer
        TritonHeightMap
        TritonIntersections
    )

    ADD_LIBRARY(${LIB_NAME} 
        ${OSGEARTH_DYNAMIC_OR_STATIC}
        ${LIB_PUBLIC_HEADERS}
    #  .cpp files go here
        TritonContext.cpp
        TritonDrawable.cpp
        TritonAPIWrapper.cpp
        TritonLayer.cpp
        TritonHeightMap.cpp
        TritonIntersections.cpp
    )

    INCLUDE_DIRECTORIES( 
        ${OSG_INCLUDE_DIR}
        ${OSGEARTH_SOURCE_DIR}
        ${TRITON_INCLUDE_DIR}
    )

    LINK_EXTERNAL(
        ${LIB_NAME}
        ${TARGET_EXTERNAL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT} )

    LINK_INTERNAL(
        ${LIB_NAME}
        osgEarth
    )

    LINK_WITH_VARIABLES(
        ${LIB_NAME} 
        OSG_LIBRARY
        OSGDB_LIBRARY
        OSGUTIL_LIBRARY
        TRITON_LIBRARY )

    LINK_CORELIB_DEFAULT(
        ${LIB_NAME}
        ${CMAKE_THREAD_LIBS_INIT} )

    # Add this project to the NodeKits solution folder
    set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

    INCLUDE(ModuleInstall OPTIONAL)

ENDIF(TRITON_FOUND)
