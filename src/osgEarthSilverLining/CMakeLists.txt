IF(SILVERLINING_FOUND)

    SET(LIB_NAME osgEarthSilverLining)

    IF(OSGEARTH_BUILD_SHARED_LIBS)
        ADD_DEFINITIONS(-DOSGEARTHSILVERLINING_LIBRARY)
    ELSE ()
        ADD_DEFINITIONS(-DOSGEARTHSILVERLINING_LIBRARY_STATIC)
    ENDIF()
    
    option(SILVERLINING_USE_PRE_5_079_API "Whether to use the pre-5.079 API due to an API change" OFF)
    if (SILVERLINING_USE_PRE_5_079_API)
        add_definitions(-DOSGEARTH_SILVERLINING_USE_PRE_5_079_API)
    endif()

    SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

    SET(LIB_PUBLIC_HEADERS
    #   header files go here
        Common
        Export
        SilverLiningOptions
        SilverLiningNode
        SilverLiningContext
        SilverLiningSkyDrawable
        SilverLiningCloudsDrawable
        SilverLiningAPIWrapper
        SilverLiningCallback
        SilverLiningContextNode
    )

    ADD_LIBRARY(${LIB_NAME}
        ${OSGEARTH_DYNAMIC_OR_STATIC}
        ${LIB_PUBLIC_HEADERS}
    #  .cpp files go here
        SilverLiningNode.cpp
        SilverLiningContext.cpp
        SilverLiningSkyDrawable.cpp
        SilverLiningCloudsDrawable.cpp
        SilverLiningAPIWrapper.cpp
        SilverLiningContextNode.cpp
    )

    INCLUDE_DIRECTORIES( 
        ${OSG_INCLUDE_DIR}
        ${OSGEARTH_SOURCE_DIR}
        ${SILVERLINING_INCLUDE_DIR}
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
        OPENTHREADS_LIBRARY
        SILVERLINING_LIBRARY)

    LINK_CORELIB_DEFAULT(
        ${LIB_NAME}
        ${CMAKE_THREAD_LIBS_INIT} )

    # Add this project to the NodeKits solution folder
    set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )

    INCLUDE(ModuleInstall OPTIONAL)

ENDIF(SILVERLINING_FOUND)
