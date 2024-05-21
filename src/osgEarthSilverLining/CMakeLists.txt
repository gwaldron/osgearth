find_package(SilverLining REQUIRED)

set(LIB_NAME osgEarthSilverLining)

set(OSGEARTH_HAVE_SILVERLINING_NODEKIT ON PARENT_SCOPE) # for BuildConfig

if(OSGEARTH_BUILD_SHARED_LIBS)
    add_definitions(-DOSGEARTHSILVERLINING_LIBRARY)
else()
    add_definitions(-DOSGEARTHSILVERLINING_LIBRARY_STATIC)
endif()

option(SILVERLINING_USE_PRE_5_079_API "Whether to use the pre-5.079 API due to an API change" OFF)
if (SILVERLINING_USE_PRE_5_079_API)
    add_definitions(-DOSGEARTH_SILVERLINING_USE_PRE_5_079_API)
endif()

SET(HEADER_PATH ${OSGEARTH_SOURCE_DIR}/include/${LIB_NAME})

SET(TARGET_H
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

set(TARGET_SRC
    SilverLiningNode.cpp
    SilverLiningContext.cpp
    SilverLiningSkyDrawable.cpp
    SilverLiningCloudsDrawable.cpp
    SilverLiningAPIWrapper.cpp
    SilverLiningContextNode.cpp)

add_osgearth_library(
    TARGET ${LIB_NAME}
    SOURCES ${TARGET_SRC}
    PUBLIC_HEADERS ${TARGET_H} )

include_directories(
    ${SILVERLINING_INCLUDE_DIR})
    
target_link_libraries(${LIB_NAME}
    PUBLIC osgEarth ${OPENSCENEGRAPH_LIBRARIES} ${OPENGL_LIBRARIES} OE::SILVERLINING)

# Add this project to the NodeKits solution folder
set_property( TARGET ${LIB_NAME} PROPERTY FOLDER "NodeKits" )
