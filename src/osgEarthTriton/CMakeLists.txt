find_package(Triton REQUIRED)

set(LIB_NAME osgEarthTriton)

set(OSGEARTH_HAVE_TRITON_NODEKIT ON PARENT_SCOPE) # BuildConfig

if(OSGEARTH_BUILD_SHARED_LIBS)
    add_definitions(-DOSGEARTHTRITON_LIBRARY)
else()
    add_definitions(-DOSGEARTHTRITON_LIBRARY_STATIC)
endif()

SET(TARGET_H
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

set(TARGET_SRC
    TritonContext.cpp
    TritonDrawable.cpp
    TritonAPIWrapper.cpp
    TritonLayer.cpp
    TritonHeightMap.cpp
    TritonIntersections.cpp)    

add_osgearth_library(
    TARGET ${LIB_NAME}
    SOURCES ${TARGET_SRC}
    PUBLIC_HEADERS ${TARGET_H}
    FOLDER "NodeKits")

target_link_libraries(${LIB_NAME}
    PRIVATE OE::TRITON OpenGL::GL)
