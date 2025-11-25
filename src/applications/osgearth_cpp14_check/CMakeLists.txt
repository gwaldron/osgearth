set(MY_TARGET osgearth_cpp14_check)

set(CMAKE_CXX_STANDARD 14)

add_executable(${MY_TARGET} main.cpp)

target_compile_features(${MY_TARGET} PRIVATE cxx_std_14)

target_link_libraries(${MY_TARGET} PRIVATE osgEarth ${OPENSCENEGRAPH_LIBRARIES} ${MY_LIBRARIES})
set_target_properties(${MY_TARGET} PROPERTIES PROJECT_LABEL "osgearth_cpp14_check")
    
set_target_properties(${MY_TARGET} PROPERTIES
    FOLDER "Tests"
    DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}"
    RELEASE_POSTFIX "${CMAKE_RELEASE_POSTFIX}"
    RELWITHDEBINFO_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}"
    MINSIZEREL_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}"
)
