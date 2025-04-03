cmake_minimum_required(VERSION 3.20)

project(
    osgearth-demo
    VERSION 1.0.0
    LANGUAGES CXX C
)

find_package(osgEarth CONFIG REQUIRED)

add_executable(osgearth-demo main.cpp)

target_link_libraries(osgearth-demo PRIVATE osgEarth::osgEarth)

install(TARGETS osgearth-demo RUNTIME DESTINATION bin)
