cmake_minimum_required(VERSION 3.20)

project(ExternalExample)

find_package(osgEarth CONFIG REQUIRED)

add_executable(ExternalExample main.cpp)

target_link_libraries(ExternalExample PRIVATE osgEarth::osgEarth)
