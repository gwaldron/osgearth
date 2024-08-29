# External Project Example
This is a simple example to test the `osgEarth-config.cmake` packaging configuration file for CMake.

These line in your `CMakeLists.txt` will tell CMake to look for `osgEarth-config.cmake` and use it to resolve osgEarth and its public-facing dependencies:
```
find_package(osgEarth CONFIG REQUIRED)
...
target_link_libraries(my_project PRIVATE osgEarth::osgEarth)
```
You may need to update your `CMAKE_PREFIX_PATH` to include the location of `osgEarthConfig.cmake`.