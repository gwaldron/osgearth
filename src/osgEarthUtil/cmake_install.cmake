# Install script for directory: /Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/lib/Debug/libosgEarthUtild.a")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtild.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtild.a")
      EXECUTE_PROCESS(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtild.a")
    ENDIF()
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/lib/Release/libosgEarthUtil.a")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a")
      EXECUTE_PROCESS(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a")
    ENDIF()
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/lib/MinSizeRel/libosgEarthUtil.a")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a")
      EXECUTE_PROCESS(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a")
    ENDIF()
  ELSEIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/lib/RelWithDebInfo/libosgEarthUtil.a")
    IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a")
      EXECUTE_PROCESS(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgEarthUtil.a")
    ENDIF()
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osgEarthUtil" TYPE FILE FILES
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/AnnotationEvents"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/AutoClipPlaneHandler"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/Common"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/Controls"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/ClampCallback"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/EarthManipulator"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/ElevationManager"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/ExampleResources"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/Export"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/FeatureManipTool"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/FeatureQueryTool"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/Formatter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/GeodeticGraticule"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/LatLongFormatter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/LineOfSight"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/LinearLineOfSight"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/MeasureTool"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/MGRSFormatter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/MGRSGraticule"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/MouseCoordsTool"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/ObjectPlacer"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/ObjectLocator"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/PolyhedralLineOfSight"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/RadialLineOfSight"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/SkyNode"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/SpatialData"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/StarData"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/TerrainProfile"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/TFS"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/TFSPackager"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/TMS"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/TMSPackager"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/UTMGraticule"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/WFS"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/WMS"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/BrightnessContrastColorFilter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/CMYKColorFilter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/GammaColorFilter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/HSLColorFilter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/RGBColorFilter"
    "/Users/hogbox/Documents/AlphaPixel/osgEarthPort/osgearth/src/osgEarthUtil/ChromaKeyColorFilter"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

