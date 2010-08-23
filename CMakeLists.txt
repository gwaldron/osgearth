cmake_minimum_required(VERSION 2.6)

PROJECT(SEAMLESS)

# We have some custom .cmake scripts not in the official distribution.
# Maybe this can be used override existing behavior if needed?
SET(CMAKE_MODULE_PATH "${SEAMLESS_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

FIND_PACKAGE(OpenThreads)
FIND_PACKAGE(osg)
FIND_PACKAGE(osgUtil)
FIND_PACKAGE(osgDB)
FIND_PACKAGE(osgViewer)
FIND_PACKAGE(osgShadow)
FIND_PACKAGE(osgSim)
FIND_PACKAGE(osgParticle)
FIND_PACKAGE(osgTerrain)
FIND_PACKAGE(osgFX)
FIND_PACKAGE(GDAL)

FIND_PACKAGE(osgEarth)

SET(SOURCES
    qsc.cpp QSC.cpp kml.cpp Patch.cpp PatchGroup.cpp PatchSet.cpp patchTest.cpp SeamlessPlugin.cpp Projected.cpp Geographic.cpp GeoPatch.cpp seamlessTest.cpp)

INCLUDE_DIRECTORIES(
        ${SEAMLESS_SOURCE_DIR}/..
        ${OPENTHREADS_INCLUDE_DIR} ${OSG_INCLUDE_DIR} ${OSGEARTH_INCLUDE_DIR}
        ${GDAL_INCLUDE_DIR})

LINK_DIRECTORIES(${OSG_LIB_DIR} ${OSGEARTH_LIB_DIR})

ADD_EXECUTABLE(kmld kml.cpp qsc.cpp)

ADD_EXECUTABLE(patchTest patchTest.cpp Patch.cpp PatchGroup.cpp PatchSet.cpp)

ADD_EXECUTABLE(seamlessTest Patch.cpp PatchGroup.cpp PatchSet.cpp SeamlessPlugin.cpp Projected.cpp QSC.cpp Geographic.cpp GeoPatch.cpp seamlessTest.cpp)

TARGET_LINK_LIBRARIES(kmld ${OSG_LIBRARY_DEBUG} )

TARGET_LINK_LIBRARIES(patchTest ${OSGEARTH_LIBRARY} ${OSG_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGSHADOW_LIBRARY} ${OSGSIM_LIBRARY} ${OSGPARTICLE_LIBRARY} ${OSGTERRAIN_LIBRARY} ${OSGFX_LIBRARY})

TARGET_LINK_LIBRARIES(
        seamlessTest ${OSGEARTH_LIBRARY} ${OSG_LIBRARY}
        ${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY}
        ${OSGVIEWER_LIBRARY} ${OSGSHADOW_LIBRARY}
        ${OSGSIM_LIBRARY} ${OSGPARTICLE_LIBRARY}
        ${OSGTERRAIN_LIBRARY} ${OSGFX_LIBRARY})
