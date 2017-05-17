INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )
SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY)

SET(TARGET_SRC
    main.cpp
    GeoExtentTests.cpp
    ImageLayerTests.cpp
    SpatialReferenceTests.cpp
    ThreadingTests.cpp
    )

#### end var setup  ###
SETUP_APPLICATION(osgEarth_tests)

add_test(NAME osgEarth_tests COMMAND osgEarth_tests)