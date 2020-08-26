INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${BENCHMARK_INCLUDE_DIR})
SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY)

SET(TARGET_SRC
    osgearth_benchmarks.cpp
    )

link_libraries(benchmark::benchmark benchmark::benchmark_main)

#### end var setup  ###
SETUP_APPLICATION(osgearth_benchmarks)