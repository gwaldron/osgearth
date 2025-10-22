message(STATUS  "oe...osgearth_map: OSG_DIR=${OSG_DIR}")
if(NOT EXISTS "${OSG_DIR}")
    message(FATAL_ERROR  "oe...osgearth_map: OSG_DIR=${OSG_DIR} not exist.")
endif(NOT EXISTS "${OSG_DIR}")

message(STATUS  "oe...osgearth_map/CMakeLists.txt: OSGEARTH_BUILD_SHARED_LIBS=${OSGEARTH_BUILD_SHARED_LIBS};;;OPENSCENEGRAPH_LIBRARIES=${OPENSCENEGRAPH_LIBRARIES}")
if(OSGEARTH_BUILD_SHARED_LIBS)
    message(STATUS  "oe...osgearth_map/CMakeLists.txt: 1")
    set(OPENSCENEGRAPH_LIBRARIES 
        ${OPENSCENEGRAPH_LIBRARIES}
        ${FREETYPE_LIBRARY}
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_ive.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_osg.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_p3d.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_rgb.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_openflight.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_obj.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_freetype.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_png.so
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_jpeg.so  
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/osgdb_curl.so
        ${OSG_DIR}/lib/libosgTerrain.so
        ${OSG_DIR}/lib/libosgFX.so
        ${OSG_DIR}/lib/libosgParticle.so        
        )
else()
    set(OPENSCENEGRAPH_LIBRARIES 
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSG_DIR}/lib/libosgTerrain.a
        ${OSG_DIR}/lib/libosgFX.a
        ${OSG_DIR}/lib/libosgParticle.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_ive.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osg.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_p3d.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_rgb.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_openflight.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_obj.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_freetype.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_png.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_jpeg.a  
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_curl.a
        ${FREETYPE_LIBRARY}
        )
endif(OSGEARTH_BUILD_SHARED_LIBS)


add_osgearth_app(
    TARGET osgearth_map
    SOURCES osgearth_map.cpp
    FOLDER Examples)
