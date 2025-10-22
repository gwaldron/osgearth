if(OSGEARTH_BUILD_SHARED_LIBS) 
    message(STATUS  "oe...osgearth_viewer/CMakeLists.txt..1: osgLIBs=${OPENSCENEGRAPH_LIBRARIES}")
   
else()
    set(OPENSCENEGRAPH_LIBRARIES 
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSG_DIR}/lib/libosgTerrain.a
        ${OSG_DIR}/lib/libosgFX.a
        ${OSG_DIR}/lib/libosgParticle.a
        ${OSG_DIR}//lib/osgPlugins-3.7.0/libosgdb_deprecated_osg.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osg.a 
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_rgb.a  
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_freetype.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_png.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_jpeg.a  
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_curl.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_tiff.a
        ${CURL_LIBRARY}
        ${JPEG_LIBRARY}
        ${PNG_LIBRARY}
        ${FREETYPE_LIBRARY}
        ${TIFF_LIBRARY}
        )
    set(MY_LIBRARIES ${MY_LIBRARIES}
        osgdb_earth
        osgdb_osgearth_engine_rex
         )
    message(STATUS  "oe...osgearth_viewer/CMakeLists.txt...2: osgLIBs=${OPENSCENEGRAPH_LIBRARIES};;MY_LIBRARIES=${MY_LIBRARIES}")
endif(OSGEARTH_BUILD_SHARED_LIBS)

add_osgearth_app(
    TARGET osgearth_viewer
    SOURCES osgearth_viewer.cpp
    FOLDER Tools 
    LIBRARIES ${MY_LIBRARIES})
 