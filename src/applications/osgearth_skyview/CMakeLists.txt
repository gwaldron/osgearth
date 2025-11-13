if(OSGEARTH_BUILD_SHARED_LIBS) 
    message(STATUS  "oe...osgearth_skyview/CMakeLists.txt: OSGEARTH_BUILD_SHARED_LIBS=1;osgLIBs=${OPENSCENEGRAPH_LIBRARIES}")
   
else()
    set(OPENSCENEGRAPH_LIBRARIES 
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_3dc.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_ac.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_bmp.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_bsp.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_bvh.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_cfg.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_curl.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_dds.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osg.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osganimation.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgfx.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgparticle.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgshadow.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgsim.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgterrain.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgtext.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgviewer.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgvolume.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_deprecated_osgwidget.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_dot.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_dxf.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_freetype.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_gdal.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_gles.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_glsl.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_gz.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_hdr.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_ive.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_jpeg.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_ktx.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_logo.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_lua.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_lws.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_md2.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_mdl.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_normals.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_obj.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_ogr.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_openflight.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osc.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osg.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osga.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osgjs.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osgshadow.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osgterrain.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osgtgz.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_osgviewer.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_p3d.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_pic.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_png.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_pnm.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_pov.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_pvr.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_revisions.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_rgb.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_rot.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_scale.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osg.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osganimation.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgfx.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgga.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgmanipulator.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgparticle.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgshadow.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgsim.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgterrain.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgtext.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgui.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgutil.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgviewer.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_serializers_osgvolume.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_shp.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_stl.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_tf.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_tga.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_tgz.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_tiff.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_trans.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_trk.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_txf.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_vtf.a
        ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_x.a
        # -------------------------------------------
        # ### do not use "osg/**/libosgdb_zip.a", but use "osgearth/**/osgdb_zip.a". 
        # ${OSG_DIR}/lib/osgPlugins-3.7.0/libosgdb_zip.a   
        # -------------------------------------------             
        ${OSG_DIR}/lib/libosgPresentation.a              
        ${OSG_DIR}/lib/libosgUI.a        
        ${OSG_DIR}/lib/libosgWidget.a 
        ${OSG_DIR}/lib/libosgViewer.a    
        ${OSG_DIR}/lib/libosgVolume.a
        ${OSG_DIR}/lib/libosgTerrain.a
        ${OSG_DIR}/lib/libosgText.a
        ${OSG_DIR}/lib/libosgShadow.a
        ${OSG_DIR}/lib/libosgSim.a
        ${OSG_DIR}/lib/libosgParticle.a
        ${OSG_DIR}/lib/libosgManipulator.a
        ${OSG_DIR}/lib/libosgGA.a
        ${OSG_DIR}/lib/libosgFX.a 
        ${OSG_DIR}/lib/libosgDB.a
        ${OSG_DIR}/lib/libosgAnimation.a 
        ${OSG_DIR}/lib/libosgUtil.a 
        ${OSG_DIR}/lib/libosg.a 
        ${OSG_DIR}/lib/libOpenThreads.a       
        ${GDAL_LIBRARY}
        ${CURL_LIBRARIES}
        ${FREETYPE_LIBRARY} 
        ${TIFF_LIBRARY} 
        ${JPEG_LIBRARY} 
        ${PNG_LIBRARY} 
        ${PROJ_LIBRARY} 
        ${GEOS_LIBRARY} 
        ${OPENSSL_SSL_LIBRARY} 
        ${OPENSSL_CRYPTO_LIBRARY} 
        # protobuf::libprotobuf-lite 
        # protobuf::libprotobuf
        # protobuf::libupb  
        # utf8_range::utf8_range  
        # utf8_range::utf8_validity
        ${SQLite3_LIBRARIES}  
        ${ZLIB_LIBRARY}   
        # libzip::zip   
        ${LIBLZMA_LIBRARY}         
        )

    set(MY_LIBRARIES ${MY_LIBRARIES}
        osgdb_osgearth_bumpmap
        osgdb_osgearth_cache_filesystem
        osgdb_osgearth_colorramp
        osgdb_osgearth_detail
        osgdb_osgearth_engine_rex
        osgdb_osgearth_featurefilter_intersect
        osgdb_osgearth_featurefilter_join
        osgdb_osgearth_scriptengine_javascript
        osgdb_osgearth_sky_gl
        osgdb_osgearth_sky_simple
        osgdb_osgearth_terrainshader
        osgdb_osgearth_vdatum_egm2008
        osgdb_osgearth_vdatum_egm84
        osgdb_osgearth_vdatum_egm96
        osgdb_osgearth_viewpoints
        osgdb_template
        osgdb_zip
        osgdb_gltf
        osgdb_kml
        osgdb_earth
        osgEarth
 
         )
    message(STATUS  "oe...osgearth_city/CMakeLists.txt;;OSGEARTH_BUILD_SHARED_LIBS=0: osgLIBs=${OPENSCENEGRAPH_LIBRARIES}")
    message(STATUS  "oe...osgearth_city/CMakeLists.txt;;OSGEARTH_BUILD_SHARED_LIBS=0: MY_LIBs=${MY_LIBRARIES}")
endif(OSGEARTH_BUILD_SHARED_LIBS)


add_osgearth_app(
    TARGET osgearth_skyview
    HEADERS SkyManipulator
    SOURCES SkyManipulator.cpp osgearth_skyview.cpp
    FOLDER Examples 
    LIBRARIES ${MY_LIBRARIES} )
