MACRO(LINK_OSG_STATIC_PLUGINS)
    FOREACH(LINKLIB ${OSG_STATIC_PLUGINS})
        SET(OSG_PLUGINS_PATH "${OSG_DIR}/lib")
        TARGET_LINK_LIBRARIES(${TARGET_TARGETNAME} optimized "${OSG_PLUGINS_PATH}/lib${LINKLIB}.a" debug "${OSG_PLUGINS_PATH}/lib${LINKLIB}${CMAKE_DEBUG_POSTFIX}.a")
    ENDFOREACH(LINKLIB)
ENDMACRO(LINK_OSG_STATIC_PLUGINS)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )

SET(TARGET_LIBRARIES_VARS OPENTHREADS_LIBRARY OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGGA_LIBRARY OSGVIEWER_LIBRARY
                          OSGANIMATION_LIBRARY OSGFX_LIBRARY OSGPARTICLE_LIBRARY OSGSHADOW_LIBRARY OSGSIM_LIBRARY OSGTERRAIN_LIBRARY OSGTEXT_LIBRARY
                          CURL_LIBRARY FREETYPE_LIBRARY GDAL_LIBRARY PROJ_LIBRARY GEOS_LIBRARY SQLITE3_LIBRARY)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthUtil)

SET(TARGET_ADDED_LIBRARIES osgdb_osgearth_agglite
osgdb_osgearth_arcgis
osgdb_osgearth_bing
osgdb_osgearth_bumpmap
osgdb_osgearth_cache_filesystem
osgdb_osgearth_colorramp
osgdb_osgearth_debug
osgdb_osgearth_detail
osgdb_earth
osgdb_osgearth_engine_mp
osgdb_osgearth_engine_rex
osgdb_osgearth_feature_elevation
osgdb_osgearth_feature_ogr
osgdb_osgearth_feature_tfs
osgdb_osgearth_feature_wfs
osgdb_osgearth_feature_xyz
osgdb_osgearth_featurefilter_intersect
osgdb_osgearth_featurefilter_join
osgdb_osgearth_gdal
osgdb_kml
osgdb_osgearth_label_annotation
osgdb_osgearth_mapinspector
osgdb_osgearth_mask_feature
osgdb_osgearth_mbtiles
osgdb_osgearth_model_feature_geom
osgdb_osgearth_model_simple
osgdb_osgearth_monitor
osgdb_osgearth_osg
osgdb_osgearth_scriptengine_javascript
osgdb_osgearth_sky_gl
osgdb_osgearth_sky_simple
osgdb_osgearth_skyview
osgdb_template
osgdb_osgearth_terrainshader
osgdb_osgearth_tileindex
osgdb_osgearth_tms
osgdb_osgearth_vdatum_egm84
osgdb_osgearth_vdatum_egm96
osgdb_osgearth_vdatum_egm2008
osgdb_osgearth_viewpoints
osgdb_osgearth_vpb
osgdb_osgearth_wcs
osgdb_osgearth_wms
osgdb_osgearth_xyz)

SET(OSG_STATIC_PLUGINS osgdb_tiff osgdb_rgb osgdb_imageio osgdb_curl osgdb_zip osgdb_freetype osgdb_osg osgdb_ive osgdb_obj osgdb_shp osgdb_openflight osgdb_rot osgdb_scale osgdb_trans)

SET(OSG_STATIC_PLUGINS ${OSG_STATIC_PLUGINS}
    osgdb_deprecated_osg osgdb_deprecated_osganimation osgdb_deprecated_osgfx
    osgdb_deprecated_osgparticle osgdb_deprecated_osgshadow osgdb_deprecated_osgsim
    osgdb_deprecated_osgterrain osgdb_deprecated_osgtext osgdb_deprecated_osgviewer
    osgdb_deprecated_osgvolume
)


SET(OSG_STATIC_PLUGINS ${OSG_STATIC_PLUGINS}
    osgdb_serializers_osg osgdb_serializers_osganimation osgdb_serializers_osgfx
    osgdb_serializers_osgmanipulator osgdb_serializers_osgparticle
    osgdb_serializers_osgshadow osgdb_serializers_osgsim osgdb_serializers_osgterrain
    osgdb_serializers_osgtext osgdb_serializers_osgutil osgdb_serializers_osgviewer
    osgdb_serializers_osgvolume
)

SET(RESOURCE_FILES  
    StartViewerController.xib
    ViewController.xib
    ${CMAKE_SOURCE_DIR}/tests/readymap.earth
)

SET(TARGET_SRC
    AppDelegate.h
    AppDelegate.mm
    ViewController.h
    ViewController.mm
    StartViewerController.h
    StartViewerController.mm
    main.mm
    osgPlugins.h
    osgEarthViewerIOS-Info.plist
    ${RESOURCE_FILES}
)


#backup setting
SET(TMP_OSGEARTH_BUILD_APPLICATION_BUNDLES {$OSGEARTH_BUILD_APPLICATION_BUNDLES}) # make sure its an app bundle
SET(OSGEARTH_BUILD_APPLICATION_BUNDLES TRUE)

SET(TMP_VPB_BUILD_APPLICATION_BUNDLES {$VPB_BUILD_APPLICATION_BUNDLES}) # make sure its an app bundle
SET(VPB_BUILD_APPLICATION_BUNDLES TRUE)

SET(TMP_CMAKE_OSX_ARCHITECTURES {$CMAKE_OSX_ARCHITECTURES}) # for now exclude armv7s as freetypes is missing it
SET(CMAKE_OSX_ARCHITECTURES "armv7;arm64")

SET(MACOSX_DEPLOYMENT_TARGET, ${IPHONE_VERSION_MIN})

SETUP_EXAMPLE(osgearth_viewer_ios)
LINK_OSG_STATIC_PLUGINS()

SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES XCODE_ATTRIBUTE_PRODUCT_TYPE "com.apple.product-type.application"
                                                                 XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "iPhone Developer"
                                                                 XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET ${IPHONE_VERSION_MIN}
                                                                 XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2"
                                                                 XCODE_ATTRIBUTE_ENABLE_BITCODE "NO")
SET(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf")
SET_SOURCE_FILES_PROPERTIES(${RESOURCE_FILES} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)


SET(CMAKE_EXE_LINKER_FLAGS "-framework QuartzCore -framework Foundation -framework UIKit 
-framework ImageIO -framework CoreImage -framework Accelerate -framework MobileCoreServices -framework CoreGraphics 
-framework JavaScriptCore -framework Security -lsqlite3 -licucore -lz -liconv -lbz2")

#restore setting
SET(OSGEARTH_BUILD_APPLICATION_BUNDLES {$TMP_OSGEARTH_BUILD_APPLICATION_BUNDLES})
SET(VPB_BUILD_APPLICATION_BUNDLES {$TMP_VPB_BUILD_APPLICATION_BUNDLES})

SET(CMAKE_OSX_ARCHITECTURES {$TMP_CMAKE_OSX_ARCHITECTURES})
