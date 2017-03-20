MACRO(LINK_OSG_STATIC_PLUGINS)
    FOREACH(LINKLIB ${OSG_STATIC_PLUGINS})
        SET(OSG_PLUGINS_PATH "${OSG_DIR}/lib")
        TARGET_LINK_LIBRARIES(${TARGET_TARGETNAME} optimized "${OSG_PLUGINS_PATH}/lib${LINKLIB}.a" debug "${OSG_PLUGINS_PATH}/lib${LINKLIB}${CMAKE_DEBUG_POSTFIX}.a")
    ENDFOREACH(LINKLIB)
ENDMACRO(LINK_OSG_STATIC_PLUGINS)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )
SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGGA_LIBRARY OSGANIMATION_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY 
                          CURL_LIBRARY FREETYPE_LIBRARY GDAL_LIBRARY PROJ_LIBRARY GEOS_LIBRARY)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthUtil)
SET(TARGET_ADDED_LIBRARIES osgdb_earth osgdb_osgearth_gdal osgdb_osgearth_tms osgdb_osgearth_engine_mp)
SET(OSG_STATIC_PLUGINS osgdb_imageio osgdb_curl osgdb_zip osgdb_freetype osgdb_osg)

SET(RESOURCE_FILES  
    StartViewerController.xib
    ViewController_iPhone.xib
    ViewController_iPad.xib
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
-framework JavaScriptCore -framework Security -lsqlite3 -licucore -lz -liconv")

#restore setting
SET(OSGEARTH_BUILD_APPLICATION_BUNDLES {$TMP_OSGEARTH_BUILD_APPLICATION_BUNDLES})
SET(VPB_BUILD_APPLICATION_BUNDLES {$TMP_VPB_BUILD_APPLICATION_BUNDLES})
SET(CMAKE_OSX_ARCHITECTURES {$TMP_CMAKE_OSX_ARCHITECTURES})