
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := osgNativeLib
### Main Install dir
NDK_ROOT                := /Users/thomashogarth/Library/Android/sdk/ndk-bundle
OSG_ANDROID_DIR		:= /Users/thomashogarth/Documents/AlphaPixel/osgEarth-Droid/osg/
OSGEARTH_ANDROID_DIR	:= /Users/thomashogarth/Documents/AlphaPixel/osgEarth-Droid/osgearth/
THIRDPARTY_ANDROID_DIR	:= /Users/thomashogarth/Documents/AlphaPixel/osgEarth-Droid/osg/3rdParty

OSG_LIBDIR 			:= $(OSG_ANDROID_DIR)/lib
OSGEARTH_LIBDIR 		:= $(OSGEARTH_ANDROID_DIR)/lib
PNG_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/libpng/obj/local/armeabi
TIFF_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/libtiff/obj/local/armeabi
GDAL_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/gdal/obj/local/armeabi
GEOS_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/geos/obj/local/armeabi
PROJ_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/proj/obj/local/armeabi
CURL_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/curl/obj/local/armeabi
FREETYPE_LIBDIR 		:= $(THIRDPARTY_ANDROID_DIR)/build/freetype/obj/local/armeabi
SQLITE_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/sqlite/obj/local/armeabi
ZLIB_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/zlib/obj/local/armeabi

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
	LOCAL_ARM_NEON 	:= true
	OSG_LIBDIR 			:= $(OSG_ANDROID_DIR)/lib
	OSGEARTH_LIBDIR 		:= $(OSGEARTH_ANDROID_DIR)/lib
	PNG_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/libpng/obj/local/armeabi-v7a
	TIFF_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/libtiff/obj/local/armeabi-v7a
	GDAL_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/gdal/obj/local/armeabi-v7a
	GEOS_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/geos/obj/local/armeabi-v7a
	PROJ_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/proj/obj/local/armeabi-v7a
	CURL_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/curl/obj/local/armeabi-v7a
	FREETYPE_LIBDIR 		:= $(THIRDPARTY_ANDROID_DIR)/build/freetype/obj/local/armeabi-v7a
	SQLITE_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/sqlite/obj/local/armeabi-v7a
	ZLIB_LIBDIR 			:= $(THIRDPARTY_ANDROID_DIR)/build/zlib/obj/local/armeabi-v7a
endif

### Add all source file names to be included in lib separated by a whitespace

LOCAL_C_INCLUDES:= $(OSG_ANDROID_DIR)/include $(OSGEARTH_ANDROID_DIR)/src
LOCAL_CFLAGS    := -Werror -fno-short-enums
LOCAL_CPPFLAGS  := -DOSG_LIBRARY_STATIC -DOSGEARTH_LIBRARY_STATIC

LOCAL_LDLIBS    := -llog -lGLESv3 -lz -ldl -lgnustl_static -lsupc++

LOCAL_SRC_FILES := osgNativeLib.cpp OsgMainApp.cpp OsgAndroidNotifyHandler.cpp


#-losgdb_osgearth_agglite \
#-losgdb_tiff \
#-losgdb_png \
#-losgdb_zip \

LOCAL_LDFLAGS   += -L$(NDK_ROOT)/sources/cxx-stl/gnu-libstdc++/4.9/libs/armeabi-v7a -lgnustl_static -lsupc++ \
-L $(OSGEARTH_LIBDIR) \
-losgdb_osgearth_arcgis \
-losgdb_osgearth_bing \
-losgdb_osgearth_bumpmap \
-losgdb_osgearth_cache_filesystem \
-losgdb_osgearth_colorramp \
-losgdb_osgearth_debug \
-losgdb_osgearth_detail \
-losgdb_earth \
-losgdb_osgearth_engine_mp \
-losgdb_osgearth_engine_rex \
-losgdb_osgearth_feature_elevation \
-losgdb_osgearth_feature_ogr \
-losgdb_osgearth_feature_tfs \
-losgdb_osgearth_feature_wfs \
-losgdb_osgearth_feature_xyz \
-losgdb_osgearth_featurefilter_intersect \
-losgdb_osgearth_featurefilter_join \
-losgdb_osgearth_gdal \
-losgdb_kml \
-losgdb_osgearth_label_annotation \
-losgdb_osgearth_mapinspector \
-losgdb_osgearth_mask_feature \
-losgdb_osgearth_mbtiles \
-losgdb_osgearth_model_feature_geom \
-losgdb_osgearth_model_simple \
-losgdb_osgearth_monitor \
-losgdb_osgearth_ocean_simple \
-losgdb_osgearth_osg \
-losgdb_osgearth_scriptengine_javascript \
-losgdb_osgearth_sky_gl \
-losgdb_osgearth_sky_simple \
-losgdb_osgearth_skyview \
-losgdb_template \
-losgdb_osgearth_terrainshader \
-losgdb_osgearth_tileindex \
-losgdb_osgearth_tms \
-losgdb_osgearth_vdatum_egm84 \
-losgdb_osgearth_vdatum_egm96 \
-losgdb_osgearth_vdatum_egm2008 \
-losgdb_osgearth_viewpoints \
-losgdb_osgearth_vpb \
-losgdb_osgearth_wcs \
-losgdb_osgearth_wms \
-losgdb_osgearth_xyz \
-losgEarthAnnotation \
-losgEarthFeatures \
-losgEarthSymbology \
-losgEarthUtil \
-losgEarth \
-L $(OSG_LIBDIR) \
-losgdb_openflight \
-losgdb_curl \
-losgdb_obj \
-losgdb_shp \
-losgdb_rot \
-losgdb_scale \
-losgdb_trans \
-losgdb_jpeg \
-losgdb_freetype \
-losgdb_osgterrain \
-losgdb_osg \
-losgdb_ive \
-losgdb_deprecated_osgviewer \
-losgdb_deprecated_osgvolume \
-losgdb_deprecated_osgtext \
-losgdb_deprecated_osgterrain \
-losgdb_deprecated_osgsim \
-losgdb_deprecated_osgshadow \
-losgdb_deprecated_osgparticle \
-losgdb_deprecated_osgfx \
-losgdb_deprecated_osganimation \
-losgdb_deprecated_osg \
-losgdb_serializers_osgvolume \
-losgdb_serializers_osgtext \
-losgdb_serializers_osgterrain \
-losgdb_serializers_osgsim \
-losgdb_serializers_osgshadow \
-losgdb_serializers_osgparticle \
-losgdb_serializers_osgmanipulator \
-losgdb_serializers_osgfx \
-losgdb_serializers_osganimation \
-losgdb_serializers_osg \
-losgViewer \
-losgVolume \
-losgTerrain \
-losgText \
-losgShadow \
-losgSim \
-losgParticle \
-losgManipulator \
-losgGA \
-losgFX \
-losgDB \
-losgAnimation \
-losgUtil \
-losg \
-lOpenThreads \
-L $(PNG_LIBDIR) \
-lpng \
-L $(FREETYPE_LIBDIR) \
-lft2 \
-L $(CURL_LIBDIR) \
-lcurl \
-L $(GDAL_LIBDIR) \
-lgdal \
-L $(GEOS_LIBDIR) \
-lgeos \
-L $(PROJ_LIBDIR) \
-lproj \
-L $(SQLITE_LIBDIR) \
-lsqlite3 \
-L $(ZLIB_LIBDIR) \
-lzlib

include $(BUILD_SHARED_LIBRARY)
