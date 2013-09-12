Building osgEarth (and OSG) for iOS
===================================

When building osgEarth for iOS it is important to note that it requires two separate builds to build for both the simulator and devices.  The instructions below are configured to build the libraries for use with devices.  Building for use in the simulator is the same but requires pointing to the proper libraries when building.

The steps to buld for use on devices are as follows:

#. **Install CMake 2.8.8 or above**
     | http://www.cmake.org
     | Or install `MacPorts <http://guide.macports.org/#installing.macports>`_ and then ``sudo ports install cmake``

#. **Install XCode 4.5.1**
     | Download from AppStore

#. **Get Dependencies**
     | Prebuilt dependencies courtesy of `Thomas Hogarth <http://www.hogbox.co.uk>`_ can be downloaded here: https://s3.amazonaws.com/pelican-downloads/ios-3rdParty.zip

#. **Javascript Support**
     | Unlike osgEarth on the desktop which uses the V8 engine, osgEarth on iOS depends on the JavaScriptCore engine for scripting support.
     | The JavaScriptCore code with projects for iOS can be found on GitHub here: https://github.com/phoboslab/JavaScriptCore-iOS
     | As noted on that page, a prebuilt library can be found in the source tree of the `Ejecta project <https://github.com/phoboslab/Ejecta/tree/master/Source/lib>`_

#. **Download OSG trunk**
   ::
      mkdir osgearth-build
      cd ./osgearth-build
      svn checkout http://www.openscenegraph.org/svn/osg/OpenSceneGraph/trunk osg-ios

#. **Edit CMakeList.txt to point to your version of iOS SDK**
     | Check ``/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer/SDKs``
     | Open CMakeList.txt and change line 236 to your SDK version: ``(SET (IPHONE_SDKVER "5.1" CACHE STRING "IOS SDK-Version")``
     | For the lib tiff plugin it is also currently necessary to copy line 587 and paste inside the else statement at line 593 (submited to osg trunk)

#. **Run CMake to generate OSG XCode Project**
     | Use below command line to generate static GLES2 build for iOS devices, change 3rdparty paths to reflect your 3rdparty path
     ::

      cd ./osg-ios

      cmake ./ -G Xcode -DOSG_BUILD_PLATFORM_IPHONE:BOOL=ON \
      -DBUILD_OSG_APPLICATIONS:BOOL=OFF \
      -DOSG_WINDOWING_SYSTEM:STRING=IOS \
      -DOSG_DEFAULT_IMAGE_PLUGIN_FOR_OSX="imageio" \
      -DOSG_GL1_AVAILABLE:BOOL=OFF \
      -DOSG_GL2_AVAILABLE:BOOL=OFF \
      -DOSG_GLES1_AVAILABLE:BOOL=OFF \
      -DOSG_GLES2_AVAILABLE:BOOL=ON \
      -DOSG_GL_DISPLAYLISTS_AVAILABLE:BOOL=OFF \
      -DOSG_GL_FIXED_FUNCTION_AVAILABLE:BOOL=OFF \
      -DOSG_GL_LIBRARY_STATIC:BOOL=OFF \
      -DOSG_GL_MATRICES_AVAILABLE:BOOL=OFF \
      -DOSG_GL_VERTEX_ARRAY_FUNCS_AVAILABLE:BOOL=OFF \
      -DOSG_GL_VERTEX_FUNCS_AVAILABLE:BOOL=OFF \
      -DCURL_INCLUDE_DIR:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/curl-ios-device/include" \
      -DCURL_LIBRARY:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/curl-ios-device/lib/libcurl.a" \
      -DFREETYPE_INCLUDE_DIR_freetype2:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/freetype-ios-universal/include/freetype" \
      -DFREETYPE_INCLUDE_DIR_ft2build:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/freetype-ios-universal/include" \
      -DFREETYPE_LIBRARY:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/freetype-ios-universal/lib/libFreeType_iphone_universal.a" \
      -DTIFF_INCLUDE_DIR:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/tiff-ios-device/include" \
      -DTIFF_LIBRARY:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/tiff-ios-device/lib/libtiff.a" \
      -DGDAL_INCLUDE_DIR:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/gdal-ios-device/include" \
      -DGDAL_LIBRARY:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/gdal-ios-device/lib/libgdal.a" \
      -DDYNAMIC_OPENSCENEGRAPH:BOOL=OFF \
      -DDYNAMIC_OPENTHREADS:BOOL=OFF

#. **Open XCode project generated and run BUILD_ALL target**

#. **Download osgEarth trunk**
   ::
      cd ../
      git clone git://github.com/gwaldron/osgearth.git osgearth-ios

#. **Edit iOS SDK version in ``./osgearth-ios/CMakeList.txt`` (see item 5)**

#. **Run CMake to generate osgEarth Xcode project**
     | Use below command line to generate static bulild for iOS device, change 3rdparty paths to reflect your 3rdparty path
     ::
     
      cmake ./ -G Xcode -DOSG_BUILD_PLATFORM_IPHONE:BOOL=ON \
      -DOSG_DIR:PATH="/Users/hogbox/Documents/osgearth-build/osg-ios" \
      -DCURL_INCLUDE_DIR:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/curl-ios-device/include" \
      -DCURL_LIBRARY:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/curl-ios-device/lib/libcurl.a" \
      -DGDAL_INCLUDE_DIR:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/gdal-ios-device/include" \
      -DGDAL_LIBRARY:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/gdal-ios-device/lib/libgdal.a" \
      -DGEOS_INCLUDE_DIR:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/geos-ios-device/include/source/headers" \
      -DGEOS_LIBRARY:PATH="/Users/hogbox/Documents/osgearth-build/3rdParty/geos-ios-device/lib/libGEOS_3.2.a" \
      -DOSGEARTH_BUILD_APPLICATION_BUNDLES:BOOL=OFF \
      -DDYNAMIC_OSGEARTH:BOOL=OFF \
      -DOSGEARTH_USE_QT:BOOL=OFF

#. **Open generated Xcode project and build**
     | For now do not build application targets as they will generate errors (CMake can not currently generate valid application targets for iOS)
     | Select *OSGEARTH* project in navigator view (top of tree on left)
     | Select *Add Target*
     | Select *Aggregate Target* in the *Other* section, name it lib-build (or whatever)
     | Select new target and select *Build Phases*, *Target Dependancies*, *+*
     | Select all the libs and plugins 
     | Select the new target as the current build target (combo box to right of the play/run button)
     | Build

#. **Open and build example project**
     | ``osgearth-ios/src/applications/osgearth_viewerIOS/osgEarthViewerIOS.xcodeproj``
     | 
     | Edit *Header Search Paths* in build settings point to your osg and osgearth folders
     | Edit *Library Search Paths* in build settings point to your osg and osgearth folders
     | Link to Accelerate.framework (fix sent)
     | Link to MobileCoreServices.framework (fix sent)
     | remove armv7s build if you are using freetypes
     | build
