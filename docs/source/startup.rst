Building osgEarth
=================

osgEarth is a cross-platform library. It uses the CMake_ build system.
You will need **version 2.8** or newer. 
(This is the same build system that OpenSceneGraph_ uses.)

    NOTE: To build osgEarth for **iOS** see :doc:`ios`

Get the Source Code
-------------------

**Option 1: use GIT**

    osgEarth is hosted on GitHub_. You will need a *git* client to access it.
    We recommend TortoiseGit_ for Windows users.

    To clone the repository, point your client at::

        git://github.com/gwaldron/osgearth.git

**Option 2: download a tagged version**

    To download a tarball or ZIP archive of the source code, visit the
    `osgEarth Tags`_ and select the one you want. The latest official release
    will be at the top.


Get the Dependencies
--------------------

**Required dependencies**:

    * OpenSceneGraph_ 3.0.1 or later, with the CURL plugin enabled.
    * GDAL_ 1.6 or later - Geospatial Data Abstraction Layer
    * CURL_ - HTTP transfer library (comes with OpenSceneGraph_ 3rd party library distros)
    
**Optional depedencies**: osgEarth will compile without them, but some functionality
will be missing:

    * GEOS_ 3.2.0 or later - C++ library for topological operations.
      osgEarth uses GEOS to perform various geometry operations like buffering and intersections.
      If you plan to use vector feature data in osgEarth, you probably want this.
    
    * Minizip_ - ZIP file extractor; include this if you want to read KMZ files.
      
    * QT_ - Cross-platform UI framework. Point the ``QT_QMAKE_EXECUTABLE`` CMake variable
      to the ``qmake.exe`` you want to use and CMake will populate all the other QT variables.

    * LevelDB_ - Google's embedded key/value store. Include this if you want to build
      osgEarth's optional "leveldb" cache driver.

    * Duktape_ - Embedded JavaScript engine. Include this is you want to embed JavaScript
      code in your earth files to control feature styling.
    
**Deprecated dependencies**: osgEarth can still use these, but they will probably go away
in the future:

    * V8_ - Google's JavaScript engine. Include this if you're a Windows user and you want
      to embed JavaScript code in your earth files. We recommend you use Duktape instead.
      
    * JavaScriptCore_ - Apple's JavaScript engine. Include this if you're an OSX or IOS user
      and you want to embed JavaScript code in your earth files. We receommend you use
      Duktape instead.
      
**Optional: get pre-built dependencies**

    * AlphaPixel_ has pre-built OSG_ and 3rd-party dependencies for various architectures.
    * `Mike Weiblen`_ has some pre-built OSG_ binaries and dependencies too.
    * FWTools_ has pre-built GDAL binaries with all the fixins.
    * Pre-built `GDAL binaries`_ for various architectures.
    
Build it
--------

Make sure you built OSG_ and all the dependencies first.

osgEarth uses CMake_, version 2.8 or later. Since OSG_ uses CMake_ as well, 
once you get OSG built the process should be familiar.

Here are a few tips.

    * Always do an "out-of-source" build with CMake. That is, use a build directory
      that is separate from the source code. This makes it easier to maintain separate
      versions and to keep GIT updates clean.
      
    * For optional dependencies (like GEOS_ or V8_), just leave the CMake field blank
      if you are not using it.
      
    * For the OSG dependencies, just input the **OSG_DIR** variable, and when you generate
      CMake will automatically find all the other OSG directories.
      
    * As always, check `the forum`_ if you have problems!
  
**Good luck!!**

----
*Copyright Pelican Mapping Inc.*

.. _GitHub:         http://github.com/gwaldron/osgearth
.. _TortoiseGit:    http://code.google.com/p/tortoisegit
.. _CMake:          http://www.cmake.org
.. _osgEarth Tags:  http://github.com/gwaldron/osgearth/tags
.. _OpenSceneGraph: http://openscenegraph.org
.. _OSG:            http://openscenegraph.org
.. _CURL:           http://curl.haxx.se/libcurl/
.. _GEOS:           http://trac.osgeo.org/geos/ 
.. _GDAL:           http://www.gdal.org/
.. _GDAL binaries:  http://vbkto.dyndns.org:1280/sdk/Default.aspx
.. _FWTools:        http://fwtools.maptools.org/
.. _Minizip:        http://www.winimage.com/zLibDll/minizip.html
.. _V8:             http://code.google.com/p/v8/
.. _AlphaPixel:     http://openscenegraph.alphapixel.com/osg/downloads/openscenegraph-third-party-library-downloads
.. _Mike Weiblen:   http://mew.cx/osg/
.. _the forum:      http://forum.osgearth.org
.. _LevelDB:        https://github.com/pelicanmapping/leveldb
.. _Duktape:        http://duktape.org
