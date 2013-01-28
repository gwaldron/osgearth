Building osgEarth (and OSG) for iOS
===================================

When building osgEarth for iOS it is important to note that it requires two separate builds to build for both the simulator and devices.  The instructions below are configured to build the libraries for use with devices.  Building for use in the simulator is the same but requires pointing to the proper libraries when building.

The steps to buld for use on devices are as follows:

#. **Install CMake 2.8.8 or above**
   http://www.cmake.org/
   Or install MacPorts http://guide.macports.org/#installing.macports
   sudo ports install cmake

#. Install XCode 4.5.1
   Download from AppStore

#. Get Dependencies (currently using prebuilt)
   www.hogbox.co.uk/data/ios-3rdParty.zip
   Install for me osgearth-build/3rdparty

#. Download OSG trunk
   mkdir osgearth-build
   cd ./osgearth-build
   svn checkout http://www.openscenegraph.org/svn/osg/OpenSceneGraph/trunk osg-ios

**TO BE CONTINUED**







