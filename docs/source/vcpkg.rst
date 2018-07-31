Building osgEarth for Windows using vcpkg
=========================================
vcpkg_ is an extremly useful C++ package manager.  It works on Windows, Linux and MacOS but for this guide we'll primarily focus on Windows.

First, download and bootstrap vcpkg_ following the instructions on the page.

Next install the dependencies required to build a fully functional osgearth
::
  vcpkg install osg:x64-windows sqlite3:x64-windows protobuf:x64-windows poco:x64-windows

This will take awhile the first time you run it as this pulls down lots of dependencies, so go get a cup of coffee.

Once all the dependencies are built, you'll need to actually build osgearth.

**Get the source code**
::
  git clone https://github.com/gwaldron/osgearth.git


**Create a directory for an out of source build**
::
  cd osgearth
  mkdir build
  cd build


**Configure Cmake**

vcpkg provides a Cmake toolchain file that helps osgEarth find all of it's dependencies.  You'll need to specify a different build directory for Release and Debug and specify the build type using -DCMAKE_BUILD_TYPE.  This is because some dependencies of osgEarth don't pick up both debug and release versions without specifying the build type.  This should be fixed in future cmake versions.  This is for a release build
::
  cmake .. -G "Visual Studio 15 2017 Win64" \
  -DCMAKE_BUILD_TYPE=Release \
  -DWIN32_USE_MP=ON \
  -DCMAKE_TOOLCHAIN_FILE=[vcpkg root]\scripts\buildsystems\vcpkg.cmake


**Build and install osgEarth**

You can build and install osgEarth on the command line using cmake or you can open up the Visual Studio solution and build it from there.
::
  cmake --build . --target INSTALL --config Release

**Setting up your runtime environment**

You'll need to make sure that the vcpkg dependencies and osgEarth are in your path.  So do something like this
::
  set PATH=%PATH%;c:\vcpkg\installed\x64-windows\bin
  set PATH=%PATH%;c:\vcpkg\installed\x64-windows\tools\osg
  set PATH=%PATH%;c:\Program Files\osgEarth\bin


Note:  If you don't want to build osgEarth yourself for your application, you can actually install it using vcpkg as well.  Just use
::
  vcpkg install osgearth:x64-windows

.. _vcpkg:          https://github.com/Microsoft/vcpkg
