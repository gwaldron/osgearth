# Building osgEarth

The documentation here is focused on Windows. 

## Building with vcpkg

[vcpkg](https://github.com/Microsoft/vcpkg) is an extremely useful package manager. It works on Windows, Linux and MacOS but for this guide we will focus on Windows.

**Step 1 - Configure vcpkg**

First, download and bootstrap [vcpkg](https://github.com/Microsoft/vcpkg) following the instructions on the page.

Next install the dependencies required to build a fully functional osgEarth. This example assume s 64-bit Windows build; you can alter that to correspond to your platform/architecture of choice.

Install the required dependencies:

```
vcpkg install osg:x64-windows gdal:x64-windows curl:x64-windows
```

For full functionality, you can install optional dependences as well:

```
vcpkg install sqlite3:x64-windows protobuf:x64-windows geos:x64-windows blend2d:x64-windows libwebp:x64-windows basisu:x64-windows draco:x64-windows libzip:x64-windows
```

This will take awhile the first time you run it as this pulls down lots of dependencies, so go get a cup of coffee.

Once all the dependencies are built, you’ll need to actually build osgEarth.

**Step 2 - Clone the repository**

Pull down the source from GitHub and create a ```build``` folder for your out-of-source build. We always recommend doing an out-of-source build to avoid problems down the road!

```
git clone --recurse-submodules https://github.com/gwaldron/osgearth.git osgearth
mkdir build
```

This will clone the repository into a folder called `osgearth` and pull down all the submodules.

**Step 3 - Configure CMake**

vcpkg provides a CMake toolchain file that helps osgEarth find all of its dependencies.

Note: You’ll need to specify a different build directory based on your build configuration (Release, RelWIthDebInfo, Debug) and specify the build type using ```-DCMAKE_BUILD_TYPE```. This is because some dependencies of osgEarth don’t pick up both debug and release versions without specifying the build type. Hopefully this will be fixed in future CMake versions.

Most developers will use a RelWithDebInfo build, like so:

```
cmake -S osgearth -B build -G "Visual Studio 15 2017 Win64" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DWIN32_USE_MP=ON -DCMAKE_INSTALL_PREFIX=[installroot] -DCMAKE_TOOLCHAIN_FILE=[vcpkgroot]\scripts\buildsystems\vcpkg.cmake
```

**Step 4 - Build and install osgEarth**

You can build and install osgEarth on the command line using CMake or you can open up the Visual Studio solution and build it from there.

```
cmake --build build --target INSTALL --config RelWithDebInfo
```

**Step 5 - Set up your runtime environment**

You’ll need to make sure that the vcpkg dependencies and osgEarth are in your path:

```
set PATH=%PATH%;c:\vcpkg\installed\x64-windows\bin
set PATH=%PATH%;c:\vcpkg\installed\x64-windows\tools\osg
set PATH=%PATH%;[installroot]
```

## Building for OpenGL CORE Profile

You may wish to build osgEarth with support for the OpenGL CORE profile. In fact is a requirement for some platforms including Apple OSX and VMWare. Doing to requires that you first build OpenSceneGraph with CORE profile support. The OpenSceneGraph dependency in *vcpkg* does NOT have GLCORE support (at the time of this writing) so you will have to build it yourself. 

#### Build OpenSceneGraph for GLCORE

1. First, [download the GL CORE include files from Khronos](https://www.khronos.org/registry/OpenGL/api/GL) and place them somewhere on your system. We'll call this the GLCORE folder.
2. In CMake, set the `OPENGL_PROFILE` property to "GLCORE".
3. In CMake, set the `GLCORE_GLCOREARB_HEADER` property to the location of the GL folder you downloaded from Khronos. For example, if you include file is at `C:\glcore\GL\glcorearb.h` you should set this property to `C:\glcore`.
4. In CMake, set the following properties to `ON` :
   * `OSG_GL3_AVAILABLE`
5. In CMake, set the following properties to `OFF` :
   * `OSG_GL1_AVAILABLE`
   * `OSG_GL2_AVAILABLE`
   * `OSG_GLES1_AVAILABLE`
   * `OSG_GLES2_AVAILABLE`
   * `OSG_GL_DISPLAYLISTS_AVAILABLE`
   * `OSG_GL_FIXED_FUNCTION_AVAILABLE`
   * `OSG_GL_MATRICES_AVAILABLE`
   * `OSG_GL_VERTEX_ARRAY_FUNCS_AVAILABLE`
   * `OSG_GL_VERTEX_FUNCS_AVAILABLE`
6. Configure and build OpenSceneGraph.

#### Build osgEarth for GLCORE

Now that you have OSG built with GLCORE support, time to build osgEarth.

1. In CMake, set the `OSGEARTH_GLCORE_INCLUDE_DIR` property to the same folder holding the Khronos include files (the same value of the `GLCORE_GLCOREARB_HEADER` in your OSG build).
2. Configure and build osgEarth.

Test you build by running this on the command line (Windows)

```
set OSG_GL_CONTEXT_VERSION=4.6
osgearth_version --caps
```

If all went well, it should report **"Core Profile = yes"**.

You can disable the CORE profile and select a compatibility profile by setting a profile mask like so

```
set OSG_GL_CONTEXT_PROFILE_MASK=1
```

The context version and profile mask are also settable via the `osg::DisplaySettings` class in the OpenSceneGraph API.

## Tips for VMware Users

Running osgEarth in a virtual machine environment can be tricky since they usually don't have direct access to the graphics hardware by default. If you are having trouble you can try these tips.

First, build OSG and osgEarth for GL CORE profile (as above). 

Next, assess the situation with a capabilities check:

```
osgearth_version --caps
```

The output will look something like this:

```
GPU Vendor:        WMware, Inc.
GPU Renderer       Gallium 0.3 on llvmpipe
GL/Driver Version: 1.2 Mesa 11.2.0
```

If is reports a Mesa driver, and the version is less than 3.3, you will need to configure a couple environment variables to move forward (Windows):

```
set OSG_GL_CONTEXT_VERSION=3.3
set MESA_GL_VERSION_OVERRIDE=3.3
osgearth_version --caps
```

Good luck!
