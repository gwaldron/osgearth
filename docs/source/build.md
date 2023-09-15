# Building osgEarth from Scratch

Normally it is sufficient to install osgEarth from `vcpkg` and use it in your appplication. However, if you want to contribute to the project or make local modifications, you will need to build osgEarth yourself.

The documentation here is focused on Windows. 

## Building with vcpkg

[vcpkg](https://github.com/Microsoft/vcpkg) is an extremely useful package manager. It works on Windows, Linux and MacOS but for this guide we will focus on Windows.

**Step 1 - Configure vcpkg**

First, download and bootstrap [vcpkg](https://github.com/Microsoft/vcpkg) following the instructions on the page.

**Step 2 - Set the osg_OPENGL_PROFILE triplet variable to GL3**

Next, you will need to set a triplet variable to ensure that osg will be built with modern OpenGL features enabled that osgEarth requires.

The latest (as of 6/6/2023) version of vcpkg will build OSG with OPENGL_PROFILE=GL2 by default.  This is not sufficient for running osgEarth as it does not enable all modern OpenGL features that osgEarth requires and keeps the deprecated fixed function pipeline path in OSG.

Open your existing x64-windows.cmake triplet file at path\to\vcpkg\triplets\x64-windows.cmake and add this line to the end of the file.
```
set(osg_OPENGL_PROFILE GL3)
```

This will build osg with modern OpenGL features and remove the fixed function pipeline.

If you wish to build OSG with a different OPENGL_PROFILE such as GLCORE you can set the osg_OPENGL_PROFILE to GLCORE like this
```
set(osg_OPENGL_PROFILE GLCORE)
```
When you install osg using vcpkg with this variable set it will build osg against the <GL/glcorearb.h> headers instead of the usual <GL/gl.h> header.

**Step 3 - Clone the repository**

Pull down the source from GitHub and create a ```build``` folder for your out-of-source build. We always recommend doing an out-of-source build to avoid problems down the road!

```
git clone --recurse-submodules https://github.com/gwaldron/osgearth.git osgearth
mkdir build
```

This will clone the repository into a folder called `osgearth` and pull down all the submodules.

**Step 4 - Configure CMake**

On Windows, you can run the provided script `bootstrap-vcpkg.bat` to configure your CMake build. This can take a while since it needs to download and build all your dependencies.

Otherwise, follow these steps:

vcpkg provides a CMake toolchain file that helps osgEarth find all of its dependencies.

Note: You’ll need to specify a different build directory based on your build configuration (Release, RelWIthDebInfo, Debug) and specify the build type using ```-DCMAKE_BUILD_TYPE```. This is because some dependencies of osgEarth don’t pick up both debug and release versions without specifying the build type. Hopefully this will be fixed in future CMake versions.

Most developers will use a RelWithDebInfo build, like so:

```
cmake -S osgearth -B build -G "Visual Studio 15 2017 Win64" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DWIN32_USE_MP=ON -DCMAKE_INSTALL_PREFIX=[installroot] -DCMAKE_TOOLCHAIN_FILE=[vcpkgroot]\scripts\buildsystems\vcpkg.cmake
```

osgEarth provides a vcpkg.json manifest file that lists all of it's necessary dependencies.  The vcpkg toolchain integration will notice this file and install the necessary dependencies in your build\vcpkg_installed directory.

**Step 5 - Build and install osgEarth**

You can build and install osgEarth on the command line using CMake or you can open up the Visual Studio solution and build it from there.

```
cmake --build build --target INSTALL --config RelWithDebInfo
```

**Step 6 - Set up your runtime environment**

You’ll need to make sure that the vcpkg dependencies and osgEarth are in your path:

```
set PATH=%PATH%;path\to\build\vcpkg_installed\x64-windows\bin
set PATH=%PATH%;path\to\build\vcpkg_installed\x64-windows\tools\osg
set PATH=%PATH%;[installroot]
```

## Building with support for cesium-native
See documentation at [here](cesium_native.html) for building osgEarth with support for cesium-native.

## Checking for an OpenGL Core Profile Context
Some situations require you to have an OpenGL Core Profile context.  The ability to create a core context is available when OSG is built with OPENGL_PROFILE=GL3 or GLCORE.  Environments such as Apple OSX and VMWare require it as does debugging with tools like NVidia NSight.  You can check to see if you are running with an OpenGL Core Profile by running a command like this (Windows)
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
