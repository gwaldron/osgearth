
# Building osgEarth with cesium-native

Pelican Mapping is proud to have been awarded a [Cesium Ecosystem Grant](https://cesium.com/cesium-ecosystem-grants/) to integrate [cesium-native](https://github.com/CesiumGS/cesium-native) into osgEarth!

cesium-native is a library developed by the Cesium team that provides support for loading 3d tiles datasets and as well as loading assets from Cesium Ion.  cesium-native is what powers Cesium for Unreal and Cesium for Omniverse and it is now available in osgEarth.

## Building osgEarth with cesium-native support

### Building cesium-native
First, you need to build cesium-native.  The official instructions for building cesium-native are [here](https://github.com/CesiumGS/cesium-native) but this is what it would generally look like
```
# Clone the cesium-native repo
git clone git@github.com:CesiumGS/cesium-native.git --recurse-submodules

# Configure cesium-native and disable tests
cmake -B build -S . -G "Visual Studio 16 2019" -A x64 -DCMAKE_INSTALL_PREFIX=install -DCESIUM_TESTS_ENABLED=OFF
# Build and install cesium-native.  We generally use RelWithDebInfo on Windows but you can also build Release and Debug if you'd like.
cmake --build build --config RelWithDebInfo
cmake --install build --config RelWithDebInfo
```

The cesium-native libraries and headers are now located at cesium-native/install

You can follow the instructions for building osgEarth [here](install.html) but when you configure cmake pass in -DCESIUM_NATIVE_DIR=/path/to/cesium-native/install and -DOSGEARTH_BUILD_CESIUM_NODEKIT=ON so that osgEarth knows how to find the cesium-native libraries and headers.

This will build the osgEarthCesium nodekit of osgEarth.

# Loading data using cesium-native
To load data from Cesium Ion, you need to tell osgEarth what your access key is so it can authenticate you.  You do this by setting the OSGEARTH_CESIUMION_KEY environment variable.

On Windows
```
set OSGEARTH_CESIUMION_KEY=YOUR_KEY
```

On Linux
```
export OSGEARTH_CESIUMION_KEY=YOUR_KEY
```

cesium-native brings best in class 3d tiles streaming to osgEarth.  This is exposed via the CesiumNative3DTiles layer in an earth file.

We can load a 3D tiles dataset that is not hosted on Cesium Ion by setting the url to the root tileset.
```xml
<CesiumNative3DTiles name="agi">
    <url>https://pelican-public.s3.amazonaws.com/3dtiles/agi-hq/tileset.json</url>
</CesiumNative3DTiles>
```

To load an asset directly from Cesium Ion you will set the asset id like this
```xml
<CesiumNative3DTiles name="New York">
    <asset_id>57587</asset_id>
</CesiumNative3DTiles>
```

Some assets, like the Cesium World Terrain, don't have any textures associated and it is useful to drape an 
imagery dataset over it.  To do that, use the raster_overlay setting.  For example, this loads the Cesium World Terrain with the Bing imagery draped over it.
```xml
<CesiumNative3DTiles name="Cesium World Terrain and BING">
    <asset_id>1</asset_id>
    <raster_overlay>2</raster_overlay>
</CesiumNative3DTiles>
```

Google has also started serving out it's entire Google Earth dataset as 3D tiles.  You can load all of Google Earth in osgEarth by simply adding this to your earth file.  You can sign up and get a Google Maps key [here](https://developers.google.com/maps/documentation/embed/get-api-key)
```xml
<CesiumNative3DTiles name="Google Tiles">
    <url>https://tile.googleapis.com/v1/3dtiles/root.json?key=YOUR_GOOGLE_MAPS_KEY</url>
</CesiumNative3DTiles>
```


# Displaying Cesium Credits
cesium-native has a credit system that tells you what attribution is required to be displayed on screen based on what is currently being displayed on screen.  To enable this in your application you need to add a 
CesiumCreditsNode to your scene.  A basic usage of the CesiumCredits node can be seen below

```c++
#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarthCesium/CesiumCreditsNode>

using namespace osgEarth;

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    osg::Group* root = new osg::Group;

    auto node = MapNodeHelper().load(arguments, &viewer);
    if (node.valid())
    {
        root->addChild(node);
        viewer.setSceneData(root);

        auto creditsNode = new osgEarth::Cesium::CesiumCreditsNode(&viewer);
        root->addChild(creditsNode);
        return viewer.run();
    }

    return 0;
}
```
