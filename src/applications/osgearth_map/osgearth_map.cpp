/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgViewer/Viewer>

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/ModelLayer>
#include <osgEarth/GeoTransform>
#include <osgEarth/CompositeTileSource>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/AutoScaleCallback>

#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/wms/WMSOptions>
#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/osg/OSGOptions>
#include <osgEarthDrivers/xyz/XYZOptions>
#include <osgEarthDrivers/debug/DebugOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>

#include <osgEarthFeatures/FeatureMaskLayer>

#include <osg/PositionAttitudeTransform>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;
using namespace osgEarth::Features;

int
usage(int argc, char** argv)
{
    OE_NOTICE 
        << "\n" << argv[0]
        << "\n    [--out outFile] : write map node to outFile before exit"
        << std::endl;

    return 0;
}

// Demonstrates how to subclass ImageLayer to directly create textures
// for use in a layer.
class MyTextureLayer : public ImageLayer
{
public:
    osg::ref_ptr<osg::Texture2D> _tex;

    MyTextureLayer(const char* path)
    {
        osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(path);
        if (image.valid())
            _tex = new osg::Texture2D(image.get());

        // Establish a profile for the layer:
        setProfile(Profile::create("global-geodetic"));

        // Direct the layer to call createTexture:
        setUseCreateTexture();

        // Restrict the data extents of this layer to LOD 0 (in this case)
        dataExtents().push_back(DataExtent(getProfile()->getExtent(), 0, 0));
    }

    osg::Texture* createTexture(const TileKey& key, ProgressCallback* progress, osg::Matrixf& textureMatrix)
    {
        // Set the texture matrix corresponding to the tile key:
        key.getExtent().createScaleBias(getProfile()->getExtent(), textureMatrix);

        return _tex.get();
    }
};

/**
 * How to create a simple osgEarth map and display it.
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if (arguments.read("--help"))
        return usage(argc, argv);

    // create the empty map.
    Map* map = new Map();

    // add a TMS imagery layer:
    TMSOptions imagery;
    imagery.url() = "http://readymap.org/readymap/tiles/1.0.0/7/";
    map->addLayer( new ImageLayer("ReadyMap Imagery", imagery) );

    // add a TMS elevation layer:
    TMSOptions elevation;
    elevation.url() = "http://readymap.org/readymap/tiles/1.0.0/116/";
    map->addLayer( new ElevationLayer("ReadyMap Elevation", elevation) );

    // add a semi-transparent XYZ layer:
    XYZOptions xyz;
    xyz.url() = "http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png";
    xyz.profile()->namedProfile() = "spherical-mercator";
    ImageLayer* imageLayer = new ImageLayer("OSM", xyz);
    imageLayer->setOpacity(0.5f);
    map->addLayer(imageLayer);

    // a custom layer that displays a user texture:
    MyTextureLayer* texLayer = new MyTextureLayer("../data/grid2.png");
    texLayer->setOpacity(0.5f);
    map->addLayer(texLayer);  
    
    // add a local GeoTIFF inset layer:
    GDALOptions gdal;
    gdal.url() = "../data/boston-inset.tif";
    map->addLayer(new ImageLayer("Boston", gdal));

    // add a WMS radar layer with transparency, and disable caching since
    // this layer updates on the server periodically.
    WMSOptions wms;
    wms.url() = "http://mesonet.agron.iastate.edu/cgi-bin/wms/nexrad/n0r.cgi";
    wms.format() = "png";
    wms.layers() = "nexrad-n0r";
    wms.srs() = "EPSG:4326";
    wms.transparent() = true;
    ImageLayerOptions wmsLayerOptions("WMS NEXRAD", wms);
    wmsLayerOptions.cachePolicy() = CachePolicy::NO_CACHE;
    map->addLayer(new ImageLayer(wmsLayerOptions));

    // add a local simple image as a layer using the OSG driver:
    OSGOptions osg;
    osg.url() = "../data/osgearth.gif";
    osg.profile()->srsString() = "wgs84";
    osg.profile()->bounds()->set(-90.0, 10.0, -80.0, 15.0);
    map->addLayer(new ImageLayer("Simple image", osg));

    // create a composite image layer that combines two other sources:
    GDALOptions c1;
    c1.url() = "../data/boston-inset-wgs84.tif";

    GDALOptions c2;
    c2.url() = "../data/nyc-inset-wgs84.tif";

    CompositeTileSourceOptions composite;
    composite.add(ImageLayerOptions(c1));
    composite.add(ImageLayerOptions(c2));

    ImageLayerOptions compLayerOptions("My Composite Layer", composite);
    map->addLayer(new ImageLayer(compLayerOptions));

    // mask layer
    OGRFeatureOptions maskOptions;
    maskOptions.geometry() = new Polygon();
    maskOptions.geometry()->push_back(osg::Vec3d(-111.0466, 42.0015, 0));
    maskOptions.geometry()->push_back(osg::Vec3d(-111.0467, 40.9979, 0));
    maskOptions.geometry()->push_back(osg::Vec3d(-109.0501, 41.0007, 0));
    maskOptions.geometry()->push_back(osg::Vec3d(-109.0452, 36.9991, 0));
    maskOptions.geometry()->push_back(osg::Vec3d(-114.0506, 37.0004, 0));
    maskOptions.geometry()->push_back(osg::Vec3d(-114.0417, 41.9937, 0));
    maskOptions.profile() = ProfileOptions("global-geodetic");
    FeatureMaskLayerOptions maskLayerOptions;
    maskLayerOptions.name() = "Mask layer";
    maskLayerOptions.featureSource() = maskOptions;
    map->addLayer(new FeatureMaskLayer(maskLayerOptions));

    // put a model on the map atop Pike's Peak, Colorado, USA
    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile("cow.osgt.(0,0,3).trans.osgearth_shadergen");
    if (model.valid())
    {
        osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform();
        pat->addCullCallback(new AutoScaleCallback<osg::PositionAttitudeTransform>(5.0));
        pat->addChild(model.get());

        GeoTransform* xform = new GeoTransform();
        xform->setPosition(GeoPoint(SpatialReference::get("wgs84"), -105.042292, 38.840829));
        xform->addChild(pat);

        map->addLayer(new ModelLayer("Model", xform));
    }

    // make the map scene graph:
    MapNode* node = new MapNode( map );

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    viewer.setSceneData( node );

    // add some stock OSG handlers:
    MapNodeHelper().configureView(&viewer);

    int r = viewer.run();

    std::string outFile;
    if (arguments.read("--out", outFile))
        osgDB::writeNodeFile(*node, outFile);

    return r;
}