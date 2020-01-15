/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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
#include <osgEarth/ModelLayer>
#include <osgEarth/GeoTransform>

#include <osgEarth/TMS>
#include <osgEarth/WMS>
#include <osgEarth/GDAL>
#include <osgEarth/XYZ>
#include <osgEarth/Composite>

#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/AutoScaleCallback>

#include <osgEarth/FeatureMaskLayer>
#include <osgEarth/OGRFeatureSource>

#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Util;

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
    std::string _path;
    osg::ref_ptr<osg::Texture2D> _tex;

    META_Layer(osgEarth, MyTextureLayer, Options, ImageLayer, mytexturelayer);

    void setPath(const char* path)
    {
        _path = path;
    }

    Status openImplementation()
    {
        osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(_path);
        if (image.valid())
            _tex = new osg::Texture2D(image.get());
        else
            return Status(Status::ConfigurationError, "no path");

        // Establish a geospatial profile for the layer:
        setProfile(Profile::create("global-geodetic"));

        // Tell the layer to call createTexture:
        setUseCreateTexture();

        // Restrict the data extents of this layer to LOD 0 (in this case)
        dataExtents().push_back(DataExtent(getProfile()->getExtent(), 0, 0));

        return Status::OK();
    }

    TextureWindow
        createTexture(const TileKey& key, ProgressCallback* progress) const
    {
        // Set the texture matrix corresponding to the tile key:
        osg::Matrixf textureMatrix;
        key.getExtent().createScaleBias(getProfile()->getExtent(), textureMatrix);
        return TextureWindow(_tex.get(), textureMatrix);
    }
};

void
checkErrors(const Layer* layer)
{
    if (layer->getStatus().isError())
    {
        OE_WARN << "Layer " << layer->getName() << " : " << layer->getStatus().message() << std::endl;
    }
}

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
    TMSImageLayer* imagery = new TMSImageLayer();
    imagery->setURL("http://readymap.org/readymap/tiles/1.0.0/7/");
    map->addLayer( imagery );

    // add a TMS elevation layer:
    TMSElevationLayer* elevation = new TMSElevationLayer();
    elevation->setURL("http://readymap.org/readymap/tiles/1.0.0/116/");
    map->addLayer( elevation );

    // add a semi-transparent XYZ layer:
    XYZImageLayer* osm = new XYZImageLayer();
    osm->setURL("http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png");
    osm->setProfile(Profile::create("spherical-mercator"));
    osm->setOpacity(0.5f);
    map->addLayer(osm);

    // a custom layer that displays a user texture:
    MyTextureLayer* texLayer = new MyTextureLayer();
    texLayer->setPath("../data/grid2.png");
    texLayer->setOpacity(0.5f);
    map->addLayer(texLayer);  

    // add a WMS radar layer with transparency, and disable caching since
    // this layer updates on the server periodically.
    WMSImageLayer* wms = new WMSImageLayer();
    wms->setURL("http://mesonet.agron.iastate.edu/cgi-bin/wms/nexrad/n0r.cgi");
    wms->setFormat("png");
    wms->setLayers("nexrad-n0r");
    wms->setSRS("EPSG:4326");
    wms->setTransparent(true);
    wms->options().cachePolicy() = CachePolicy::NO_CACHE;
    map->addLayer(wms);

    // Add a local simple image as a layer. You can use the GDAL driver for this,
    // but since a GIF has no spatial reference information, you have to assign a
    // geospatial Profile so that osgEarth knows where to render it on the map.
    GDALImageLayer* gif = new GDALImageLayer();
    gif->setURL( "../data/osgearth.gif" );
    gif->setProfile( Profile::create("WGS84", -90.0, 10.0, -80.0, 15.0) );
    map->addLayer(gif);

    // Add a composite image layer that combines two other sources:
    GDALImageLayer* comp1 = new GDALImageLayer();
    comp1->setURL("../data/boston-inset-wgs84.tif");
    GDALImageLayer* comp2 = new GDALImageLayer();
    comp2->setURL("../data/nyc-inset-wgs84.tif");
    CompositeImageLayer* compImage = new CompositeImageLayer();
    compImage->addLayer(comp1);
    compImage->addLayer(comp2);
    map->addLayer(compImage);

    // Add a composite elevation layer tha tcombines two other sources:
    GDALElevationLayer* elev1 = new GDALElevationLayer();
    elev1->setURL("../data/terrain/mt_fuji_90m.tif");
    GDALElevationLayer* elev2 = new GDALElevationLayer();
    elev2->setURL("../data/terrain/mt_everest_90m.tif");
    CompositeElevationLayer* compElev = new CompositeElevationLayer();
    compElev->addLayer(elev1);
    compElev->addLayer(elev2);
    map->addLayer(compElev);

    // mask layer
    Polygon* maskGeom = new Polygon();
    maskGeom->push_back(osg::Vec3d(-111.0466, 42.0015, 0));
    maskGeom->push_back(osg::Vec3d(-111.0467, 40.9979, 0));
    maskGeom->push_back(osg::Vec3d(-109.0501, 41.0007, 0));
    maskGeom->push_back(osg::Vec3d(-109.0452, 36.9991, 0));
    maskGeom->push_back(osg::Vec3d(-114.0506, 37.0004, 0));
    maskGeom->push_back(osg::Vec3d(-114.0417, 41.9937, 0));

    OGRFeatureSource* maskFeatures = new OGRFeatureSource();
    maskFeatures->setGeometry(maskGeom);
    maskFeatures->setProfile(Profile::create("global-geodetic"));

    if (maskFeatures->open().isOK())
    {
        FeatureMaskLayer* maskLayer = new FeatureMaskLayer();
        maskLayer->setFeatureSource(maskFeatures);
        map->addLayer(maskLayer);
    }

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

        ModelLayer* layer = new ModelLayer();
        layer->setNode(xform);
        map->addLayer(layer);
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