/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* MIT License
*/
#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/TMS>

#include <osgEarth/ExampleResources>
#include <osgEarth/EarthManipulator>
#include <osgEarth/LogarithmicDepthBuffer>

#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>

#ifdef  OSGEARTH_LIBRARY_STATIC

USE_OSGPLUGIN(osg)
USE_OSGPLUGIN(curl)
USE_OSGPLUGIN(tiff)
USE_OSGPLUGIN(jpeg)
USE_OSGPLUGIN(shp)
USE_OSGPLUGIN(earth)
USE_OSGPLUGIN(osgearth_engine_rex)

USE_DOTOSGWRAPPER_LIBRARY(osg)

//--- to use  REGISTERed X11 WINDOWINGSYSTEMINTERFACE2
USE_GRAPHICSWINDOW()
#endif//#ifdef  OSGEARTH_LIBRARY_STATIC


using namespace osgEarth;
using namespace osgEarth::Util;

#define IMAGERY_URL      "http://readymap.org/readymap/tiles/1.0.0/22/"
#define ELEVATION_URL    "http://readymap.org/readymap/tiles/1.0.0/116/"
std::string g_dataRootDir("/home/abner/abner2/zdev/nv/osgearth0x/3rd/osgearth/data/");
#define BUILDINGS_URL    "../data/boston_buildings_utm19.shp"
#define RESOURCE_LIB_URL "../data/resources/textures_us/catalog.xml"
#define STREETS_URL      "../data/boston-scl-utm19n-meters.shp"
#define PARKS_URL        "../data/boston-parks.shp"
#define TREE_MODEL_URL   "../data/tree.osg"

 
 
/**
 * This code example effectively duplicates the "boston.earth" sample,
 * demonstrating how to create a 3D city model in osgEarth.
 *
 * Run this from the tests folder.
 */
 

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>
#include <osgEarth/ShaderGenerator>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgUtil/Optimizer>
#include <iostream>
#include <osgEarth/Metrics>
#include <osgEarth/GDAL>
using namespace osgEarth;
using namespace osgEarth::Util;
#include <osgUtil/CullVisitor>
#include <string>
#include <iostream>

using namespace std;
EarthManipulator* g_mp = nullptr;
class MyLayerCullCallBack : public osgEarth::Layer::TraversalCallback
{
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) const;
};

void MyLayerCullCallBack::operator()(osg::Node* node, osg::NodeVisitor* nv) const
{
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
    if (cv) {
        osg::Vec3 eye, focal, up;
        cv->getCurrentCamera()->getViewMatrixAsLookAt(eye, focal, up);//获取相机世界坐标
        double x = g_mp->getViewpoint().focalPoint()->x();//获取相机焦点经度
        double y = g_mp->getViewpoint().focalPoint()->y();//获取相机焦点纬度
        cout << endl;
    }

    traverse(node, nv);
}


int main()
{
    osgEarth::initialize();
    //create a viewer
    osgViewer::Viewer viewer ;
    viewer.setUpViewInWindow(0, 100, 800, 600);

    // ? why
    viewer.setReleaseContextAtEndOfFrameHint(false);

    //set camera manipulator
    EarthManipulator* mp = new EarthManipulator;
    viewer.setCameraManipulator(mp);

    // Map is datamodel for collection of layers.
    osg::ref_ptr<osgEarth::Map> rootMap = new osgEarth::Map ;

    //GeoTiff Layer
    string worldTifFilename = g_dataRootDir+"world.tif";
    osg::ref_ptr<GDALImageLayer> gdalLayer = new GDALImageLayer();    
    gdalLayer->setURL(osgEarth::URI(worldTifFilename));

    // MapNode is the render or visualization of Map.
    osg::ref_ptr<osgEarth::MapNode> rootMapNode = new osgEarth::MapNode(rootMap.get());
    rootMap->addLayer(gdalLayer);
    viewer.setSceneData(rootMapNode);

    //view point
    // Heading in degrees
    // pitch 俯仰角 in degrees
    // range in meters
    Viewpoint vp("Home",
        116.4042, 39.9072, 5.0,   // 海拔高度 单位米。
        0,   // Heading 相机指向焦点角度，单位角度。
        -90,   // pitch 相机相对焦点俯仰角度，单位角度。
        1E7 // 距离焦点距离，这里表示距离地表经纬度点的距离，单位米。
    );
    mp->setViewpoint(vp , 
        3  // 相机移动时间，单位秒。
       );

    g_mp = mp;
    gdalLayer->setCullCallback(new MyLayerCullCallBack);

    return viewer.run();
} 
 