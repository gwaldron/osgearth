#include <osgEarth/MapNode>
#include <osgEarth/TMS>
#include <osgEarth/EarthManipulator>
#include <osgViewer/Viewer>

int main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc, argv);
    osgViewer::Viewer viewer(args);
    
    osgEarth::initialize();
    osgEarth::setNotifyLevel(osg::INFO);
    
    auto layer = new osgEarth::TMSImageLayer();
    layer->setURL("https://readymap.org/readymap/tiles/1.0.0/7/");
    
    auto mapNode = new osgEarth::MapNode();
    mapNode->getMap()->addLayer(layer);
    
    viewer.setCameraManipulator(new osgEarth::Util::EarthManipulator());
    
    viewer.setSceneData(mapNode);
    return viewer.run();
}