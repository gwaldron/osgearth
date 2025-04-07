/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgEarth/GLUtils>
#include <osgEarth/LogarithmicDepthBuffer>
#include <iostream>


int
usage(const char* name, const char* message)
{
    std::cerr << "Error: " << message << std::endl;
    std::cerr << "Usage: " << name << " file.earth" << std::endl;
    return -1;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if (argc < 2)
        return usage(argv[0], "Missing earth file");

    // One time osgEarth initialization:
    osgEarth::initialize(arguments);

    // Load the earth file:
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles(arguments);
    if (!node.valid())
        return usage(argv[0], "File not found");

    // If the node doesn't contain a MapNode (earth file), bail out:
    auto mapNode = osgEarth::MapNode::get(node);
    if (!mapNode)
        return usage(argv[0], "No MapNode in file");

    // Set up a simple viewer with our custom manipulator:
    osgViewer::Viewer viewer(arguments);
    viewer.setSceneData(node);
    viewer.setCameraManipulator(new osgEarth::Util::EarthManipulator(arguments));

    // This is optional, but it will mitigate near-plane clipping in a whole earth scene:
    osgEarth::Util::LogarithmicDepthBuffer ldb;
    ldb.install(viewer.getCamera());

#ifndef OSG_GL3_AVAILABLE
    // If your OSG is build with a GL2 profile, install our custom realize op
    // to get all the shaders working:
    viewer.setRealizeOperation(new osgEarth::GL3RealizeOperation());
#endif

    return viewer.run();
}