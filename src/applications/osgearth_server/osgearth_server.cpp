/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgDB/ReadFile>
#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/GDAL>
#include <osgViewer/Viewer>
#include <osgEarth/GLUtils>
#include <osgEarth/TileRasterizer>
#include <osgEarth/NodeUtils>
#include "httplib.h"

using namespace osgEarth;
using namespace httplib;

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

    unsigned int port = 1234;
    if (arguments.read("--port", port))
    {
        std::cout << "Listening on " << port << std::endl;
    }

    // configure the fastest possible frame loop.
    GL3RealizeOperation* realizer = new GL3RealizeOperation();
    realizer->setSyncToVBlank(false);

    osgViewer::Viewer viewer;
    viewer.setRealizeOperation(realizer);
    viewer.setThreadingModel(viewer.SingleThreaded);

    // configure for headless rendering.
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits();
    traits->x = 0;
    traits->y = 0;
    traits->width = 1;
    traits->height = 1;
    traits->windowDecoration = false;
    traits->pbuffer = true;
    traits->doubleBuffer = true;
    traits->sharedContext = nullptr;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    auto* camera = viewer.getCamera();
    camera->setGraphicsContext(gc.get());
    camera->setDrawBuffer(GL_BACK);
    camera->setReadBuffer(GL_BACK);
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));

    // disable the kdtrees for performance boost
    osgDB::Registry::instance()->setKdTreeBuilder(nullptr);

    std::mutex rootMutex;
    osg::ref_ptr< osg::Group > root = new osg::Group;
    viewer.setSceneData(root);
    viewer.realize();

    std::atomic<int> pendingRasterizedImages;

    Server svr;
    //svr.new_task_queue = [] { return new ThreadPool(1); };

    svr.Get("/layer/:layer/:z/:x/:y", [&mapNode, &viewer, &root, &pendingRasterizedImages, &rootMutex](const Request& req, Response& res) {
        auto layerName = req.path_params.at("layer");
        auto z = req.path_params.at("z");
        auto x = req.path_params.at("x");
        auto y = req.path_params.at("y");

        osgEarth::Layer* layer = mapNode->getMap()->getLayerByName<osgEarth::Layer>(layerName);
        if (layer != nullptr)
        {
            ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);
            ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
            if (imageLayer)
            {
                GeoImage image;

                auto* rasterizer = osgEarth::findTopMostNodeOfType<TileRasterizer>(imageLayer->getNode());
                if (rasterizer)
                {
                    // make the rasterizer faster? Maybe?
                    rasterizer->setNumRenderersPerGraphicsContext(64);
                    rasterizer->setNumJobsToDispatchPerFrame(64);

                    std::lock_guard<std::mutex> lock(rootMutex);
                    if (!root->containsNode(rasterizer))
                    {
                        root->addChild(rasterizer);
                    }                    
                }

                if  (rasterizer) ++pendingRasterizedImages;
                image = imageLayer->createImage(osgEarth::TileKey(std::stoi(z), std::stoi(x), std::stoi(y), imageLayer->getProfile()));                
                if (rasterizer) --pendingRasterizedImages;

                if (image.valid())
                {
                    auto rw = osgDB::Registry::instance()->getReaderWriterForExtension("png");
                    if (rw)
                    {
                        std::stringstream buf;
                        rw->writeImage(*image.getImage(), buf);
                        res.set_content(buf.str(), "image/png");
                    }
                }
                else
                {
                    res.status = 404;
                }
            }
            else if (elevationLayer)
            {                
                auto heightField = elevationLayer->createHeightField(osgEarth::TileKey(std::stoi(z), std::stoi(x), std::stoi(y), elevationLayer->getProfile()));
                if (heightField.valid())
                {
                    std::string content = osgEarth::GDAL::heightFieldToTiff(heightField.getHeightField());
                    res.set_content(content, "image/tiff");
                }
                else
                {
                    res.status = 404;
                }
            }
            
        }
        else
        {
            res.set_content(layerName + " Not Found", "text/plain");
            res.status = 404;
        }        
    });

    svr.Get("/elevation/:z/:x/:y", [&mapNode](const Request& req, Response& res) {
        auto z = req.path_params.at("z");
        auto x = req.path_params.at("x");
        auto y = req.path_params.at("y");

        osg::ref_ptr<ElevationTexture> elevTex;
        if (mapNode->getMap()->getElevationPool()->getTile(osgEarth::TileKey(std::stoi(z), std::stoi(x), std::stoi(y), mapNode->getMap()->getProfile()), false, elevTex, nullptr, nullptr))
        {
            std::string content = osgEarth::GDAL::heightFieldToTiff(elevTex->getHeightField());
            res.set_content(content, "image/tiff");
        }
        else
        {
            res.status = 404;
        }
        });

    // Run the server in a thread so we can run the frame loop in main
    std::thread serverThread([&svr, &port]() {
        svr.listen("0.0.0.0", port);
    });


    while (true)
    {
        // If there are any pending futures run the frame loop to render them.
        if (pendingRasterizedImages > 0)
        {            
            viewer.frame();
        }
        else
        {         
            std::this_thread::yield();
        }
    }

    return 0;
}