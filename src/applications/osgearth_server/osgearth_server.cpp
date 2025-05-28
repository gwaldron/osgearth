/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgDB/ReadFile>
#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/GDAL>
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

    unsigned int port = 1234;
    arguments.read("--port", port);

    std::string host = "0.0.0.0";
    arguments.read("--host", host);

    unsigned int threads = std::max(std::thread::hardware_concurrency() - 1, 8u);
    arguments.read("--threads", threads);

    // Load the earth file:
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles(arguments);
    if (!node.valid())
        return usage(argv[0], "File not found");

    // If the node doesn't contain a MapNode (earth file), bail out:
    auto mapNode = osgEarth::MapNode::get(node);
    if (!mapNode)
        return usage(argv[0], "No MapNode in file");

    Server svr;
    svr.new_task_queue = [&threads] { return new ThreadPool(threads); };

    svr.Get("/layer/:layer/:z/:x/:y", [&mapNode](const Request& req, Response& res) {
        auto layerName = req.path_params.at("layer");
        auto z = req.path_params.at("z");
        auto x = req.path_params.at("x");
        auto y = req.path_params.at("y");  

        std::cout 
            << "/layer/"<< layerName
            << "/" << z
            << "/" << x
            << "/" << y << std::endl;

        osgEarth::Layer* layer = mapNode->getMap()->getLayerByName<osgEarth::Layer>(layerName);
        if (layer != nullptr)
        {
            ImageLayer* imageLayer = dynamic_cast<ImageLayer*>(layer);
            ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
            if (imageLayer)
            {
                auto image = imageLayer->createImage(osgEarth::TileKey(std::stoi(z), std::stoi(x), std::stoi(y), imageLayer->getProfile()));
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

        std::cout
            << "/elevation/"
            << "/" << z
            << "/" << x
            << "/" << y << std::endl;

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

    svr.listen(host, port);

    return 0;
}