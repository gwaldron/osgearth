/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* MIT License
*/
#define LC "[osgearth_lod] "

#include <osgEarth/Notify>
#include <osgEarth/LODGenerator>

#include <osg/ArgumentParser>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

using namespace osgEarth;

int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc, argv);

    // Read the ouptut filename
    std::string outFilename = "out.osgb";
    args.read("--out", outFilename);

    // Read the levels from the command line, formatted like --level threshold,range,aggressive
    std::vector< LODGenerator::LODOptions> options;
    float threshold = 0.0f;
    float range = 0.0f;
    bool aggressive = false;
    while (args.read("--level", threshold, range, aggressive))
    {
        OE_NOTICE << "Adding level " << threshold << ", " << range << ", " << aggressive << std::endl;
        options.push_back({ threshold, range, aggressive });
    }

    osg::ref_ptr<osg::Node> root = osgDB::readRefNodeFiles(args);

    LODGenerator generator;
    osg::ref_ptr< osg::Node> result = generator.generateLODs(root.get(), options);

    osgDB::writeNodeFile(*result, outFilename);
    return 0;
}
