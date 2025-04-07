/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Metrics>
#include <osgEarth/Lighting>
#include <osgEarth/NodeUtils>
#include <osgEarth/PhongLightingEffect>

#include <osgEarthProcedural/BiomeLayer>
#include <osgEarthProcedural/BiomeManager>
#include <osgEarthProcedural/LifeMapLayer>
#include <osgEarthProcedural/VegetationLayer>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Uniform>
#include <iostream>

#define LC "[osgearth_biome] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Procedural;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name
        << "\n   --encode-texture [filename]"
        << std::endl;
    return 0;
}

int
encodeTexture(osg::ArgumentParser& args)
{
    std::string infile;
    if (args.read("--encode-texture", infile))
    {
        std::string libName = osgDB::Registry::instance()->createLibraryNameForNodeKit("osgEarthProcedural");
        osgDB::Registry::LoadStatus status = osgDB::Registry::instance()->loadLibrary(libName);

        osg::ref_ptr<osg::Image> image;

        image = osgDB::readRefImageFile(infile + ".oe_splat_rgbh");
        if (image.valid())
            osgDB::writeImageFile(*image.get(), infile + ".oe_splat_rgbh");

        image = osgDB::readRefImageFile(infile + ".oe_splat_nnra");
        if (image.valid())
            osgDB::writeImageFile(*image.get(), infile + ".oe_splat_nnra");
    }
    return 0;
}

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc, argv);

    if (arguments.find("--encode-texture") >= 0)
        return encodeTexture(arguments);
    
    return usage(argv[0]);
}
