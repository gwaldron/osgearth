/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/Notify>
#include <osgEarth/MapNode>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/Feature>
#include <osgEarth/TerrainTileModelFactory>

#define LC "[clamp] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name, const std::string& error)
{
    OE_NOTICE 
        << "Clamps shapefile features to terrain and writes out a new shapefile."
        << "\nError: " << error
        << "\nUsage:"
        << "\n" << name
        << "\n  <earthfile>          ; earth file containing elevation layer"
        << "\n  --in in.shp          ; input features to clamp"
        << "\n  --out out.shp        ; output features"
        << "\n  --attribute <name>   ; attribute in which to store elevation value"
        << "\n  [--quiet]            ; suppress console output"
        << std::endl;

    return -1;
}

struct App
{
    osg::ref_ptr<MapNode> mapNode;
    const Map* map;
    osg::ref_ptr<OGRFeatureSource> input;
    osg::ref_ptr<OGRFeatureSource> output;
    Threading::Mutexed<std::queue<FeatureList*> > outputQueue;
    std::string attrName;
    bool verbose;

    App() : verbose(true) { }

    int open(int argc, char** argv)
    {
        osg::ArgumentParser arguments(&argc, argv);

        verbose = !arguments.read("--quiet");

        std::string infile;
        if (!arguments.read("--in", infile))
            return usage(argv[0], "Missing --in");

        std::string outfile;
        if (!arguments.read("--out", outfile))
            return usage(argv[0], "Missing --out");

        if (!arguments.read("--attribute", attrName))
            return usage(argv[0], "Missing --attribute");

        mapNode = MapNode::load(arguments);
        if (!mapNode.valid())
            return usage(argv[0], "No earth file");

        // open input shapefile
        input = new OGRFeatureSource();
        input->setURL(infile);
        if (input->open().isError())
            return usage(argv[0], input->getStatus().message());

        // create output shapefile
        FeatureSchema outSchema;
        outSchema = input->getSchema();
        outSchema[attrName] = ATTRTYPE_DOUBLE;
        output = new OGRFeatureSource();
        output->setOGRDriver("ESRI Shapefile");
        output->setURL(outfile);
        if (output->create(input->getFeatureProfile(), outSchema, input->getGeometryType(), NULL).isError())
            return usage(argv[0], output->getStatus().toString());

        return 0;
    }

    void run()
    {
        ElevationPool::WorkingSet workingSet;

        unsigned total = input->getFeatureCount();
        unsigned count = 0u;

        std::cout << "\n";

        GeoPoint point(input->getFeatureProfile()->getSRS(),0,0,0);

        osg::ref_ptr<FeatureCursor> cursor = input->createFeatureCursor();
        while(cursor->hasMore())
        {
            Feature* f = cursor->nextFeature();
            GeoExtent e = f->getExtent();

            ElevationSample sample = map->getElevationPool()->getSample(
                e.getCentroid(),
                &workingSet);
            
            float value = sample.elevation().as(Units::METERS);
            if (value == NO_DATA_VALUE)
                value = 0.0f;

            f->set(attrName, value);

            output->insertFeature(f);

            if (verbose)
            {
                ++count;
                if (count==1 || count%1000==0 || count==total)
                    std::cout << "\r" << count << "/" << total << std::flush;
            }
        }

        if (verbose)
            std::cout << std::endl;
    }
};

int
main(int argc, char** argv)
{
    App app;

    if (app.open(argc, argv) < 0)
        return -1;

    app.run();

    if (app.verbose)
        std::cout << "\nBuilding index..." << std::flush;

    app.output->buildSpatialIndex();
    app.output->close();

    if (app.verbose)
        std::cout << "\rDone!            " << std::endl;
}