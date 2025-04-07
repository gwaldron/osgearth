/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/Notify>
#include <osgEarth/MapNode>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/Feature>
#include <osgEarth/TerrainTileModelFactory>
#include <osgEarth/LandCover>
#include <osgEarthSplat/GroundCoverLayer>
#include <osgEarthSplat/NoiseTextureFactory>
#include <osgEarthSplat/GroundCoverFeatureGenerator>

#define LC "[exportgroundcover] "

using namespace osgEarth;
using namespace osgEarth::Splat;
using namespace osgEarth::Util;

int
usage(const char* name, const std::string& error)
{
    OE_NOTICE
        << "Error: " << error
        << "\nUsage:"
        << "\n" << name << " file.earth"
        << "\n  --layer layername                    ; name of GroundCover layer"
        << "\n  --extents swlong swlat nelong nelat  ; extents in degrees"
        << "\n  --out out.shp                        ; output features"
        << "\n  --include-billboard-property <name>  ; include billboard property name as attribute (optional)"
        << std::endl;

    return -1;
}

struct App
{
    osg::ref_ptr<const MapNode> mapNode;
    GroundCoverLayer* gclayer;
    GeoExtent extent;
    GroundCoverFeatureGenerator featureGen;
    osg::ref_ptr<OGRFeatureSource> outfs;

    Threading::Mutexed<std::queue<FeatureList*> > outputQueue;
    Threading::Event gate;

    App() { }

    int open(int argc, char** argv)
    {
        osg::ArgumentParser arguments(&argc, argv);

        std::string layername;
        if (!arguments.read("--layer", layername))
            return usage(argv[0], "Missing --layer");

        double xmin, ymin, xmax, ymax;
        if (!arguments.read("--extents", xmin, ymin, xmax, ymax))
            return usage(argv[0], "Missing --extents");
        extent = GeoExtent(SpatialReference::get("wgs84"), xmin, ymin, xmax, ymax);

        std::string outfile;
        if (!arguments.read("--out", outfile))
            return usage(argv[0], "Missing --out");

        mapNode = MapNode::load(arguments);
        if (!mapNode.valid())
            return usage(argv[0], "No earth file");

        const Map* map = mapNode->getMap();

        gclayer = map->getLayerByName<GroundCoverLayer>(layername);
        if (!gclayer)
            return usage(argv[0], "Cannot find --layer in map; check the layer name");

        featureGen.setMap(map);
        featureGen.setLayer(gclayer);
        featureGen.setFactory(new TerrainTileModelFactory(mapNode->options().terrain().get()));

        if (featureGen.getStatus().isError())
            return usage(argv[0], featureGen.getStatus().message());

        // create output shapefile
        osg::ref_ptr<FeatureProfile> outProfile = new FeatureProfile(extent);
        FeatureSchema outSchema;
        outSchema["elevation"] = ATTRTYPE_DOUBLE;
        outSchema["width"] = ATTRTYPE_DOUBLE;
        outSchema["height"] = ATTRTYPE_DOUBLE;

        std::string prop;
        while (arguments.read("--include-billboard-property", prop))
        {
            featureGen.addBillboardPropertyName(prop);
            outSchema[prop] = ATTRTYPE_STRING;
        }

        outfs = new OGRFeatureSource();
        outfs->setOGRDriver("ESRI Shapefile");
        outfs->setURL(outfile);
        if (outfs->create(outProfile.get(), outSchema, Geometry::TYPE_POINT, NULL).isError())
            return usage(argv[0], outfs->getStatus().toString());

        return 0;
    }

    void exportKey(const TileKey& key)
    {
        // even if the output if empty, we still must push a FeatureList
        // to the output queue b/c it's expecting an exact number of keys.
        FeatureList* output = new FeatureList();
        featureGen.getFeatures(key, *output);

        outputQueue.lock();
        outputQueue.push(output);
        gate.set();
        outputQueue.unlock();
    }
};

struct ExportOperation : public osg::Operation
{
    App& _app;
    TileKey _key;

    ExportOperation(App& app, const TileKey& key) :
        osg::Operation("build", false), _app(app), _key(key) { }

    void operator()(osg::Object*) override
    {
        _app.exportKey(_key);
    }
};

int
main(int argc, char** argv)
{
    osg::Timer_t start = osg::Timer::instance()->tick();

    App app;

    if (app.open(argc, argv) < 0)
        return -1;

    // find all intersecting tile keys
    std::vector<TileKey> keys;
    app.mapNode->getMap()->getProfile()->getIntersectingTiles(app.extent, app.gclayer->getLOD(), keys);
    if (keys.empty())
        return usage(argv[0], "No data in extent");

    std::cout << "Exporting " << keys.size() << " keys.." << std::endl;

    jobs::context ctx;
    ctx.pool = jobs::get_pool("GroundCover Export");
    ctx.pool->set_concurrency(4u);

    for (const auto& key : keys)
    {
        jobs::dispatch([&app, key]() { app.exportKey(key); }, ctx);
    }

    unsigned totalFeatures = 0u;
    std::vector<TimeSpan> writeTimes;

    for (unsigned i = 0; i < keys.size(); )
    {
        app.gate.waitAndReset();

        std::vector<FeatureList*> outputs;

        app.outputQueue.lock();
        while (!app.outputQueue.empty())
        {
            outputs.push_back(app.outputQueue.front());
            app.outputQueue.pop();
        }
        app.outputQueue.unlock();

        for (unsigned j = 0; j < outputs.size(); ++j)
        {
            osg::Timer_t startWrite = osg::Timer::instance()->tick();

            FeatureList* output = outputs[j];
            for (FeatureList::iterator k = output->begin(); k != output->end(); ++k)
            {
                app.outfs->insertFeature(k->get());
                ++totalFeatures;
            }

            delete output;
            std::cout << "\r" << (++i) << "/" << keys.size() << std::flush;

            osg::Timer_t endWrite = osg::Timer::instance()->tick();
            writeTimes.push_back(osg::Timer::instance()->delta_s(startWrite, endWrite));
        }
    }

    std::cout << "\nBuilding index.." << std::flush;
    app.outfs->buildSpatialIndex();

    app.outfs->close();

    osg::Timer_t end = osg::Timer::instance()->tick();
    double totalTime = osg::Timer::instance()->delta_s(start, end);

    TimeSpan avgWriteTime = 0, totalWriteTime = 0;
    for (int i = 0; i < writeTimes.size(); ++i) totalWriteTime += writeTimes[i];
    avgWriteTime /= writeTimes.size();

    std::cout
        << "\rDone"
        << "; keys=" << keys.size()
        << "; features=" << totalFeatures
        << "; time=" << totalTime << "s"
        << "; write=" << totalWriteTime << "s (" << (int)(100 * totalWriteTime / totalTime) << "%)"
        << std::endl;
}