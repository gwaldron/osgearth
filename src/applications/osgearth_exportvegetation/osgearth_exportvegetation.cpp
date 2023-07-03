/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include <osgEarth/Notify>
#include <osgEarth/MapNode>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/Feature>
#include <osgEarth/TerrainTileModelFactory>
#include <osgEarth/LandCover>
#include <osgEarthProcedural/VegetationLayer>
#include <osgEarthProcedural/NoiseTextureFactory>
#include <osgEarthProcedural/VegetationFeatureGenerator>
#include <osgDB/ReadFile>

#define LC "[exportvegetation] "

using namespace osgEarth;
using namespace osgEarth::Procedural;
using namespace osgEarth::Util;

int
usage(const char* name, const std::string& error)
{
    OE_NOTICE 
        << "Error: " << error
        << "\nUsage:"
        << "\n" << name << " file.earth"
        << "\n  --layer layername                    ; name of Vegetation layer (optional)"
        << "\n  --extents swlong swlat nelong nelat  ; extents in degrees"
        << "\n  --out out.shp                        ; output features"
        << "\n  --include-asset-property <name>      ; include asset property name as attribute (optional)"
        << std::endl;

    return -1;
}

struct App
{
    osg::ref_ptr<const MapNode> mapNode;
    VegetationLayer* veglayer;
    GeoExtent extent;
    VegetationFeatureGenerator featureGen;
    osg::ref_ptr<OGRFeatureSource> outfs;

    Threading::Mutexed<std::queue<FeatureList*> > outputQueue;
    Threading::Event outputReady;
    bool debug;

    App() { }

    int open(int argc, char** argv)
    {
        osg::ArgumentParser arguments(&argc, argv);

        debug = arguments.read("--pause");
        if (debug)
        {
            std::cout << "Press enter to run..." << std::endl;
            getchar();
        }

        std::string layername;
        arguments.read("--layer", layername);

        double xmin, ymin, xmax, ymax;
        if (!arguments.read("--extents", xmin, ymin, xmax, ymax))
            return usage(argv[0], "Missing --extents");
        extent = GeoExtent(SpatialReference::get("wgs84"), xmin, ymin, xmax, ymax);

        std::string outfile;
        if (!arguments.read("--out", outfile))
            return usage(argv[0], "Missing --out");

        osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles(arguments);
        mapNode = MapNode::get(node.get());
        if (!mapNode.valid())
            return usage(argv[0], "No earth file");

        const Map* map = mapNode->getMap();

        if (!layername.empty())
            veglayer = map->getLayerByName<VegetationLayer>(layername);
        else
            veglayer = map->getLayer<VegetationLayer>();

        if (!veglayer)
            return usage(argv[0], "Cannot find Vegetation layer in map");

        // prevent the biome manager from unloading data while we're exporting.
        if (veglayer->getBiomeLayer())
            veglayer->getBiomeLayer()->getBiomeManager().setLocked(true);

        featureGen.setMap(map);
        featureGen.setLayer(veglayer);

        if (featureGen.getStatus().isError())
            return usage(argv[0], featureGen.getStatus().message());

        // create output shapefile
        osg::ref_ptr<FeatureProfile> outProfile = new FeatureProfile(extent);
        FeatureSchema outSchema;
        outSchema["elevation"] = ATTRTYPE_DOUBLE;
        outSchema["width"] = ATTRTYPE_DOUBLE;
        outSchema["height"] = ATTRTYPE_DOUBLE;
        outSchema["rotation"] = ATTRTYPE_DOUBLE;

        std::string prop;
        while(arguments.read("--include-asset-property", prop))
        {
            featureGen.addAssetPropertyName(prop);
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
        std::cout << " Key = " << key.str() << std::endl;

        // even if the output if empty, we still must push a FeatureList
        // to the output queue b/c it's expecting an exact number of keys.
        FeatureList* output = new FeatureList();
        featureGen.getFeatures(key, *output);

        outputQueue.lock();
        outputQueue.push(output);
        outputReady.set();
        outputQueue.unlock();
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
    unsigned lod = app.veglayer->options().group("trees").lod().get();
    app.mapNode->getMap()->getProfile()->getIntersectingTiles(app.extent, lod, keys);
    if (keys.empty())
        return usage(argv[0], "No data in extent");

    JobArena arena("Vegetation Export", 1u);

    std::cout << "Exporting " << keys.size() << " keys.." << std::endl;

    for(const auto key : keys)
    {
        Job(&arena).dispatch(
            [&app, key](Cancelable*)
            {
                app.exportKey(key);
            }
        );
    }
    

    unsigned totalFeatures = 0u;
    std::vector<TimeSpan> writeTimes;

    for(unsigned i=0; i<keys.size(); )
    {
        app.outputReady.waitAndReset();

        std::vector<FeatureList*> outputs;

        app.outputQueue.lock();
        while(!app.outputQueue.empty())
        {
            outputs.push_back(app.outputQueue.front());
            app.outputQueue.pop();
        }
        app.outputQueue.unlock();

        for(unsigned j=0; j<outputs.size(); ++j)
        {
            osg::Timer_t startWrite = osg::Timer::instance()->tick();

            FeatureList* output = outputs[j];
            for(FeatureList::iterator k = output->begin(); k != output->end(); ++k)
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

    TimeSpan avgWriteTime=0, totalWriteTime=0;
    for(int i=0; i<writeTimes.size(); ++i) totalWriteTime += writeTimes[i];
    avgWriteTime /= writeTimes.size();

    std::cout 
        << "\rDone"
        << "; keys=" << keys.size()
        << "; features=" << totalFeatures
        << "; time=" << totalTime << "s"
        << "; write=" << totalWriteTime << "s ("<<(int)(100*totalWriteTime/totalTime)<<"%)"
        << std::endl;
}
