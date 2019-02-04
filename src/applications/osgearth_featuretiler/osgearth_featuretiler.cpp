/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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
#include <osgEarth/TDTiles>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/FeatureCursor>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osg/ArgumentParser>
#include <osgDB/FileUtils>
#include <fstream>

#define LC "[featuretiler] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

int
usage(const char* msg)
{
    OE_NOTICE 
        << "\n" << msg
        << "\nUsage: osgearth_featuretiler"
        << "\n    --in driver [driver]   ; default=ogr"
        << "\n    --in url [location]    ; filename or URL"
        << "\n    --output [directory]   ; output folder name"
        << "\n    --depth [n]            ; maximum split depth"
        << "\n    --geometricerror [n]   ; geometric error in meters (default = 10m)"
        << std::endl;
    return -1;
}

struct Env
{
    double geometricError;
    unsigned maxDepth;
    URIContext uriContext;
    unsigned counter;
};

int
split(const Config& inputConf, TDTiles::Tile* parentTile, unsigned depth, Env& env)
{
    osg::ref_ptr<FeatureSource> input = FeatureSourceFactory::create(ConfigOptions(inputConf));
    if (!input.valid())
        return usage("Failed to open an input feature source");

    const Status& inputStatus = input->open();
    if (inputStatus.isError())
        return usage(inputStatus.message().c_str());

    const Profile* profile = 0L;
    const FeatureProfile* inputFP = input->getFeatureProfile();
    if (inputFP && inputFP->getExtent().isValid())
    {
        const GeoExtent& fex = inputFP->getExtent();
        profile = Profile::create(fex.getSRS(), fex.xMin(), fex.yMin(), fex.xMax(), fex.yMax());
    }

    OGRFeatureOptions outputConf[2];
    osg::ref_ptr<FeatureSource> output[2];

    for (unsigned i = 0; i < 2; ++i)
    {
        outputConf[i].url() = URI(Stringify() << "test_" << depth << "_" << (env.counter++) << "_" << i << ".shp", env.uriContext);
        outputConf[i].openWrite() = true;
        if (profile)
            outputConf[i].profile() = profile->toProfileOptions();

        output[i] = FeatureSourceFactory::create(outputConf[i]);
        if (!output[i].valid())
            return usage("Failed to open output dataset");

        const Status& outputStatus = output[i]->create(
            input->getFeatureProfile(),
            input->getSchema(),
            input->getGeometryType(),
            NULL);

        if (outputStatus.isError())
            return usage(outputStatus.message().c_str());
    }

    struct FeatureData {
        double x, y;
        FeatureID fid;
    };

    struct {
        bool operator()(const FeatureData& lhs, const FeatureData& rhs) const {
            return lhs.x < rhs.x;
        }
    } sortByX;

    struct {
        bool operator()(const FeatureData& lhs, const FeatureData& rhs) const {
            return lhs.y < rhs.y;
        }
    } sortByY;

    std::vector<FeatureData> data;
    int count = input->getFeatureCount();
    if (count > 0)
    {
        OE_INFO << "Feature count = " << count << std::endl;
        data.reserve(count);
    }
    else
    {
        OE_INFO << "Feature count not available" << std::endl;
    }

    unsigned i = 0;
    osg::ref_ptr<FeatureCursor> cursor = input->createFeatureCursor(0L);
    if (cursor.valid())
    {
        while (cursor->hasMore())
        {
            Feature* f = cursor->nextFeature();
            const GeoExtent& fex = f->getExtent();
            FeatureData d;
            fex.getCentroid(d.x, d.y);
            d.fid = f->getFID();
            data.push_back(d);
        }
    }

    const GeoExtent& extent = input->getFeatureProfile()->getExtent();
    bool isWide = extent.width() > extent.height();

    OE_INFO << "Features Extent = " << extent.toString() << std::endl;

    double median;

    if (isWide)
    {
        std::sort(data.begin(), data.end(), sortByX);
        median = ((data.size() & 0x1) == 0) ?
            0.5 * (data[data.size() / 2].x + data[data.size() / 2].x) :
            data[data.size() / 2].x;
        OE_INFO << "Median X = " << median << std::endl;
    }
    else
    {
        std::sort(data.begin(), data.end(), sortByY);
        median = ((data.size() & 0x1) == 0) ?
            0.5 * (data[data.size() / 2].y + data[data.size() / 2].y) :
            data[data.size() / 2].y;
        OE_INFO << "Median Y = " << median << std::endl;
    }

    osg::ref_ptr<TDTiles::Tile> child[2];

    child[0] = new TDTiles::Tile();
    child[0]->refine() = TDTiles::REFINE_REPLACE;
    parentTile->children().push_back(child[0].get());

    child[1] = new TDTiles::Tile();
    child[0]->refine() = TDTiles::REFINE_REPLACE;
    parentTile->children().push_back(child[1].get());

    for (unsigned i = 0; i < data.size(); ++i)
    {
        const FeatureData& d = data[i];
        Feature* f = input->getFeature(d.fid);
        double x, y;
        f->getExtent().getCentroid(x, y);
        double side = isWide ? x : y;
        if (side < median)
            output[0]->insertFeature(f);
        else
            output[1]->insertFeature(f);
    }

    input = 0L;
    output[0] = 0L;
    output[1] = 0L;

    for(unsigned i=0; i<2; ++i)
    {
        child[i]->content()->uri() = outputConf[i].url().get();

        if (depth < env.maxDepth)
        {
            Config conf = inputConf;
            conf.set("url", outputConf[i].url()->full());
            split(conf, child[i].get(), depth+1, env);
        }
    }

    //TODO: heights
    const float zmin = 0.0f;
    const float zmax = 1.0f;

    GeoExtent a(extent.getSRS()), b(extent.getSRS());

    if (isWide)
    {
        a.set(extent.xMin(), extent.yMin(), median, extent.yMax());
        b.set(median, extent.yMin(), extent.xMax(), extent.yMax());
    }
    else
    {
        a.set(extent.xMin(), extent.yMin(), extent.xMax(), median);
        b.set(extent.xMin(), median, extent.xMax(), extent.yMax());
    }

    const SpatialReference* epsg4979 = SpatialReference::get("epsg:4979");
    a = a.transform(epsg4979);
    b = b.transform(epsg4979);

    child[0]->boundingVolume()->region()->set(
        osg::DegreesToRadians(a.xMin()), osg::DegreesToRadians(a.yMin()), zmin, 
        osg::DegreesToRadians(a.xMax()), osg::DegreesToRadians(a.yMax()), zmax);
    child[0]->geometricError() = env.geometricError / double(depth+1);

    child[1]->boundingVolume()->region()->set(
        osg::DegreesToRadians(b.xMin()), osg::DegreesToRadians(b.yMin()), zmin,
        osg::DegreesToRadians(b.xMax()), osg::DegreesToRadians(b.yMax()), zmax);
    child[1]->geometricError() = env.geometricError / double(depth+1);

    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if (arguments.read("--help"))
        return usage("Help!");

    Config inputConf;
    std::string key, value;
    while(arguments.read("--in", key, value))
        inputConf.set(key, value);

    std::string outputLocation;
    if (!arguments.read("--out", outputLocation))
        return usage("Missing required --out argument");

    unsigned maxDepth = 1u;
    arguments.read("--depth", maxDepth);

    double geometricError = 10.0;
    arguments.read("--geometricerror", geometricError);

    if (!osgDB::makeDirectory(outputLocation))
        return usage("Unable to create/find output location");

    osg::ref_ptr<TDTiles::Tile> root = new TDTiles::Tile();

    std::string outFile = Stringify() << outputLocation << "/tileset.json";
    URIContext uriContext(outFile);

    Env env;
    env.counter = 0;
    env.geometricError = geometricError;
    env.maxDepth = maxDepth;
    env.uriContext = uriContext;

    split(inputConf, root, 0, env);

    osg::ref_ptr<TDTiles::Tileset> tileset = new TDTiles::Tileset();
    tileset->root() = root.get();

    TDTiles::Asset asset;
    tileset->asset()->version() = "1.0";

    std::ofstream out(outFile);
    Json::Value tilesetJSON = tileset->getJSON();
    Json::StyledStreamWriter writer;
    writer.write(out, tilesetJSON);
    out.close();

    return 0;
}