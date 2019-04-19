/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osgEarthFeatures/ResampleFilter>
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
        << "\n    --in [location]        ; filename of source data (e.g. a shapefile)"
        << "\n    --out [directory]      ; output folder name"
        << "\n    --errors [...]         ; Comma-delimited geometric error per level (e.g. 1,150,500)"
        << std::endl;
    return -1;
}

struct Env
{
    osg::ref_ptr<FeatureSource> input;
    double* geometricError;
    unsigned maxDepth;
    URIContext uriContext;
    unsigned counter;

    ~Env() { delete [] geometricError; }
};

struct FeatureData {
    double x, y;
    FeatureID fid;
};

struct SortByX {
    bool operator()(const FeatureData& lhs, const FeatureData& rhs) const {
        return lhs.x < rhs.x;
    }
};

struct SortByY {
    bool operator()(const FeatureData& lhs, const FeatureData& rhs) const {
        return lhs.y < rhs.y;
    }
};

int
split(const GeoExtent& extent, TDTiles::Tile* parentTile, unsigned depth, Env& env)
{
    const Profile* profile = 0L;
    const FeatureProfile* inputFP = env.input->getFeatureProfile();
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
            env.input->getFeatureProfile(),
            env.input->getSchema(),
            env.input->getGeometryType(),
            NULL);

        if (outputStatus.isError())
            return usage(outputStatus.message().c_str());
    }

    std::vector<FeatureData> data;
    int count = env.input->getFeatureCount();
    if (count > 0)
    {
        //OE_INFO << "Feature count = " << count << std::endl;
        data.reserve(count);
    }
    else
    {
        OE_INFO << "Feature count not available" << std::endl;
    }

    // Just divide the error by 10 each level. Placeholder for something smarter.
    double error = env.geometricError[depth];

    FeatureList features;
    unsigned i = 0;
    Query query;
    query.bounds() = extent.bounds();
    osg::ref_ptr<FeatureCursor> cursor = env.input->createFeatureCursor(query, 0L);
    if (!cursor.valid())
    {
        OE_WARN << "Feature cursor error" << std::endl;
        return 0;
    }

    cursor->fill(features);
    if (features.empty())
        return 0;

    ResampleFilter resample(error, DBL_MAX);
    FilterContext fc(0L, env.input->getFeatureProfile(), extent);
    resample.push(features, fc);
    
    for(FeatureList::iterator i = features.begin(); i != features.end(); ++i)
    {
        Feature* f = i->get();
        const GeoExtent& fex = f->getExtent();
        if (fex.width() < error && fex.height() < error)
            continue;

        FeatureData d;
        fex.getCentroid(d.x, d.y);
        d.fid = f->getFID();
        data.push_back(d);
    }

    bool isWide = extent.width() > extent.height();

    OE_INFO << "Depth = " << depth << ", features = " << data.size() << std::endl;

    double median;

    if (data.size() > 0)
    {
        if (isWide)
        {
            SortByX sortByX;
            std::sort(data.begin(), data.end(), sortByX);
            median = ((data.size() & 0x1) == 0) ?
                0.5 * (data[data.size() / 2].x + data[data.size() / 2].x) :
                data[data.size() / 2].x;
            OE_INFO << "  Median X = " << median << std::endl;
        }
        else
        {
            SortByY sortByY;
            std::sort(data.begin(), data.end(), sortByY);
            median = ((data.size() & 0x1) == 0) ?
                0.5 * (data[data.size() / 2].y + data[data.size() / 2].y) :
                data[data.size() / 2].y;
            OE_INFO << "  Median Y = " << median << std::endl;
        }
    }
    else
    {
        if (isWide)
            median = extent.xMin() + extent.width()/2.0;
        else
            median = extent.yMin() + extent.height()/2.0;
    }

    osg::ref_ptr<TDTiles::Tile> child[2];

    child[0] = new TDTiles::Tile();
    child[0]->refine() = TDTiles::REFINE_REPLACE;
    parentTile->children().push_back(child[0].get());

    child[1] = new TDTiles::Tile();
    child[0]->refine() = TDTiles::REFINE_REPLACE;
    parentTile->children().push_back(child[1].get());

    unsigned afeatures = 0u, bfeatures = 0u;

    for (unsigned i = 0; i < data.size(); ++i)
    {
        const FeatureData& d = data[i];
        Feature* f = env.input->getFeature(d.fid);
        double x, y;
        f->getExtent().getCentroid(x, y);
        double side = isWide ? x : y;
        if (side < median)
        {
            afeatures++;
            output[0]->insertFeature(f);
        }
        else
        {
            bfeatures++;
            output[1]->insertFeature(f);
        }
    }

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

    if (depth < env.maxDepth)
    {
        split(a, child[0].get(), depth + 1, env);
        split(b, child[1].get(), depth + 1, env);
    }

    output[0] = 0L;
    output[1] = 0L;

    //TODO: heights
    const float zmin = 0.0f;
    const float zmax = 1.0f;

    const SpatialReference* epsg4979 = SpatialReference::get("epsg:4979");
    a = a.transform(epsg4979);
    b = b.transform(epsg4979);

    if (afeatures > 0u)
        child[0]->content()->uri() = outputConf[0].url().get();

    child[0]->boundingVolume()->region()->set(
        osg::DegreesToRadians(a.xMin()), osg::DegreesToRadians(a.yMin()), zmin, 
        osg::DegreesToRadians(a.xMax()), osg::DegreesToRadians(a.yMax()), zmax);
    child[0]->geometricError() = error;

    if (bfeatures > 0u)
        child[1]->content()->uri() = outputConf[1].url().get();

    child[1]->boundingVolume()->region()->set(
        osg::DegreesToRadians(b.xMin()), osg::DegreesToRadians(b.yMin()), zmin,
        osg::DegreesToRadians(b.xMax()), osg::DegreesToRadians(b.yMax()), zmax);
    child[1]->geometricError() = error;

    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if (arguments.read("--help"))
        return usage("Help!");

    std::string source;
    if (!arguments.read("--in", source))
        return usage("Missing required --in argument");

    std::string outputLocation;
    if (!arguments.read("--out", outputLocation))
        return usage("Missing required --out argument");

    std::string errors("1,80,200");
    arguments.read("--errors", errors);

    if (!osgDB::makeDirectory(outputLocation))
        return usage("Unable to create/find output location");

    osg::ref_ptr<TDTiles::Tile> root = new TDTiles::Tile();

    std::string outFile = Stringify() << outputLocation << "/tileset.json";
    URIContext uriContext(outFile);

    Config inputConf;
    inputConf.set("driver", "ogr");
    inputConf.set("url", source);
    osg::ref_ptr<FeatureSource> input = FeatureSourceFactory::create(ConfigOptions(inputConf));
    if (!input.valid())
        return usage("Failed to create the input feature source");

    if (input->open().isError())
        return usage("Failed to open the input feature source");

    std::vector<std::string> errorStrings;
    StringTokenizer(errors, errorStrings, ",");
    if (errorStrings.size() < 1)
        return usage("Illegal errors input");

    Env env;
    env.input = input.get();
    env.counter = 0;
    env.uriContext = uriContext;    
    env.geometricError = new double[errorStrings.size()];
    env.maxDepth = errorStrings.size()-1;

    // generate level errors:
    OE_NOTICE << "Geometric errors: ";
    for(int i=0; i<errorStrings.size(); ++i)
    {
        int j = errorStrings.size() - 1 - i;
        env.geometricError[j] = osgEarth::as<double>(errorStrings[i], -1.0);
        if (env.geometricError[j] <= 0.0)
            return usage("Illegal geometric error in input");

        OE_NOTIFY(osg::NOTICE, env.geometricError[j] << " ");
    }
    OE_NOTIFY(osg::NOTICE, std::endl);

    // make it so
    split(input->getFeatureProfile()->getExtent(), root, 0, env);

    osg::ref_ptr<TDTiles::Tileset> tileset = new TDTiles::Tileset();
    tileset->root() = root.get();

    TDTiles::Asset asset;
    tileset->asset()->version() = "1.0";

    std::ofstream out(outFile.c_str());
    Json::Value tilesetJSON = tileset->getJSON();
    Json::StyledStreamWriter writer;
    writer.write(out, tilesetJSON);
    out.close();

    return 0;
}
