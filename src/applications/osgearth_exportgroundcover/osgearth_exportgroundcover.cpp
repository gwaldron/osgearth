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
#include <osgEarthSplat/GroundCoverLayer>
#include <osgEarthSplat/NoiseTextureFactory>
#include <OpenThreads/Thread>

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
        << "\n  --layer layername                     ; name of GroundCover layer"
        << "\n  --extents swlong swlat nelong nelat   ; extents in degrees"
        << "\n  --out out.shp                         ; output features"
        << std::endl;

    return -1;
}

// biome weighted index LUT
struct BillboardLUTEntry {
    float width;
    float height;
    float sizeVariation;
};
typedef std::vector<BillboardLUTEntry> BillboardLUT;
typedef UnorderedMap<const GroundCoverBiome*, BillboardLUT> BiomeLUT;
void buildLUT(const GroundCover* gc, BiomeLUT& lut)
{
    for(GroundCoverBiomes::const_iterator b = gc->getBiomes().begin();
        b != gc->getBiomes().end();
        ++b)
    {
        BillboardLUT& billboards = lut[b->get()];

        for (GroundCoverObjects::const_iterator i = b->get()->getObjects().begin();
            i != b->get()->getObjects().end();
            ++i)
        {
            const GroundCoverBillboard* bb = static_cast<const GroundCoverBillboard*>(i->get());
            if (bb)
            {
                unsigned weight = 1u;
                BillboardLUTEntry entry;
                if (bb->_symbol.valid())
                {
                    entry.width = bb->_symbol->width().get();
                    entry.height = bb->_symbol->height().get();
                    entry.sizeVariation = bb->_symbol->sizeVariation().get();
                    weight = bb->_symbol->selectionWeight().get();
                }
                for(unsigned w=0; w<weight; ++w)
                {
                    billboards.push_back(entry);
                }               
            }
        }
    }
}

// GLSL functions :)
float fract(float x)
{
    return fmodf(x, 1.0f);
}

float clamp(float x, float m0, float m1)
{
    return osg::clampBetween(x, m0, m1);
}

void sample(osg::Vec4f& output, ImageUtils::PixelReader& texture, const osg::Matrixf& matrix, float u, float v)
{
    u = clamp(u*matrix(0, 0) + matrix(3, 0), 0.0f, 1.0f);
    v = clamp(v*matrix(1, 1) + matrix(3, 1), 0.0f, 1.0f);
    return texture(output, u, v);
}

const int NOISE_SMOOTH = 0;
const int NOISE_RANDOM = 1;
const int NOISE_RANDOM_2 = 2;
const int NOISE_CLUMPY = 3;

struct App
{
    osg::ref_ptr<MapNode> mapNode;
    const Map* map;
    GeoExtent extent;
    GroundCoverLayer* gclayer;
    LandCoverLayer* lclayer;
    LandCoverDictionary* lcdict;
    ImageLayer* masklayer;
    ElevationLayer* elevlayer;
    osg::ref_ptr<OGRFeatureSource> outfs;
    osg::ref_ptr<osg::Texture> noiseTexture;
    ImageUtils::PixelReader sampleNoise;
    CreateTileModelFilter layerFilter;
    TerrainTileModelFactory* factory;

    Threading::Lockable<std::queue<FeatureList*> > outputQueue;
    Threading::Event gate;

    App() : sampleNoise(NULL) { }

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

        // find layers
        map = mapNode->getMap();

        gclayer = map->getLayerByName<GroundCoverLayer>(layername);
        if (!gclayer)
            return usage(argv[0], "GroundCover layer not found in map");

        lclayer = map->getLayer<LandCoverLayer>();
        if (!lclayer)
            std::cout << "** Note: no LandCover layer found in the map" << std::endl;

        lcdict = map->getLayer<LandCoverDictionary>();
        if (lclayer && !lcdict)
            return usage(argv[0], "No LandCoverDictionary found in the map");

        masklayer = gclayer->getMaskLayer(); // could be null.

        // any elevation layer will do
        elevlayer = map->getLayer<ElevationLayer>();

        // open layers
        if (lclayer && lclayer->open().isError())
            return usage(argv[0], lclayer->getStatus().toString());

        if (masklayer && masklayer->open().isError())
            return usage(argv[0], masklayer->getStatus().toString());

        if (gclayer->open().isError())
            return usage(argv[0], gclayer->getStatus().toString());

        if (elevlayer && elevlayer->open().isError())
            return usage(argv[0], elevlayer->getStatus().toString());

        // create output shapefile
        osg::ref_ptr<FeatureProfile> outProfile = new FeatureProfile(extent);
        FeatureSchema outSchema;
        outSchema["elevation"] = ATTRTYPE_DOUBLE;
        outSchema["width"] = ATTRTYPE_DOUBLE;
        outSchema["height"] = ATTRTYPE_DOUBLE;
        outfs = new OGRFeatureSource();
        outfs->setOGRDriver("ESRI Shapefile");
        outfs->setURL(outfile);
        if (outfs->create(outProfile.get(), outSchema, Geometry::TYPE_POINT, NULL).isError())
            return usage(argv[0], outfs->getStatus().toString());

        // create noise texture
        NoiseTextureFactory noise;
        noiseTexture = noise.create(256u, 4u);
        sampleNoise.setTexture(noiseTexture.get());

        // set up the factory
        if (lclayer) layerFilter.layers().insert(lclayer->getUID());
        if (masklayer)layerFilter.layers().insert(masklayer->getUID());
        if (elevlayer) layerFilter.layers().insert(elevlayer->getUID());
        if (gclayer) layerFilter.layers().insert(gclayer->getUID());
        factory = new TerrainTileModelFactory(const_cast<const MapNode*>(mapNode.get())->options().terrain().get());

        return 0;
    }

    void exportKey(const TileKey& key)
    {
        osg::Vec4f landCover, mask, elev;
        osg::ref_ptr<TerrainTileModel> model = factory->createStandaloneTileModel(map, key, layerFilter, NULL, NULL);
        if (model.valid())
        {
            FeatureList* output = new FeatureList();

            // for now, default to zone 0
            Zone* zone = gclayer->getZones()[0].get();
            if (zone)
            {
                GroundCover* groundcover = zone->getGroundCover();
                if (groundcover)
                {
                    // mask texture/matrix:
                    osg::Texture* maskTex = NULL;
                    osg::Matrix maskMat;
                    if (masklayer)
                    {
                        maskTex = model->getTexture(masklayer->getUID());
                        osg::RefMatrixf* r = model->getMatrix(masklayer->getUID());
                        if (r) maskMat = *r;
                    }
                    ImageUtils::PixelReader maskSampler(maskTex ? maskTex->getImage(0) : NULL);

                    // landcover texture/matrix:
                    osg::Texture* lcTex = NULL;
                    osg::Matrixf lcMat;
                    if (lclayer)
                    {
                        lcTex = model->getLandCoverTexture();
                        if (!lcTex)
                        {
                            OE_WARN << "No land cover texture for this key..." << std::endl;
                            delete output;
                            return;
                        }
                        osg::RefMatrixf* r = model->getLandCoverTextureMatrix();
                        if (r) lcMat = *r;
                    }
                    ImageUtils::PixelReader lcSampler;
                    lcSampler.setTexture(lcTex);

                    // elevation
                    osg::Texture* elevTex = NULL;
                    osg::Matrix elevMat;
                    if (model->elevationModel().valid())
                    {
                        elevTex = model->getElevationTexture();
                        osg::RefMatrixf* r = model->getElevationTextureMatrix();
                        if (r) elevMat = *r;
                    }
                    ImageUtils::PixelReader elevSampler;
                    elevSampler.setTexture(elevTex);

                    // because in the shader oe_terrain_getElevation adjusts the sampling
                    // with scale coefficients:
                    elevSampler.setSampleAsTexture(false);

                    BiomeLUT biomeLUT;
                    buildLUT(groundcover, biomeLUT);

                    unsigned lod = gclayer->getLOD();
                    unsigned tx, ty;
                    map->getProfile()->getNumTiles(lod, tx, ty);
                    GeoExtent e = TileKey(lod, tx / 2, ty / 2, map->getProfile()).getExtent();
                    GeoCircle c = e.computeBoundingGeoCircle();
                    double tileWidth_m = 2.0 * c.getRadius() / 1.4142;
                    float spacing_m = groundcover->getSpacing();
                    unsigned vboTileSize = (unsigned)(tileWidth_m / spacing_m);

                    int numInstancesX = vboTileSize;
                    int numInstancesY = vboTileSize;
                    unsigned totalNumInstances = numInstancesX * numInstancesY;

                    osg::Vec2f offset, halfSpacing, tilec, shift;
                    osg::Vec4f noise(0,0,0,0);

                    int instanceID = 0;

                    for(unsigned instanceID = 0; instanceID < totalNumInstances; ++instanceID)
                    {
                        offset.set(
                            (float)(instanceID % numInstancesX),
                            (float)(instanceID / numInstancesY));

                        halfSpacing.set(
                            0.5f / (float)numInstancesX,
                            0.5f / (float)numInstancesY);

                        tilec.set(
                            halfSpacing.x() + offset.x() / (float)numInstancesX,
                            halfSpacing.y() + offset.y() / (float)numInstancesY);

                        sampleNoise(noise, tilec.x(), tilec.y());

                        // check the fill
                        if (noise[NOISE_SMOOTH] > groundcover->getFill())
                            continue;
                        else
                            noise[NOISE_SMOOTH] /= groundcover->getFill();

                        shift.set(
                            fract(noise[NOISE_RANDOM]*1.5)*2.0f - 1.0f,
                            fract(noise[NOISE_RANDOM_2]*1.5)*2.0f - 1.0f);

                        tilec.x() += shift.x()*halfSpacing.x();
                        tilec.y() += shift.y()*halfSpacing.y();

                        // check the land cover
                        const GroundCoverBiome* biome = NULL;
                        const LandCoverClass* lcclass = NULL;
                        if (lcTex)
                        {
                            sample(landCover, lcSampler, lcMat, tilec.x(), tilec.y());
                            lcclass = lcdict->getClassByValue((int)landCover.r());
                            if (lcclass == NULL)
                                continue;
                            biome = groundcover->getBiome(lcclass);
                            if (!biome)
                                continue;
                        }

                        // check the mask
                        if (maskTex)
                        {
                            sample(mask, maskSampler, maskMat, tilec.x(), tilec.y());
                            if (mask.r() > 0.0)
                                continue;
                        }

                        // clamp
                        float z = 0.0;
                        if (elevTex)
                        {
                            sample(elev, elevSampler, elevMat, tilec.x(), tilec.y());
                            //if (fabs(elev.r()) > 30000.0f)
                            //{
                            //    OE_WARN << "That's a problem.. elevation value is " << elev.r() << std::endl;
                            //    osg::Vec4f val;
                            //    sample(val, elevSampler, elevMat, tilec.x(), tilec.y());
                            //    OE_WARN << "val.r() = " << val.r() << std::endl;
                            //    
                            //}
                            if (elev.r() != NO_DATA_VALUE)
                            {
                                z = elev.r();
                            }
                        }

                        // keeper
                        Point* point = new Point();

                        point->push_back(osg::Vec3d(
                            key.getExtent().xMin() + tilec.x()*key.getExtent().width(),
                            key.getExtent().yMin() + tilec.y()*key.getExtent().height(),
                            0.0));

                        osg::ref_ptr<Feature> feature = new Feature(point, key.getExtent().getSRS());
                        feature->set("elevation", z);

                        // Resolve the symbol so we can add attributes
                        if (biome)
                        {
                            BillboardLUT& bblut = biomeLUT[biome];
                            unsigned index = (unsigned)clamp(noise[NOISE_RANDOM], 0.0, 0.9999999) * (float)(bblut.size());
                            BillboardLUTEntry& bb = bblut[index];
                            float sizeScale = bb.sizeVariation * (noise[NOISE_RANDOM_2] * 2.0 - 1.0);
                            float width = bb.width + bb.width*sizeScale;
                            float height = bb.height + bb.height*sizeScale;
                            feature->set("width", width);
                            feature->set("height", height);
                        }

                        output->push_back(feature.get());
                    }
                }
            }

            outputQueue.lock();
            outputQueue.push(output);
            gate.set();
            outputQueue.unlock();
        }
    }

    ~App()
    {
        delete factory;
    }
};

struct ExportOperation : public osg::Operation
{
    App& _app;
    TileKey _key;

    ExportOperation(App& app, const TileKey& key) : osg::Operation("build", false), _app(app), _key(key) { }

    void operator()(osg::Object*)
    {
        _app.exportKey(_key);
    }
};

int
main(int argc, char** argv)
{
    App app;

    if (app.open(argc, argv) < 0)
        return -1;

    // find all intersecting tile keys
    std::vector<TileKey> keys;
    app.map->getProfile()->getIntersectingTiles(app.extent, app.gclayer->getLOD(), keys);
    if (keys.empty())
        return usage(argv[0], "Invalid extent");

    osg::ref_ptr<ThreadPool> pool = new ThreadPool(4u);

    for (std::vector<TileKey>::const_iterator i = keys.begin();
        i != keys.end();
        ++i)
    {
        pool->getQueue()->add(new ExportOperation(app, *i));
    }

    for(unsigned i=0; i<keys.size(); )
    {
        app.gate.waitAndReset();

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
            FeatureList* output = outputs[j];
            for(FeatureList::iterator k = output->begin(); k != output->end(); ++k)
            {
                app.outfs->insertFeature(k->get());
            }
            delete output;
            std::cout << "\r" << (++i) << "/" << keys.size() << std::flush;
        }
    }

    std::cout << "\nBuilding index.." << std::flush;
    app.outfs->buildSpatialIndex();

    app.outfs->close();

    std::cout << "\rDone!           " << std::endl;
}