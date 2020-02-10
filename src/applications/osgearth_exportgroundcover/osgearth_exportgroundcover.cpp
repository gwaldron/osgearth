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
#include <osgEarth/MapNode>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/Feature>
#include <osgEarth/TerrainTileModelFactory>
#include <osgEarth/LandCover>
#include <osgEarthSplat/GroundCoverLayer>
#include <osgEarthSplat/NoiseTextureFactory>


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

    return 0;
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
double fract(double x)
{
    return fmod(x, 1.0);
}
double clamp(double x, double m0, double m1)
{
    return osg::clampBetween(x, m0, m1);
}

void sample(osg::Vec4f& output, ImageUtils::PixelReader& texture, const osg::Matrixf& matrix, float u, float v)
{
    return texture(output, u*matrix(0,0)+matrix(3,0), v*matrix(1,1)+matrix(3,1));
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    std::string layername;
    if (!arguments.read("--layer", layername))
        return usage(argv[0], "Missing --layer");

    double xmin, ymin, xmax, ymax;
    if (!arguments.read("--extents", xmin, ymin, xmax, ymax))
        return usage(argv[0], "Missing --extents");
    GeoExtent extent(SpatialReference::get("wgs84"), xmin, ymin, xmax, ymax);

    std::string outfile;
    if (!arguments.read("--out", outfile))
        return usage(argv[0], "Missing --out");

    osg::ref_ptr<MapNode> mapNode = MapNode::load(arguments);
    if (!mapNode.valid())
        return usage(argv[0], "No earth file");

    // find layers
    const Map* map = mapNode->getMap();

    GroundCoverLayer* gclayer = map->getLayerByName<GroundCoverLayer>(layername);
    if (!gclayer)
        return usage(argv[0], "GroundCover layer not found in map");

    LandCoverLayer* lclayer = map->getLayer<LandCoverLayer>();
    if (!lclayer)
        std::cout << "** Note: no LandCover layer found in the map" << std::endl;

    LandCoverDictionary* lcdict = map->getLayer<LandCoverDictionary>();
    if (lclayer && !lcdict)
        return usage(argv[0], "No LandCoverDictionary found in the map");

    ImageLayer* masklayer = gclayer->getMaskLayer(); // could be null.

    // any elevation layer will do
    ElevationLayer* elevlayer = map->getLayer<ElevationLayer>();

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
    osg::ref_ptr<OGRFeatureSource> outfs = new OGRFeatureSource();
    outfs->setOGRDriver("ESRI Shapefile");
    outfs->setURL(outfile);
    if (outfs->create(outProfile.get(), outSchema, Geometry::TYPE_POINT, NULL).isError())
        return usage(argv[0], outfs->getStatus().toString());

    // create noise texture
    NoiseTextureFactory noise;
    osg::ref_ptr<osg::Texture> noiseTexture = noise.create(256u, 4u);
    const int NOISE_SMOOTH = 0;
    const int NOISE_RANDOM = 1;
    const int NOISE_RANDOM_2 = 2;
    const int NOISE_CLUMPY = 3;
    ImageUtils::PixelReader noiseSampler(noiseTexture->getImage(0));

    // find all intersecting tile keys
    std::vector<TileKey> keys;
    map->getProfile()->getIntersectingTiles(extent, gclayer->getLOD(), keys);
    if (keys.empty())
        return usage(argv[0], "Invalid extent");

    // set up the factory
    CreateTileModelFilter layerFilter;
    if (lclayer) layerFilter.layers().insert(lclayer->getUID());
    if (masklayer)layerFilter.layers().insert(masklayer->getUID());
    if (elevlayer) layerFilter.layers().insert(elevlayer->getUID());
    if (gclayer) layerFilter.layers().insert(gclayer->getUID());
    TerrainTileModelFactory factory(const_cast<const MapNode*>(mapNode.get())->options().terrain().get());

    int count = 0;
    osg::Vec4f landCover, mask, elev;

    for(std::vector<TileKey>::const_iterator i = keys.begin();
        i != keys.end();
        ++i)
    {
        ++count;
        std::cout << "\r" << count << "/" << keys.size() << std::flush;

        const TileKey& key = *i;
        osg::ref_ptr<TerrainTileModel> model = factory.createStandaloneTileModel(map, key, layerFilter, NULL, NULL);
        if (model.valid())
        {
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
                    ImageUtils::PixelReader maskSampler(maskTex? maskTex->getImage(0) : NULL);

                    // landcover texture/matrix:
                    osg::Texture* lcTex = NULL;
                    osg::Matrixf lcMat;
                    if (lclayer)
                    {
                        lcTex = model->getLandCoverTexture();
                        if (!lcTex)
                        {
                            OE_WARN << "No land cover texture for this key..." << std::endl;
                            continue;
                        }
                        osg::RefMatrixf* r = model->getLandCoverTextureMatrix();
                        if (r) lcMat = *r;
                    }
                    ImageUtils::PixelReader lcSampler(lcTex ? lcTex->getImage(0) : NULL);

                    // elevation
                    osg::Texture* elevTex = NULL;
                    osg::Matrix elevMat;
                    if (model->elevationModel().valid())
                    {
                        elevTex = model->getElevationTexture();
                        osg::RefMatrixf* r = model->getElevationTextureMatrix();
                        if (r) elevMat = *r;
                    }
                    ImageUtils::PixelReader elevSampler(elevTex ? elevTex->getImage(0) : NULL);
                    elevSampler.setBilinear(true);

                    BiomeLUT biomeLUT;
                    buildLUT(groundcover, biomeLUT);


                    osg::Vec2f numInstances(128, 128); // TODO: fetch this.
                    unsigned numS = (unsigned)numInstances.x();
                    unsigned numT = (unsigned)numInstances.y();

                    osg::Vec2f offset, halfSpacing, tilec, shift;
                    osg::Vec4f noise(1,1,1,1);

                    for(unsigned t=0; t< numT; ++t)
                    {
                        for(unsigned s=0; s< numS; ++s)
                        {
                            float instanceID = (float)(t*numT + s);
                            
                            offset.set(
                                fmod(instanceID, (float)numS),
                                instanceID / (float)numT);

                            halfSpacing.set(
                                0.5/(float)numS,
                                0.5/(float)numT);

                            tilec.set(
                                halfSpacing.x() + offset.x()/numInstances.x(),
                                halfSpacing.y() + offset.y()/numInstances.y());

                            float u = (float)s / (float)(numS-1);
                            float v = (float)t / (float)(numT-1);
                            noiseSampler(noise, u, v);

                            shift.set(
                                fract(noise[NOISE_RANDOM]*5.5)*2.0-1.0,
                                fract(noise[NOISE_RANDOM_2]*5.5)*2.0-1.0);

                            tilec.x() += shift.x()*halfSpacing.x();
                            tilec.y() += shift.y()*halfSpacing.y();

                            // check the fill
                            if (noise[NOISE_SMOOTH] > groundcover->getFill())
                                continue;                         
                            else if (noise[NOISE_SMOOTH] > 0.0)
                                noise[NOISE_SMOOTH] /= groundcover->getFill();

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
                                if (elev.r() != NO_DATA_VALUE)
                                    z = elev.r();
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
                                unsigned index = (unsigned)clamp(noise[NOISE_RANDOM],0.0,0.9999999) * (float)(bblut.size());
                                BillboardLUTEntry& bb = bblut[index];
                                float sizeScale = bb.sizeVariation * (noise[NOISE_RANDOM_2]*2.0-1.0);
                                float width = bb.width + bb.width*sizeScale;
                                float height = bb.height + bb.height*sizeScale;
                                feature->set("width", width);
                                feature->set("height", height);
                            }

                            outfs->insertFeature(feature.get());
                        }
                    }
                }
            }
        }
    }

    std::cout << "\rBuilding index.." << std::endl;
    outfs->buildSpatialIndex();

    std::cout << std::endl;
    outfs->close();
}