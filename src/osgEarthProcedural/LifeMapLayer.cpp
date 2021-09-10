/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include "LifeMapLayer"
#include "NoiseTextureFactory"

#include <osgEarth/Map>
#include <osgEarth/ElevationPool>
#include <osgEarth/Math>
#include <osgEarth/MetaTile>
#include <osgEarth/rtree.h>

#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>

#define LC "[LifeMapLayer] " << getName() << ": "

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(lifemap, LifeMapLayer);

//........................................................................

class GeoImageReader
{
public:
    const GeoImage& _i;
    const GeoExtent& _ex;
    ImageUtils::PixelReader _reader;
    GeoImageReader(const GeoImage& i) : 
        _i(i), _ex(i.getExtent()), _reader(_i.getImage()) { 
        _reader.setBilinear(false);
        _reader.setDenormalize(false);
    }
    void read(osg::Vec4& out, double x, double y) {
        double u = (x - _ex.xMin()) / _ex.width();
        double v = (y - _ex.yMin()) / _ex.height();
        _reader(out, u, v);
    }
};

//........................................................................

Config
LifeMapLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    biomeLayer().set(conf, "biomes_layer");
    maskLayer().set(conf, "mask_layer");
    colorLayer().set(conf, "color_layer");
    landUseLayer().set(conf, "land_use_layer");
    conf.set("use_land_cover", useLandCover());
    conf.set("use_terrain", useTerrain());
    return conf;
}

void
LifeMapLayer::Options::fromConfig(const Config& conf)
{
    useLandCover().setDefault(false);
    useTerrain().setDefault(true);
    useLandUse().setDefault(true); // if the layer is set (not serialized)
    useColor().setDefault(true); // if the layer is set (not serialized)
    landCoverWeight() = 1.0f;
    landCoverBlur() = 0.1f;
    terrainWeight() = 1.0f;
    colorWeight() = 1.0f;
    slopeIntensity() = 1.0f;

    biomeLayer().get(conf, "biomes_layer");
    maskLayer().get(conf, "mask_layer");
    colorLayer().get(conf, "color_layer");
    landUseLayer().get(conf, "land_use_layer");
    conf.get("use_land_cover", useLandCover());
    conf.get("use_terrain", useTerrain());
}

//........................................................................

namespace
{
    // noise channels
    constexpr unsigned SMOOTH = 0;
    constexpr unsigned RANDOM = 1;
    constexpr unsigned RANDOM2 = 2;
    constexpr unsigned CLUMPY = 3;

    // raster sampling coordinate scaler
    void scaleCoordsToRefLOD(osg::Vec2& tc, const TileKey& key, unsigned refLOD)
    {
        if (key.getLOD() <= refLOD)
            return;

        unsigned tilesX, tilesY;
        key.getProfile()->getNumTiles(key.getLOD(), tilesX, tilesY);

        double dL = (double)(key.getLOD() - refLOD);
        double factor = exp2(dL);
        double invFactor = 1.0f/factor;

        double rx = tc.x() * invFactor;
        double ry = tc.y() * invFactor;

        double tx = (double)key.getTileX();
        double ty = (double)(tilesY - key.getTileY() - 1);

        double ax = floor(tx * invFactor);
        double ay = floor(ty * invFactor);

        double bx = ax * factor;
        double by = ay * factor;

        double cx = bx + factor;
        double cy = by + factor;

        if (factor >= 1.0f)
        {
            rx += (tx-bx)/(cx-bx);
            ry += (ty-by)/(cy-by);
        }

        tc.set(rx, ry);
    }

    void getNoise(
        osg::Vec4& noise,
        ImageUtils::PixelReader& read,
        const osg::Vec2& coords)
    {
        read(noise, coords.x(), coords.y());
        //noise *= 2.0;
        //noise.r() -= 1.0, noise.g() -= 1.0, noise.b() -= 1.0, noise.a() -= 1.0;
    }

    struct LandUseTile
    {
        RTree<osg::ref_ptr<Feature>, double, 2> _index;
        int _size;
        const SpatialReference* _tilesrs;
        const SpatialReference* _featuresrs;

        LandUseTile() : _size(0) { }

        bool empty() const { return _size == 0; }

        void load(
            const TileKey& key, 
            FeatureSource* fs, 
            FeatureFilterChain* filters, 
            const Distance& buffer, 
            const LifeMapValueTable* table)
        {
            _tilesrs = key.getExtent().getSRS();
            _featuresrs = fs->getFeatureProfile()->getSRS();

            osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(key, buffer, filters, nullptr, nullptr);
            while (cursor.valid() && cursor->hasMore())
            {
                Feature* f = cursor->nextFeature();

                if (f->getGeometry()->isPolygon() && (
                    (table->getValue("landuse." + f->getString("landuse")) ||
                    (table->getValue("natural." + f->getString("natural"))))))
                {
                    //OE_INFO << "landuse=" << f->getString("landuse") << ", natural=" << f->getString("natural") << std::endl;
                    const GeoExtent& ex = f->getExtent();
                    double a_min[2] = { ex.xMin(), ex.yMin() };
                    double a_max[2] = { ex.xMax(), ex.yMax() };
                    _index.Insert(a_min, a_max, osg::ref_ptr<Feature>(f));
                    ++_size;
                }
            }
        }

        const LifeMapValue* get(double x, double y, const LifeMapValueTable* table) const
        {
            _tilesrs->transform2D(x, y, _featuresrs, x, y);

            constexpr double e = 0.0;
            double a_min[2] = { x-e, y-e };
            double a_max[2] = { x+e, y+e };

            std::vector<osg::ref_ptr<Feature>> hits;

            if (_index.Search(a_min, a_max, &hits, ~0) == 0)
                return nullptr;

            const LifeMapValue* result = nullptr;

            for (const auto& f : hits)
            {
                if (f->getGeometry()->contains2D(x, y))
                {
                    result = table->getValue("landuse." + f->getString("landuse"));
                    if (result)
                        return result;

                    result = table->getValue("natural." + f->getString("natural"));
                    if (result)
                        return result;
                }
            }
            return nullptr;
        }
    };
}

//........................................................................

void
LifeMapLayer::init()
{
    ImageLayer::init();

    // LifeMap is invisible AND shared by default.
    options().visible().setDefault(false);
    options().shared().setDefault(true);

    NoiseTextureFactory nf;
    _noiseFunc = nf.createImage(1024u, 4u);
}

Status
LifeMapLayer::openImplementation() 
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    options().maskLayer().open(getReadOptions());

    options().colorLayer().open(getReadOptions());

    options().landUseLayer().open(getReadOptions());

    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
    return Status::OK();
}

Status
LifeMapLayer::closeImplementation()
{
    return ImageLayer::closeImplementation();
}

void
LifeMapLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    options().biomeLayer().addedToMap(map);
    options().maskLayer().addedToMap(map);
    options().colorLayer().addedToMap(map);
    options().landUseLayer().addedToMap(map);

    // not specified; try to find it
    if (!getBiomeLayer())
        setBiomeLayer(map->getLayer<BiomeLayer>());
    
    Status biomeStatus = options().biomeLayer().open(getReadOptions());
    if (getBiomeLayer())
    {
        OE_INFO << LC << "Using biome layer \"" << getBiomeLayer()->getName() << "\"" << std::endl;
    }

    _map = map;

    if (getUseLandCover() == true)
    {
        _landCoverDictionary = map->getLayer<LandCoverDictionary>();
        if (!_landCoverDictionary.valid())
        {
            OE_WARN << LC 
                << "No land cover dictionary found in the map;"
                << " Land cover data will not be used."
                << std::endl;
        }

        map->getOpenLayers(_landCoverLayers);
        if (_landCoverLayers.empty())
        {
            OE_WARN << LC
                << "No land cover layers found in the map;"
                << " Land cover data will not be used."
                << std::endl;
        }
    }
}

void
LifeMapLayer::removedFromMap(const Map* map)
{
    _map = nullptr;
    options().biomeLayer().removedFromMap(map);
    options().maskLayer().removedFromMap(map);
    options().colorLayer().removedFromMap(map);
    ImageLayer::removedFromMap(map);
}

void
LifeMapLayer::setBiomeLayer(BiomeLayer* layer)
{
    options().biomeLayer().setLayer(layer);
}

BiomeLayer*
LifeMapLayer::getBiomeLayer() const
{
    return options().biomeLayer().getLayer();
}

void
LifeMapLayer::setMaskLayer(ImageLayer* layer)
{
    options().maskLayer().setLayer(layer);
}

ImageLayer*
LifeMapLayer::getMaskLayer() const
{
    return options().maskLayer().getLayer();
}

void
LifeMapLayer::setLandUseLayer(FeatureSource* layer)
{
    options().landUseLayer().setLayer(layer);
}

FeatureSource*
LifeMapLayer::getLandUseLayer() const
{
    return options().landUseLayer().getLayer();
}

void
LifeMapLayer::setColorLayer(ImageLayer* layer)
{
    options().colorLayer().setLayer(layer);
}

ImageLayer*
LifeMapLayer::getColorLayer() const
{
    return options().colorLayer().getLayer();
}

void
LifeMapLayer::setUseLandCover(bool value)
{
    options().useLandCover() = value;
}

bool
LifeMapLayer::getUseLandCover() const
{
    return options().useLandCover().get();
}

void
LifeMapLayer::setUseTerrain(bool value)
{
    options().useTerrain() = value;
}

bool
LifeMapLayer::getUseTerrain() const
{
    return options().useTerrain().get();
}

void
LifeMapLayer::setUseLandUse(bool value)
{
    options().useLandUse() = value;
}

bool
LifeMapLayer::getUseLandUse() const
{
    return options().useLandUse().get();
}

void
LifeMapLayer::setUseColor(bool value)
{
    options().useColor() = value;
}

bool
LifeMapLayer::getUseColor() const
{
    return options().useColor().get();
}

namespace
{
    struct Value
    {
        Value() : value(0.0f), weight(0.0f) { }
        float value;
        float weight;
    };

    struct Sample
    {
        Sample() : dense(), lush(), rugged(), special(0), weight(0.0f) { }
        Value dense;
        Value lush;
        Value rugged;
        int special;
        float weight;
    };

    #define TERRAIN 0
    #define LANDUSE 1
    #define LANDCOVER 2
    #define COLOR 3
}

GeoImage
LifeMapLayer::createImageImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return GeoImage::INVALID;

    // collect the elevation data:
    osg::ref_ptr<ElevationTexture> elevTile;
    ElevationPool* ep = map->getElevationPool();
    ep->getTile(key, true, elevTile, &_workingSet, progress);

    if (!elevTile.valid())
        return GeoImage::INVALID;

    // ensure we have a normal map for slopes and curvatures:
    elevTile->generateNormalMap(map.get(), &_workingSet, progress);

    // if we're using land use data, fetch that now:
    LandUseTile landuse;
    const LifeMapValueTable* landuse_table = nullptr;

    TileKey lu_key = key.getLOD() > 14u ? key.createAncestorKey(14u) : key;
    const GeoExtent& lu_extent = lu_key.getExtent();
    double x_jitter = lu_extent.width() * 0.1;
    double y_jitter = lu_extent.height() * 0.1;

    if (getLandUseLayer() && getUseLandUse() && getBiomeLayer() && getBiomeLayer()->getBiomeCatalog())
    {
        // the catalog:
        landuse_table = getBiomeLayer()->getBiomeCatalog()->getLandUseTable();

        // populate the tile with features that exist in the catalog:
        landuse.load(
            lu_key,
            getLandUseLayer(),
            nullptr, // filters
            Distance(std::max(x_jitter, y_jitter), lu_extent.getSRS()->getUnits()),
            landuse_table);
    }

    GeoExtent extent = key.getExtent();

    // bring in the land cover data if requested:
    osg::ref_ptr<osg::Image> landcover;
    ImageUtils::PixelReader readLandCover;
    osg::Vec4 landcover_pixel;
    osg::Matrixf landcover_matrix;
    const LifeMapValueTable* landcover_table;
    std::unordered_map<std::string, int> specialTextureIndexLUT;


    TileKeyMetaImage landcover_metaimage;



    if (getBiomeLayer())
    {
        landcover_table = getBiomeLayer()->getBiomeCatalog()->getLandCoverTable();

        if (getUseLandCover() && !_landCoverLayers.empty() && _landCoverDictionary.valid() && landcover_table)
        {
            TileKey landcover_key(key);

            // fall back until we get a valid result
            for (; !landcover.valid() && landcover_key.valid(); landcover_key.makeParent())
            {
                _landCoverLayers.populateLandCoverImage(landcover, landcover_key, progress);
                if (landcover.valid())
                {
                    readLandCover.setImage(landcover.get());
                    readLandCover.setBilinear(false);
                    readLandCover.setSampleAsTexture(true);
                    extent.createScaleBias(landcover_key.getExtent(), landcover_matrix);

                    landcover_metaimage.setTileKey(landcover_key);
                    landcover_metaimage.setCreateImageFunction(
                        [&](const TileKey& key, ProgressCallback* prog)
                        {
                            osg::ref_ptr<osg::Image> output;
                            _landCoverLayers.populateLandCoverImage(output, key, prog);
                            return GeoImage(output.release(), key.getExtent());
                        }
                    );
                }
            }

            int index = 1; // start at one b/c zero means no special asset
            for (auto& tex : getBiomeLayer()->getBiomeCatalog()->getAssets().getSpecialTextures())
            {
                specialTextureIndexLUT[tex.name().get()] = index++;
            }
        }
    }

    GeoImage densityMask;
    ImageUtils::PixelReader readDensityMask;
    osg::Vec4 dm_pixel;
    osg::Matrixf dm_matrix;

    // the mask layer zero's out density(etc)
    if (getMaskLayer())
    {
        TileKey dm_key(key);

        while(dm_key.valid() && !densityMask.valid())
        {
            densityMask = getMaskLayer()->createImage(dm_key, progress);
            if (!densityMask.valid())
                dm_key.makeParent();
        }

        if (densityMask.valid())
        {
            readDensityMask.setImage(densityMask.getImage());
            readDensityMask.setBilinear(true);
            readDensityMask.setSampleAsTexture(true);
            extent.createScaleBias(dm_key.getExtent(), dm_matrix);
        }
    }

    // the color layer alters lifemap values based on colors.
    GeoImage color;
    ImageUtils::PixelReader readColor;
    osg::Vec4 color_pixel;
    osg::Matrixf color_matrix;

    if (getColorLayer() && options().useColor() == true)
    {
        TileKey color_key(key);

        while (color_key.valid() && !color.valid())
        {
            color = getColorLayer()->createImage(color_key, progress);
            if (!color.valid())
                color_key.makeParent();
        }

        if (color.valid())
        {
            readColor.setImage(color.getImage());
            readColor.setBilinear(true);
            readColor.setSampleAsTexture(true);
            extent.createScaleBias(color_key.getExtent(), color_matrix);
        }
    }

    // assemble the image:
    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(
        getTileSize(),
        getTileSize(),
        1,
        GL_RGBA,
        GL_UNSIGNED_BYTE);

    ImageUtils::PixelWriter write(image.get());
    osg::Vec4 pixel, temp;
    //ElevationSample elevSample;
    float elevation;
    osg::Vec3 normal;
    float slope;
    const osg::Vec3 up(0,0,1);
    
    std::string lu_id;
    osg::Vec4f hsl;

    osg::Vec2 noiseCoords[4];
    osg::Vec4 noise[4];
    const unsigned noiseLOD[4] = { 0u, 9u, 13u, 16u };
    ImageUtils::PixelReader noiseSampler(_noiseFunc.get());
    noiseSampler.setBilinear(true);
    noiseSampler.setSampleAsRepeatingTexture(true);

    GeoImage result(image.get(), extent);

    GeoImageIterator i(result);
    i.forEachPixelOnCenter([&]() {

        Sample sample[4];

        // Generate noise:
        for (int n = 0; n < 4; ++n)
        {
            //TODO: well?
            if (true)//key.getLOD() >= noiseLOD[n])
            {
                noiseCoords[n].set(i.u(), i.v());
                scaleCoordsToRefLOD(noiseCoords[n], key, noiseLOD[n]);
                getNoise(noise[n], noiseSampler, noiseCoords[n]);
            }
        }

        // Establish elevation at this pixel:
        elevation = elevTile->getElevation(i.x(), i.y()).elevation().as(Units::METERS);

        // Normal map at this pixel:
        normal = elevTile->getNormal(i.x(), i.y());

        // exaggerate the slope value
        //slope = harden(harden(1.0 - (normal * up)));
        slope = 1.0 - (normal*up);
        slope = clamp(slope*2.0f, 0.0f, 1.0f); // make 0.5 the maximum slope
        slope *= options().slopeIntensity().get();

        // NOISE VALUES:
        float dense_noise =
            (0.3*noise[0][RANDOM]) +
            (0.3*noise[1][SMOOTH]) +
            (0.4*noise[2][CLUMPY]);

        float lush_noise =
            noise[2][SMOOTH];
            //noise[1][SMOOTH];

        float rugged_noise =
            0.5*noise[1][SMOOTH] +
            0.5*noise[2][CLUMPY];


        // LAND USE CONTRIBUTION:
        if (!landuse.empty())
        {
            double xx = i.x() + x_jitter * 0.3*(noise[2][CLUMPY] * 2.0 - 1.0);
            double yy = i.y() + y_jitter * 0.3*(noise[2][SMOOTH] * 2.0 - 1.0);

            const LifeMapValue* lu = landuse.get(xx, yy, landuse_table);

            if (lu)
            {
                if (lu->dense().isSet())
                {
                    sample[LANDUSE].dense.value = lu->dense().get() + 0.2*(dense_noise*2.0 - 1.0);
                    sample[LANDUSE].dense.weight = 1.0f;
                }

                if (lu->lush().isSet())
                {
                    sample[LANDUSE].lush.value = lu->lush().get() + 0.2*(lush_noise*2.0 - 1.0);
                    sample[LANDUSE].lush.weight = 1.0f;
                }

                if (lu->rugged().isSet())
                {
                    sample[LANDUSE].rugged.value = lu->rugged().get() + 0.2*(rugged_noise*2.0 - 1.0);
                    sample[LANDUSE].rugged.weight = 1.0f;
                }

                sample[LANDUSE].weight = 1.0f;
            }
        }

        // LANDCOVER CONTRIBUTION:
        if (landcover.valid())
        {
            double uu = i.u() * landcover_matrix(0, 0) + landcover_matrix(3, 0);
            double vv = i.v() * landcover_matrix(1, 1) + landcover_matrix(3, 1);

#if 1
            // read the landcover with a blurring filter.
            landcover_pixel.set(0, 0, 0, 0);
            double blur = clamp(options().landCoverBlur().get(), 0.0f, 0.9f);
            int lc_kernel_count = 0;
            LifeMapValue lc_temp;
            for (int a = -1; a <= 1; ++a) {
                for (int b = -1; b <= 1; ++b) {
                    osg::Vec4f temp;
                    landcover_metaimage.read(uu + (double)a*blur, vv + (double)b*blur, temp);
                    const LandCoverClass* lcc = _landCoverDictionary->getClassByValue((int)temp.r());
                    if (lcc) {
                        const LifeMapValue* value = landcover_table->getValue(lcc->getName());
                        if (value) {
                            lc_kernel_count++;
                            landcover_pixel[LIFEMAP_DENSE] += value->dense().get();
                            landcover_pixel[LIFEMAP_LUSH] += value->lush().get();
                            landcover_pixel[LIFEMAP_RUGGED] += value->rugged().get();
                        }
                    }
                }
            }
            landcover_pixel /= lc_kernel_count;

            sample[LANDCOVER].dense.value = landcover_pixel[LIFEMAP_DENSE];
            sample[LANDCOVER].dense.weight = 1.0f;
            sample[LANDCOVER].lush.value = landcover_pixel[LIFEMAP_LUSH];
            sample[LANDCOVER].lush.weight = 1.0f;
            sample[LANDCOVER].rugged.value = landcover_pixel[LIFEMAP_RUGGED];
            sample[LANDCOVER].rugged.weight = 1.0f;

            sample[LANDCOVER].weight = options().landCoverWeight().get();

#else
            readLandCover(landcover_pixel, uu, vv);

            const LandCoverClass* lcc = _landCoverDictionary->getClassByValue(
                (int)landcover_pixel.r());

            if (lcc)
            {
                const LifeMapValue* value = landcover_table->getValue(lcc->getName());
                if (value)
                {
                    bool has_special = value->special().isSet();

                    if (has_special)
                    {
                        sample[LANDCOVER].special =
                            specialTextureIndexLUT[value->special().get()];
                    }

                    const int ni = 3;
                    float weight = 1.0f; // 0.65f + 0.35f*noise[2][SMOOTH];

                    if (value->dense().isSet())
                    {
                        float dn = has_special ? 0.0f : (dense_noise*2.0f - 1.0f)*noise[ni][CLUMPY];
                        //sample[LANDCOVER].dense.value = value->dense().get() + 0.2*(dense_noise*2.0 - 1.0);
                        sample[LANDCOVER].dense.value = value->dense().get() + dn;
                        sample[LANDCOVER].dense.weight = weight;
                    }

                    if (value->lush().isSet())
                    {
                        float dn = has_special ? 0.0f : (lush_noise*2.0f - 1.0f)*noise[ni][RANDOM];
                        //sample[LANDCOVER].lush.value = value->lush().get() + 0.2*(lush_noise*2.0 - 1.0);
                        sample[LANDCOVER].lush.value = value->lush().get() + dn;
                        sample[LANDCOVER].lush.weight = weight;
                    }

                    if (value->rugged().isSet())
                    {
                        float dn = has_special ? 0.0f : (rugged_noise*2.0f - 1.0f)*noise[ni][SMOOTH];
                        //sample[LANDCOVER].rugged.value = value->rugged().get() + 0.2*(rugged_noise*2.0 - 1.0);
                        sample[LANDCOVER].rugged.value = value->rugged().get() + dn;
                        sample[LANDCOVER].rugged.weight = weight;
                    }

                    sample[LANDCOVER].weight = options().landCoverWeight().get(); // 1.0f;
                }
            }
#endif
        }

        // COLOR CONTRIBUTION:
        if (color.valid())
        {
            sample[COLOR].weight = options().colorWeight().get(); // 1.0f;

            double uu = i.u() * color_matrix(0, 0) + color_matrix(3, 0);
            double vv = i.v() * color_matrix(1, 1) + color_matrix(3, 1);
            readColor(color_pixel, uu, vv);

            // adjust lifemap based on the "greenness" of the pixel
            // https://www.sciencedirect.com/science/article/pii/S2214317315000347

            // normalize
            //Color c(color_pixel.r(), color_pixel.g(), color_pixel.b(), 0.0f);
            //float total = c.r() + c.g() + c.b();
            //c /= total;

            //float ExG = 2.0f*c.g() - c.r() - c.b();
            //float ExGR = ExG - (1.4f*c.r() - c.g());
            //if (ExGR > 0.0)
            //{
            //    dense *= (1.0 + ExGR);
            //    rugged /= (1.0 + ExGR);
            //}
            //lush = clamp(ExGR, 0.0f, 1.0f);

            ////float VEG = c.g() / (pow(c.r(), 0.667)*pow(c.b(), 0.333));
            //c = color_pixel;
            
            // normalized color: (do this???)
            Color c(color_pixel.r(), color_pixel.g(), color_pixel.b(), 0.0f);
            c /= (c.r() + c.g() + c.b());

            // convert to HSL:
            hsl = c.asHSL();
            //rgb2hsl(c, hsl);

            constexpr float red = 0.0f;
            constexpr float green = 0.3333333f;
            constexpr float blue = 0.6666667f;
            constexpr float green_amp = 3.0f;
            constexpr float lush_amp = 2.0f; // old=1.2f

            float inv_green = fabs(hsl[0] - green);
            if (inv_green > 0.5f) inv_green = 1.0f - inv_green;
            float greenness = 1.0f - 2.0f*inv_green;
            greenness = pow(greenness, green_amp);

            float inv_red = fabs(hsl[0] - red);
            if (inv_red > 0.5f) inv_red = 1.0f - inv_red;
            float redness = 1.0f - 2.0*inv_red;

            sample[COLOR].dense.value = greenness;
            sample[COLOR].dense.weight = 1.0f;

            sample[COLOR].lush.value = clamp(lush_amp * (hsl[1] - 0.5f*hsl[2]), 0.0f, 1.0f);
            sample[COLOR].lush.weight = 1.0f;

            sample[COLOR].rugged.value = (redness + (1.0f - hsl[1]) + hsl[2]) / 3.0f;
            sample[COLOR].rugged.value = clamp(sample[COLOR].rugged.value - 0.5, 0.0, 1.0) * 2.0;
            sample[COLOR].rugged.weight = (getUseTerrain() ? 0.0f : 1.0f);
        }

        // TERRAIN CONTRIBUTION:
        if (getUseTerrain())
        {
            sample[TERRAIN].weight = options().terrainWeight().get();

            if (sample[COLOR].weight == 0.0f)
            {
                sample[TERRAIN].dense.value = -(0.8f * slope);
                sample[TERRAIN].dense.weight = 1.0f;

                sample[TERRAIN].lush.value = -(0.75f*slope);
                sample[TERRAIN].lush.weight = 1.0f;
            }

            sample[TERRAIN].rugged.value = slope;
            sample[TERRAIN].rugged.weight = 1.0f;
        }

        // CALCULATE WEIGHTED AVERAGES:
        pixel.set(0.0f, 0.0f, 0.0f, 0.0f);

        float
            dense_total = 0.0f,
            lush_total = 0.0f,
            rugged_total = 0.0f;

        float
            dense_factor = 0.0f,
            lush_factor = 0.0f,
            rugged_factor = 0.0f;

        for (int i = 0; i < 4; ++i)
        {
            dense_factor = sample[i].dense.weight*sample[i].weight;
            pixel[LIFEMAP_DENSE] += sample[i].dense.value*dense_factor;
            dense_total += dense_factor;
            
            lush_factor = sample[i].lush.weight*sample[i].weight;
            pixel[LIFEMAP_LUSH] += sample[i].lush.value*lush_factor;
            lush_total += lush_factor;

            rugged_factor = sample[i].rugged.weight*sample[i].weight;
            pixel[LIFEMAP_RUGGED] += sample[i].rugged.value*rugged_factor;
            rugged_total += rugged_factor;

            if (sample[i].special > 0)
                pixel[LIFEMAP_SPECIAL] = (float)sample[i].special / 255.0f;
        }

        // for a special encoding, zero out the other values
        if (pixel[LIFEMAP_SPECIAL] > 0)
            pixel[LIFEMAP_RUGGED] = pixel[LIFEMAP_DENSE] = pixel[LIFEMAP_LUSH] = 0.0f;

        // MASK CONTRIBUTION (applied to final data)
        if (densityMask.valid())
        {
            double uu = clamp(i.u() * dm_matrix(0, 0) + dm_matrix(3, 0), 0.0, 1.0);
            double vv = clamp(i.v() * dm_matrix(1, 1) + dm_matrix(3, 1), 0.0, 1.0);
            readDensityMask(dm_pixel, uu, vv);

            pixel[LIFEMAP_DENSE] *= dm_pixel.r();
            pixel[LIFEMAP_LUSH] *= (0.25 + 0.75*dm_pixel.r()); // 75% effect
            pixel[LIFEMAP_RUGGED] *= dm_pixel.r();
        }

        for (int i = 0; i < 4; ++i)
        {
            pixel[i] = clamp(pixel[i], 0.0f, 1.0f);
        }

        write(pixel, i.s(), i.t());

    });

    return std::move(result);
}
