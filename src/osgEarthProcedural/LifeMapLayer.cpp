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
    conf.set("land_cover_weight", landCoverWeight());
    conf.set("land_cover_blur", landCoverBlur());
    conf.set("terrain_weight", terrainWeight());
    conf.set("color_weight", colorWeight());
    conf.set("noise_weight", noiseWeight());
    conf.set("land_use_weight", landUseWeight());
    return conf;
}

void
LifeMapLayer::Options::fromConfig(const Config& conf)
{
    landCoverWeight().setDefault(1.0f);
    landCoverBlur().setDefault(Distance(20.0f, Units::METERS));
    terrainWeight().setDefault(1.0f);
    slopeIntensity().setDefault(1.0f);
    colorWeight().setDefault(1.0f);
    noiseWeight().setDefault(0.3f);
    landUseWeight().setDefault(1.0f);

    biomeLayer().get(conf, "biomes_layer");
    maskLayer().get(conf, "mask_layer");
    colorLayer().get(conf, "color_layer");
    landUseLayer().get(conf, "land_use_layer");
    conf.get("land_cover_weight", landCoverWeight());
    conf.get("land_cover_blur", landCoverBlur());
    conf.get("terrain_weight", terrainWeight());
    conf.get("color_weight", colorWeight());
    conf.get("noise_weight", noiseWeight());
    conf.get("land_use_weight", landUseWeight());
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
        noise *= 2.0;
        noise.r() -= 1.0, noise.g() -= 1.0, noise.b() -= 1.0, noise.a() -= 1.0;
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
LifeMapLayer::setLandCoverWeight(float value)
{
    options().landCoverWeight() = value;
}

float
LifeMapLayer::getLandCoverWeight() const
{
    return options().landCoverWeight().get();
}

bool
LifeMapLayer::getUseLandCover() const
{
    return getLandCoverWeight() > 0;
}

void
LifeMapLayer::setTerrainWeight(float value)
{
    options().terrainWeight() = value;
}

float
LifeMapLayer::getTerrainWeight() const
{
    return options().terrainWeight().get();
}

bool
LifeMapLayer::getUseTerrain() const
{
    return getTerrainWeight() > 0.0f;
}

void
LifeMapLayer::setColorWeight(float value)
{
    options().colorWeight() = value;
}

float
LifeMapLayer::getColorWeight() const
{
    return options().colorWeight().get();
}

bool
LifeMapLayer::getUseColor() const
{
    return getColorWeight() > 0.0f;
}

void
LifeMapLayer::setLandUseWeight(float value)
{
    options().landUseWeight() = value;
}

float
LifeMapLayer::getLandUseWeight() const
{
    return options().landUseWeight().get();
}

bool
LifeMapLayer::getUseLandUse() const
{
    return getLandUseWeight() > 0.0f;
}

void
LifeMapLayer::setNoiseWeight(float value)
{
    options().noiseWeight() = value;
}

float
LifeMapLayer::getNoiseWeight() const
{
    return options().noiseWeight().get();
}

bool
LifeMapLayer::getUseNoise() const
{
    return getNoiseWeight() > 0.0f;
}

#define NUM_INPUTS 5

#define NOISE 0
#define TERRAIN 1
#define LANDCOVER 2
#define LANDUSE 3
#define COLOR 4

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
    osg::Matrixf landcover_matrix;
    const LifeMapValueTable* landcover_table;
    std::unordered_map<std::string, int> specialTextureIndexLUT;

    // a "metaimage" lets us sample in the neighborhood of a tilekey image
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
        }
    }

    GeoImage densityMask;
    ImageUtils::PixelReader readDensityMask;
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
    osg::Matrixf color_matrix;

    if (getColorLayer() && getColorWeight() > 0.0f)
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
        GL_RGB, //GL_RGBA,
        GL_UNSIGNED_BYTE);

    ImageUtils::PixelWriter write(image.get());

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

    // size of the tile in meters:
    double width_m = key.getExtent().width(Units::METERS);
    double height_m = key.getExtent().height(Units::METERS);

    // land cover blurring values
    double lc_blur_m = std::max(0.0, options().landCoverBlur()->as(Units::METERS));
    double lc_blur_u = lc_blur_m / width_m;
    double lc_blur_v = lc_blur_m / height_m;

    GeoImage result(image.get(), extent);

    GeoImageIterator i(result);
    i.forEachPixelOnCenter([&]() {

        osg::Vec4f pixel[NUM_INPUTS];
        float weight[NUM_INPUTS] = { 0,0,0,0,0 };
        osg::Vec4f temp;

        // NOISE contribution
        if (getUseNoise())
        {
            for (int n = 0; n < 4; ++n)
            {
                noiseCoords[n].set(i.u(), i.v());
                scaleCoordsToRefLOD(noiseCoords[n], key, noiseLOD[n]);
                getNoise(noise[n], noiseSampler, noiseCoords[n]);
            }

            pixel[NOISE][LIFEMAP_DENSE] = noise[3][CLUMPY];
            pixel[NOISE][LIFEMAP_LUSH] = 0.0;
            pixel[NOISE][LIFEMAP_RUGGED] = noise[2][CLUMPY];

            weight[NOISE] = getNoiseWeight();
        }

        // LAND COVER CONTRIBUTION
        if (landcover.valid())
        {
            // scaled and biased UV coords
            double uu = i.u() * landcover_matrix(0, 0) + landcover_matrix(3, 0);
            double vv = i.v() * landcover_matrix(1, 1) + landcover_matrix(3, 1);

            // read the landcover with a blurring filter.
            int lc_kernel_count = 0;
            osg::Vec4f kernel;
            for (int a = -1; a <= 1; ++a)
            {
                for (int b = -1; b <= 1; ++b)
                {
                    landcover_metaimage.read(uu + (double)a*lc_blur_u, vv + (double)b*lc_blur_v, temp);
                    const LandCoverClass* lcc = _landCoverDictionary->getClassByValue((int)temp.r());
                    if (lcc)
                    {
                        const LifeMapValue* value = landcover_table->getValue(lcc->getName());
                        if (value)
                        {
                            lc_kernel_count++;
                            kernel[LIFEMAP_DENSE] += value->dense().get();
                            kernel[LIFEMAP_LUSH] += value->lush().get();
                            kernel[LIFEMAP_RUGGED] += value->rugged().get();
                        }
                    }
                }
            }
            if (lc_kernel_count > 0)
            {
                kernel /= lc_kernel_count;
                pixel[LANDCOVER] = kernel;
                weight[LANDCOVER] = getLandCoverWeight();
            }
            else
            {
                weight[LANDCOVER] = 0.0f;
            }
        }

        // COLOR CONTRIBUTION:
        if (color.valid())
        {
            double uu = i.u() * color_matrix(0, 0) + color_matrix(3, 0);
            double vv = i.v() * color_matrix(1, 1) + color_matrix(3, 1);
            readColor(temp, uu, vv);

            // convert to HSL:
            Color c(temp.r(), temp.g(), temp.b(), 0.0f);
            hsl = c.asHSL();

            constexpr float red = 0.0f;
            constexpr float green = 0.3333333f;
            constexpr float blue = 0.6666667f;

            // amplification factors for greenness and redness,
            // obtained empirically
            constexpr float green_amp = 2.0f;
            constexpr float red_amp = 5.0f;

            // Set lower limits for saturation and lightness, because
            // when these levels get too low, the HUE channel starts to
            // introduce math errors that can result in bad color values
            // that we do not want. (We determined these empirically
            // using an interactive shader.)
            constexpr float saturation_threshold = 0.2f;
            constexpr float lightness_threshold = 0.03f;

            // "Greenness" implies vegetation
            float dist_to_green = fabs(green - hsl[0]);
            if (dist_to_green > 0.5f)
                dist_to_green = 1.0f - dist_to_green;
            float greenness = 1.0f - 2.0f*dist_to_green;

            // "redness" implies ruggedness/rock
            float dist_to_red = fabs(red - hsl[0]);
            if (dist_to_red > 0.5f)
                dist_to_red = 1.0f - dist_to_red;
            float redness = 1.0f - 2.0f*dist_to_red;

            if (hsl[1] < saturation_threshold)
            {
                greenness *= hsl[1] / saturation_threshold;
                redness *= hsl[1] / saturation_threshold;
            }
            if (hsl[2] < lightness_threshold)
            {
                greenness *= hsl[2] / lightness_threshold;
                redness *= hsl[2] / lightness_threshold;
            }

            greenness = pow(greenness, green_amp);
            redness = pow(redness, red_amp);

            float w = options().colorWeight().get();

            pixel[COLOR][LIFEMAP_DENSE] = greenness;
            pixel[COLOR][LIFEMAP_LUSH] = greenness * (1.0 - hsl.z()); // lighter green is less lush.
            pixel[COLOR][LIFEMAP_RUGGED] = redness;

            // if the lightness value is too high, it's white, which is usually
            // snow or clouds, and we can't use it for anything meaningful
            if (pow(hsl[2], 5.0f) > 0.5f)
                weight[COLOR] = 0.0f;
            else
                weight[COLOR] = getColorWeight(); // * max(greeness, redness) ...???
        }

        // TERRAIN CONTRIBUTION:
        if (getUseTerrain())
        {
            // Establish elevation at this pixel:
            elevation = elevTile->getElevation(i.x(), i.y()).elevation().as(Units::METERS);

            // Normal map at this pixel:
            normal = elevTile->getNormal(i.x(), i.y());

            // exaggerate the slope value
            slope = 1.0 - (normal*up);
            float r = decel(slope * options().slopeIntensity().get());
            pixel[TERRAIN][LIFEMAP_RUGGED] = r;
            pixel[TERRAIN][LIFEMAP_DENSE] = -r;
            pixel[TERRAIN][LIFEMAP_LUSH] = -r;

            weight[TERRAIN] = getTerrainWeight();
        }

        // LAND USE CONTRIBUTION:
        if (!landuse.empty())
        {
            double xx = i.x(); // +x_jitter * 0.3*(noise[2][CLUMPY] * 2.0 - 1.0);
            double yy = i.y(); // +y_jitter * 0.3*(noise[2][SMOOTH] * 2.0 - 1.0);

            const LifeMapValue* lu = landuse.get(xx, yy, landuse_table);
            if (lu)
            {
                pixel[LANDUSE][LIFEMAP_DENSE] = lu->dense().get();
                pixel[LANDUSE][LIFEMAP_LUSH] = lu->lush().get();
                pixel[LANDUSE][LIFEMAP_RUGGED] = lu->rugged().get();
                weight[LANDUSE] = getLandUseWeight();
            }
        }

        // CONBINE WITH WEIGHTS:
        osg::Vec4f combined_pixel;

        // first, combine landcover and color by relative weight.
        float w2 = weight[LANDCOVER] + weight[COLOR];
        if (w2 > 0.0f)
        {
            combined_pixel =
                pixel[LANDCOVER] * weight[LANDCOVER]/w2 +
                pixel[COLOR] * weight[COLOR]/w2;
        }

        // apply terrain additively:
        combined_pixel += pixel[TERRAIN] * weight[TERRAIN];

        // apply the noise additively:
        combined_pixel += pixel[NOISE] * weight[NOISE];

        // MASK CONTRIBUTION (applied to final combined pixel data)
        if (densityMask.valid())
        {
            double uu = clamp(i.u() * dm_matrix(0, 0) + dm_matrix(3, 0), 0.0, 1.0);
            double vv = clamp(i.v() * dm_matrix(1, 1) + dm_matrix(3, 1), 0.0, 1.0);
            readDensityMask(temp, uu, vv);
            
            combined_pixel[LIFEMAP_DENSE] *= temp.r();
            combined_pixel[LIFEMAP_LUSH] *= temp.r();
            combined_pixel[LIFEMAP_RUGGED] *= temp.r();
        }

        // Clamp everything to [0..1] and write it out.
        for (int i = 0; i < 4; ++i)
        {
            combined_pixel[i] = clamp(combined_pixel[i], 0.0f, 1.0f);
        }

        write(combined_pixel, i.s(), i.t());

    });

    return std::move(result);
}
