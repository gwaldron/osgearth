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
#include "ProceduralShaders"
#include "NoiseTextureFactory"

#include <osgEarth/TextureArena>
#include <osgEarth/Map>
#include <osgEarth/ElevationPool>
#include <osgEarth/Math>
#include <osgEarth/VirtualProgram>
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
    densityMaskLayer().set(conf, "density_mask_layer");
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

    biomeLayer().get(conf, "biomes_layer");
    densityMaskLayer().get(conf, "density_mask_layer");
    colorLayer().get(conf, "color_layer");
    landUseLayer().get(conf, "land_use_layer");
    conf.get("use_land_cover", useLandCover());
    conf.get("use_terrain", useTerrain());
}

//........................................................................

namespace
{
    // The four components of a LifeMap pixel
    constexpr unsigned DENSE = 0;
    constexpr unsigned LUSH = 1;
    constexpr unsigned RUGGED = 2;
    constexpr unsigned BIOME = 3;

    // noise channels
    constexpr unsigned SMOOTH = 0;
    constexpr unsigned RANDOM = 1;
    constexpr unsigned TURBULENT = 2;
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

    // OSG loader for reading an albedo texture and a heightmap texture
    // and merging them into a single RGBH texture
    struct RGBHPseudoLoader : public osgDB::ReaderWriter
    {
        RGBHPseudoLoader() {
            supportsExtension("oe_splat_rgbh", "RGB+H");
        }

        ReadResult readImage(const std::string& uri, const osgDB::Options* options) const override
        {
            std::string ext = osgDB::getLowerCaseFileExtension(uri);
            if (ext == "oe_splat_rgbh")
            {
                URI colorURI(
                    osgDB::getNameLessExtension(uri) + "_Color.jpg");

                osg::ref_ptr<osg::Image> color = colorURI.getImage(options);

                if (!color.valid())
                    return ReadResult::FILE_NOT_FOUND;

                URI heightURI(
                    osgDB::getNameLessExtension(uri) + "_Displacement.jpg");

                osg::ref_ptr<osg::Image> height = heightURI.getImage(options);

                osg::ref_ptr<osg::Image> rgbh = new osg::Image();
                rgbh->allocateImage(color->s(), color->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

                ImageUtils::PixelReader readHeight(height.get());
                ImageUtils::PixelReader readColor(color.get());
                ImageUtils::PixelWriter writeRGBH(rgbh.get());
                osg::Vec4 temp, temp2;
                float minh = 1.0f, maxh = 0.0f;

                ImageUtils::ImageIterator iter(rgbh.get());
                iter.forEachPixel([&]()
                    {
                        readColor(temp, iter.s(), iter.t());
                        if (height.valid())
                        {
                            readHeight(temp2, iter.s(), iter.t());
                            temp.a() = temp2.r();
                        }
                        else
                        {
                            temp.a() = 0.0f;
                        }

                        minh = osg::minimum(minh, temp.a());
                        maxh = osg::maximum(maxh, temp.a());
                        writeRGBH(temp, iter.s(), iter.t());
                    });

                OE_INFO << heightURI.base() << ", MinH=" << minh << ", MaxH=" << maxh << std::endl;

                ImageUtils::compressImageInPlace(rgbh.get());

                return rgbh;
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };


#define DEFAULT_ROUGHNESS 0.75
#define DEFAULT_AO 1.0

    // OSG load for reading normal, roughness, and AO textures and merging
    // them into a single encoded material texture.
    struct NNRAPseudoLoader : public osgDB::ReaderWriter
    {
        NNRAPseudoLoader() {
            supportsExtension("oe_splat_nnra", "Normal/Roughness/AO");
        }

        ReadResult readImage(const std::string& uri, const osgDB::Options* options) const override
        {
            std::string ext = osgDB::getLowerCaseFileExtension(uri);
            if (ext == "oe_splat_nnra")
            {
                URI normalsURI(
                    osgDB::getNameLessExtension(uri) + "_Normal.jpg");

                osg::ref_ptr<osg::Image> normals = 
                    normalsURI.getImage(options);

                if (!normals.valid())
                    return ReadResult::FILE_NOT_FOUND;

                URI roughnessURI(
                    osgDB::getNameLessExtension(uri) + "_Roughness.jpg");

                osg::ref_ptr<osg::Image> roughness =
                    roughnessURI.getImage(options);

                URI aoURI(
                    osgDB::getNameLessExtension(uri) + "_AmbientOcclusion.jpg");

                osg::ref_ptr<osg::Image> ao =
                    aoURI.getImage(options);

                osg::ref_ptr<osg::Image> nnra = new osg::Image();
                nnra->allocateImage(normals->s(), normals->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

                ImageUtils::PixelReader readNormals(normals.get());
                ImageUtils::PixelReader readAO(ao.get());
                ImageUtils::PixelReader readRoughness(roughness.get());
                ImageUtils::PixelWriter writeMat(nnra.get());

                osg::Vec3 normal3;
                osg::Vec4 normal;
                osg::Vec4 roughnessVal;
                osg::Vec4 aoVal;
                osg::Vec4 packed;

                ImageUtils::ImageIterator iter(nnra.get());
                iter.forEachPixel([&]() 
                    {
                        readNormals(normal, iter.s(), iter.t());
                        normal3.set(normal.x()*2.0 - 1.0, normal.y()*2.0 - 1.0, normal.z()*2.0 - 1.0);
                        NormalMapGenerator::pack(normal3, packed);
                        if (roughness.valid())
                        {
                            readRoughness(roughnessVal, iter.s(), iter.t());
                            packed[2] = roughnessVal.r();
                        }
                        else packed[2] = DEFAULT_ROUGHNESS;

                        if (ao.valid())
                        {
                            readAO(aoVal, iter.s(), iter.t());
                            packed[3] = aoVal.r();
                        }
                        else packed[3] = DEFAULT_AO;

                        writeMat(packed, iter.s(), iter.t());
                    });

                return nnra;
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };

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
                return false;

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
            return false;
        }
    };
}

REGISTER_OSGPLUGIN(oe_splat_rgbh, RGBHPseudoLoader);
REGISTER_OSGPLUGIN(oe_splat_nnra, NNRAPseudoLoader);

//........................................................................

void
LifeMapLayer::loadMaterials(const AssetCatalog* cat)
{
    for (const auto tex : cat->getTextures())
    {
        Texture* rgbh = new Texture();
        rgbh->_uri = URI(tex->uri()->full() + ".oe_splat_rgbh");
        _arena->add(rgbh);

        Texture* nnra = new Texture();
        nnra->_uri = URI(tex->uri()->full() + ".oe_splat_nnra");
        _arena->add(nnra);
    }
}


void
LifeMapLayer::init()
{
    ImageLayer::init();

    NoiseTextureFactory nf;
    _noiseFunc = nf.createImage(1024u, 4u);

    osg::StateSet* ss = this->getOrCreateStateSet();

    // Arena holds all the splatting textures
    _arena = new TextureArena();
    ss->setAttribute(_arena);

    // Install the texture splatting shader
    // TODO: consider moving this to another layer and making the LifeMapLayer
    // a non-visible data layer.
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    TerrainShaders shaders;
    shaders.load(vp, shaders.TextureSplatting2, getReadOptions());
}

Status
LifeMapLayer::openImplementation() 
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    options().densityMaskLayer().open(getReadOptions());

    options().colorLayer().open(getReadOptions());

    options().landUseLayer().open(getReadOptions());

    setProfile(Profile::create("global-geodetic"));
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
    options().densityMaskLayer().addedToMap(map);
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

    // bind the color layer if available
    if (getColorLayer() && getColorLayer()->isShared())
    {
        osg::StateSet* ss = getOrCreateStateSet();
        ss->setDefine("OE_COLOR_LAYER_TEX", getColorLayer()->getSharedTextureUniformName());
        ss->setDefine("OE_COLOR_LAYER_MAT", getColorLayer()->getSharedTextureMatrixUniformName());
    }

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
LifeMapLayer::prepareForRendering(TerrainEngine* engine)
{
    ImageLayer::prepareForRendering(engine);

    // Since we're actually rendering, load the materials for splatting
    if (getBiomeLayer())
    {
        const BiomeCatalog* cat = getBiomeLayer()->getBiomeCatalog();
        if (cat)
        {
            const AssetCatalog* assets = cat->getAssets();
            if (assets)
            {
                osg::ref_ptr<LifeMapLayer> layer(this);

                _loadMaterialsJob = Job<bool>::dispatch(
                    [layer, assets](Cancelable*) {
                        layer->loadMaterials(assets);
                        return true;
                    }
                );
            }
        }
    }
}

void
LifeMapLayer::removedFromMap(const Map* map)
{
    _map = nullptr;
    options().biomeLayer().removedFromMap(map);
    options().densityMaskLayer().removedFromMap(map);
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
LifeMapLayer::setDensityMaskLayer(ImageLayer* layer)
{
    options().densityMaskLayer().setLayer(layer);
}

ImageLayer*
LifeMapLayer::getDensityMaskLayer() const
{
    return options().densityMaskLayer().getLayer();
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

namespace
{
    inline float step(float edge, float x)
    {
        return x < edge ? 0.0f : 1.0f;
    }

    osg::Vec4f mix4(osg::Vec4f& a, osg::Vec4f& b, float m)
    {
        return osg::Vec4f(
            mix(a[0], b[0], m),
            mix(a[1], b[1], m),
            mix(a[2], b[2], m),
            mix(a[3], b[3], m));
    }

    void rgb2hsl(osg::Vec4f& c, osg::Vec4f& out)
    {
        osg::Vec4f K(0.0f, -1.0f / 3.0f, 2.0f / 3.0f, -1.0f);
        osg::Vec4f A(c.b(), c.g(), K.w(), K.z());
        osg::Vec4f B(c.g(), c.b(), K.x(), K.y());
        osg::Vec4f p = mix(A, B, step(c.b(), c.g()));
        A.set(p.x(), p.y(), p.w(), c.r());
        B.set(c.r(), p.y(), p.z(), p.x());
        osg::Vec4f q = mix(A, B, step(p.x(), c.r()));
        float d = q.x() - std::min(q.w(), q.y());
        const float e = 1.0e-10;
        out.set(
            fabs(q.z() + (q.w() - q.y()) / (6.0f*d + e)),
            d / (q.x() + e),
            q.x(),
            c.a());
    }

    //const vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    //vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    //vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
    //float d = q.x - min(q.w, q.y);
    //const float e = 1.0e-10;
    //return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);

    struct Value
    {
        Value() : weight(0.0f) { }
        float value;
        float weight;
    };

    struct Sample
    {
        Sample() : dense(), lush(), rugged(), weight(0.0f) { }
        Value dense;
        Value lush;
        Value rugged;
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
    // wait for all materials to load
    _loadMaterialsJob.wait();

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

    if (getLandUseLayer() && getBiomeLayer() && getBiomeLayer()->getBiomeCatalog())
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
    const LifeMapValueTable* landcover_table = getBiomeLayer()->getBiomeCatalog()->getLandCoverTable();

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
            }
        }
    }

    GeoImage densityMask;
    ImageUtils::PixelReader readDensityMask;
    osg::Vec4 dm_pixel;
    osg::Matrixf dm_matrix;

    // the mask layer zero's out density(etc)
    if (getDensityMaskLayer())
    {
        TileKey dm_key(key);

        while(dm_key.valid() && !densityMask.valid())
        {
            densityMask = getDensityMaskLayer()->createImage(dm_key, progress);
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

    if (getColorLayer())
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
    
    Sample sample[4];

    //float dense, lush, rugged;
    //float dense_mix, lush_mix, rugged_mix;
    //float default_mix;
    std::string lu_id;

    osg::Vec2 noiseCoords[4];
    osg::Vec4 noise[4];
    const unsigned noiseLOD[4] = { 0u, 9u, 13u, 16u };
    ImageUtils::PixelReader noiseSampler(_noiseFunc.get());
    noiseSampler.setBilinear(true);
    noiseSampler.setSampleAsRepeatingTexture(true);

    GeoImage result(image.get(), extent);

    //std::unordered_map<std::string, int> counts;

    GeoImageIterator  i(image.get(), extent);
    i.forEachPixelOnCenter([&]() {

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
        slope = harden(harden(1.0 - (normal * up)));

        // NOISE VALUES:
        float dense_noise =
            (0.3*noise[0][RANDOM]) +
            (0.3*noise[1][SMOOTH]) +
            (0.4*noise[2][CLUMPY]);

        float lush_noise =
            noise[1][SMOOTH];

        float rugged_noise =
            noise[1][CLUMPY];


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
            readLandCover(landcover_pixel, uu, vv);

            const LandCoverClass* lcc = _landCoverDictionary->getClassByValue(
                (int)landcover_pixel.r());

            if (lcc)
            {
                const LifeMapValue* value = landcover_table->getValue(lcc->getName());
                if (value)
                {
                    if (value->dense().isSet())
                    {
                        sample[LANDCOVER].dense.value = value->dense().get() + 0.2*(dense_noise*2.0 - 1.0);
                        sample[LANDCOVER].dense.weight = 1.0f;
                    }

                    if (value->lush().isSet())
                    {
                        sample[LANDCOVER].lush.value = value->lush().get() + 0.2*(lush_noise*2.0 - 1.0);
                        sample[LANDCOVER].lush.weight = 1.0f;
                    }

                    if (value->rugged().isSet())
                    {
                        sample[LANDCOVER].rugged.value = value->rugged().get() + 0.2*(rugged_noise*2.0 - 1.0);
                        sample[LANDCOVER].rugged.weight = 1.0f;
                    }

                    sample[LANDCOVER].weight = 1.0f;
                }
            }
        }

        // COLOR CONTRIBUTION:
        if (color.valid())
        {
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
            osg::Vec4f c(color_pixel.r(), color_pixel.g(), color_pixel.b(), 0.0f);
            c /= (c.r() + c.g() + c.b());

            // convert to HSL:
            osg::Vec4f hsl;
            rgb2hsl(c, hsl);

            constexpr float red = 0.0f;
            constexpr float green = 0.3333333f;
            constexpr float blue = 0.6666667f;
            constexpr float green_amp = 3.0f;

            float inv_green = fabs(hsl[0] - green);
            if (inv_green > 0.5f) inv_green = 1.0f - inv_green;
            float greenness = 1.0f - 2.0f*inv_green;
            greenness = pow(greenness, green_amp);

            float inv_red = fabs(hsl[0] - red);
            if (inv_red > 0.5f) inv_red = 1.0f - inv_red;
            float redness = 1.0f - 2.0*inv_red;

            sample[COLOR].dense.value = greenness;
            sample[COLOR].dense.weight = 1.0f;

            sample[COLOR].lush.value = clamp(1.2f * (hsl[1] - 0.5f*hsl[2]), 0.0f, 1.0f);
            sample[COLOR].lush.weight = 1.0f;

            sample[COLOR].rugged.value = (redness + (1.0f - hsl[1]) + hsl[2]) / 3.0f;
            sample[COLOR].rugged.value = clamp(sample[COLOR].rugged.value - 0.5, 0.0, 1.0) * 2.0;
            sample[COLOR].rugged.weight = 1.0f;

            sample[COLOR].weight = 1.0f;
        }

        // TERRAIN CONTRIBUTION:
        if (getUseTerrain())
        {
            sample[TERRAIN].weight = 1.0f;

            // only incorporate if we didn't use color:
            if (sample[COLOR].weight == 0.0f)
            {
                sample[TERRAIN].dense.value =
                    dense_noise - (0.8f*slope);
                sample[TERRAIN].dense.weight = 1.0f;

                sample[TERRAIN].lush.value =
                    (1.0f - lerpstep(250.0f, 3000.0f, elevation))
                    + (0.2f*lush_noise)
                    - (0.5f*slope);
                sample[TERRAIN].lush.weight = 1.0f;
            }

            sample[TERRAIN].rugged.value =
                //lerpstep(1600.0f, 5000.0f, elevation) +
                elevTile->getRuggedness(i.x(), i.y()) -
                (0.2f*rugged_noise);
            sample[TERRAIN].rugged.weight = 1.0f;
        }

        // CALCULATE WEIGHTED AVERAGES:
        pixel.set(0.0f, 0.0f, 0.0f, 1.0f);

        float
            dense_weight = 0.0f,
            lush_weight = 0.0f,
            rugged_weight = 0.0f;

        for (int i = 0; i < 4; ++i)
        {
            pixel[DENSE] += sample[i].dense.value*sample[i].dense.weight*sample[i].weight;
            dense_weight += sample[i].dense.weight*sample[i].weight;
            
            pixel[LUSH] += sample[i].lush.value*sample[i].lush.weight*sample[i].weight;
            lush_weight += sample[i].lush.weight*sample[i].weight;

            pixel[RUGGED] += sample[i].rugged.value*sample[i].rugged.weight*sample[i].weight;
            rugged_weight += sample[i].rugged.weight*sample[i].weight;
        }

        for (int i = 0; i < 4; ++i)
        {
            pixel[DENSE] = dense_weight > 0.0f ? clamp(pixel[DENSE] / dense_weight, 0.0f, 1.0f) : 0.0f;
            pixel[LUSH] = lush_weight > 0.0f ? clamp(pixel[LUSH] / lush_weight, 0.0f, 1.0f) : 0.0f;
            pixel[RUGGED] = rugged_weight > 0.0f ? clamp(pixel[RUGGED] / rugged_weight, 0.0f, 1.0f) : 0.0f;
        }

        // MASK CONTRIBUTION (applied to final data)
        if (densityMask.valid())
        {
            double uu = i.u() * dm_matrix(0, 0) + dm_matrix(3, 0);
            double vv = i.v() * dm_matrix(1, 1) + dm_matrix(3, 1);
            readDensityMask(dm_pixel, uu, vv);

            pixel[DENSE] *= dm_pixel.r();
            pixel[LUSH] *= dm_pixel.r();
            pixel[RUGGED] *= dm_pixel.r();
        }

        write(pixel, i.s(), i.t());

    });

    return std::move(result);
}
