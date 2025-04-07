/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "LifeMapLayer"
#include "SplatShaders"
#include "NoiseTextureFactory"

#include <osgEarth/TextureArena>
#include <osgEarth/Map>
#include <osgEarth/ElevationPool>
#include <osgEarth/Math>

#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>

#define LC "[LifeMapLayer] " << getName() << ": "

using namespace osgEarth;
using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(lifemap, LifeMapLayer);

//........................................................................

Config
LifeMapLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    biomes().set(conf, "biomes");
    return conf;
}

void
LifeMapLayer::Options::fromConfig(const Config& conf)
{
    biomes().get(conf, "biomes");
}

//........................................................................

namespace
{
    // The four components of a LifeMap pixel
    constexpr unsigned DENSITY = 0;
    constexpr unsigned MOISTURE = 1;
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
        noise *= 2.0;
        noise.r() -= 1.0, noise.g() -= 1.0, noise.b() -= 1.0, noise.a() -= 1.0;
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

                for(int t=0; t< writeRGBH.t(); ++t)
                {
                    for(int s=0; s< writeRGBH.s(); ++s)
                    {
                        readColor(temp, s, t);
                        if (height.valid())
                        {
                            readHeight(temp2, s, t);
                            temp.a() = temp2.r();
                        }
                        else
                        {
                            temp.a() = 0.0f;
                        }

                        minh = osg::minimum(minh, temp.a());
                        maxh = osg::maximum(maxh, temp.a());
                        writeRGBH(temp, s, t);
                    }
                }
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

                for(int t=0; t<writeMat.t(); ++t)
                {
                    for(int s=0; s< writeMat.s(); ++s)
                    {
                        readNormals(normal, s, t);
                        normal3.set(normal.x()*2.0-1.0, normal.y()*2.0-1.0, normal.z()*2.0-1.0);
                        NormalMapGenerator::pack(normal3, packed);
                        if (roughness.valid())
                        {
                            readRoughness(roughnessVal, s, t);
                            packed[2] = roughnessVal.r();
                        }
                        else packed[2] = DEFAULT_ROUGHNESS;

                        if (ao.valid())
                        {
                            readAO(aoVal, s, t);
                            packed[3] = aoVal.r();
                        }
                        else packed[3] = DEFAULT_AO;

                        writeMat(packed, s, t);
                    }
                }
                return nnra;
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };
}

REGISTER_OSGPLUGIN(oe_splat_rgbh, RGBHPseudoLoader);
REGISTER_OSGPLUGIN(oe_splat_nnra, NNRAPseudoLoader);

//........................................................................

void
LifeMapLayer::loadMaterials(const std::string& base)
{
    Texture* rgbh = new Texture();
    rgbh->_uri = URI(base + ".oe_splat_rgbh");
    _arena->add(rgbh);

    Texture* nnra = new Texture();
    nnra->_uri = URI(base + ".oe_splat_nnra");
    _arena->add(nnra);
}

void
LifeMapLayer::init()
{
    ImageLayer::init();

    NoiseTextureFactory nf;
    _noiseFunc = nf.createImage(1024u, 4u);

    // Arena holds all the splatting textures
    _arena = new TextureArena();
    osg::StateSet* ss = this->getOrCreateStateSet();
    ss->setAttribute(_arena);

    loadMaterials("D:/data/splat/Rock030_2K");
    loadMaterials("D:/data/splat/Ground029_2K");
    loadMaterials("D:/data/splat/Ground037_2K");
    loadMaterials("D:/data/splat/Ground003_2K");
}

Status
LifeMapLayer::openImplementation() 
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

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
    options().biomes().addedToMap(map);
    
    Status biomeStatus = options().biomes().open(getReadOptions());
    if (biomeStatus.isOK())
    {
        OE_INFO << LC << "Using biome layer \"" << getBiomeLayer()->getName() << std::endl;
    }

    _map = map;
}

void
LifeMapLayer::removedFromMap(const Map* map)
{
    _map = nullptr;
    options().biomes().close();
    options().biomes().removedFromMap(map);
    ImageLayer::removedFromMap(map);
}

BiomeLayer*
LifeMapLayer::getBiomeLayer() const
{
    return options().biomes().getLayer();
}

GeoImage
LifeMapLayer::createImageImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(
        getTileSize(),
        getTileSize(),
        1,
        GL_RGBA,
        GL_UNSIGNED_BYTE);

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

    GeoExtent extent = key.getExtent();

    // assemble the image:
    ImageUtils::PixelWriter write(image.get());
    osg::Vec4 pixel, temp;
    ElevationSample sample;
    float elevation;
    osg::Vec3 normal;
    float slope;
    const osg::Vec3 up(0,0,1);

    osg::Vec2 noiseCoords[4];
    osg::Vec4 noise[4];
    const unsigned noiseLOD[4] = { 0u, 9u, 13u, 16u };
    ImageUtils::PixelReader noiseSampler(_noiseFunc.get());
    noiseSampler.setBilinear(true);
    noiseSampler.setSampleAsRepeatingTexture(true);

    for(int t=0; t<write.t(); ++t)
    {
        double v = (double)t/(double)(write.t()-1);
        double y = extent.yMin() + extent.height()*v;

        for(int s=0; s<write.s(); ++s)
        {
            double u = (double)s/(double)(write.s()-1);
            double x = extent.xMin() + extent.width()*u;

            for(int n=0; n<4; ++n)
            {
                //TODO: well?
                if (true)//key.getLOD() >= noiseLOD[n])
                {
                    noiseCoords[n].set(u, v);
                    scaleCoordsToRefLOD(noiseCoords[n], key, noiseLOD[n]);
                    getNoise(noise[n], noiseSampler, noiseCoords[n]);
                }
            }

            elevation = elevTile->getElevation(x, y).elevation().as(Units::METERS);

            normal = elevTile->getNormal(x, y);
            slope = harden(harden(1.0 - (normal * up)));


            // roughess increases with altitude:
            pixel[RUGGED] = lerpstep(1600.0f, 5000.0f, elevation);
            // and increases with slope:
            pixel[RUGGED] += slope;

            // Ramp life down slowly as we increase in altitude.
            pixel[DENSITY] = 1.0f - osg::clampBetween(
                elevation / 6500.0f,
                0.0f, 1.0f);

            // Randomize it
            pixel[DENSITY] +=
                (0.3*noise[0][RANDOM]) +
                (0.5*noise[1][CLUMPY]) +
                (0.4*noise[2][SMOOTH]);

            // Discourage density in rugged areas
            pixel[DENSITY] -= 2.0*pixel[RUGGED];

            // moisture is fairly arbitrary
            float moisture = 1.0f - lerpstep(250.0f, 3000.0f, elevation);
            moisture += (0.2 * noise[1][SMOOTH]);
            moisture -= 0.5*slope;

            float temperature = 1.0f - lerpstep(0.0f, 4000.0f, elevation);
            temperature += (0.2 * noise[1][SMOOTH]);

            pixel[MOISTURE] = ((float)(((int)(moisture*16.0f) << 4) | ((int)(temperature*16.0f)))) / 255.0f;

            float biomeNoise = fract(
                noise[1][RANDOM]);

            pixel[BIOME] = (float)lookupBiome(x, y, biomeNoise) / 255.0f;

            // saturate:
            for(int i=0; i<4; ++i)
                pixel[i] = clamp(pixel[i], 0.0f, 1.0f);

            write(pixel, s, t);
        }
    }

    return GeoImage(image.get(), key.getExtent());
}

int
LifeMapLayer::lookupBiome(double x, double y, float noise) const
{    
    BiomeLayer* layer = getBiomeLayer();
    if (layer)
    {
        std::set<BiomeLayer::SearchResult> results;
        layer->getNearestBiomes(x, y, 3, results);
        if (results.empty())
        {
            return 0;
        }
        else
        {
            const double strength = 9.0;
            double total = 0.0;
            for (auto& i : results)
                total += pow(1.0 / i.range2, strength);

            double runningTotal = 0.0;
            double pick = total * (double)noise;
            for (auto& i : results)
            {
                runningTotal += pow(1.0 / i.range2, strength);
                if (pick < runningTotal)
                    return i.biomeid;
            }

            // return the last one by default
            return results.rbegin()->biomeid;
        }
    }
    return 0;
}
