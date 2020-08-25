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

#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>

#define LC "[LifeMapLayer] " << getName() << ": "

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(lifemap, LifeMapLayer);

template<typename T>
class ControlVectors
{
public:
    struct Point {
        double x, y;
        T data;
    };
    struct Result {
        const T* data;
        float weight;
        bool operator < (const Result& rhs) const {
            return weight < rhs.weight;
        }
    };
    typedef std::set<Result> ResultSet;

    std::vector<Point> _index;

    void add(double x, double y, const T& data)
    {
        _index.emplace_back(Point());
        _index.back().x = x;
        _index.back().y = y;
        _index.back().data = data;
    }

    void lookup(double x, double y, ResultSet& results) const
    {
        results.clear();

        //TODO: replace this brute-force search with a proper
        // spatial index.
        const int maxCount = 3;
        double farthest2 = 0.0;
        std::set<Result>::iterator farthest2_iter;

        for (const auto& cp : _index)
        {
            double range2 = (cp.x - x)*(cp.x - x) + (cp.y - y)*(cp.y - y);

            if (results.size() < maxCount)
            {
                Result r;
                r.data = &cp.data;
                r.weight = range2;
                results.insert(r);
            }
            else
            {
                if (range2 < results.rbegin()->weight)
                {
                    Result r;
                    r.data = &cp.data;
                    r.weight = range2;
                    results.insert(r);
                    results.erase(std::prev(results.end()));
                }
            }
        }
    }
};

//........................................................................

LifeMapLayer::LandCoverLifeMapping::LandCoverLifeMapping(const Config& conf)
{
    conf.get("class", className());
    conf.get("density", density());
    conf.get("moisture", moisture());
    conf.get("rugged", rugged());
    conf.get("temperature", temperature());
}

Config
LifeMapLayer::LandCoverLifeMapping::getConfig() const
{
    Config conf("mapping");
    conf.set("class", className());
    conf.set("density", density());
    conf.set("moisture", moisture());
    conf.set("rugged", rugged());
    conf.set("temperature", temperature());
    return conf;
}

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
    landCoverLayer().set(conf, "landcover_layer");
    
    Config mappings_conf;
    for (const auto& mapping : landCoverMappings())
        mappings_conf.add(mapping.getConfig());
    if (!mappings_conf.empty()) conf.add(mappings_conf);

    return conf;
}

void
LifeMapLayer::Options::fromConfig(const Config& conf)
{
    biomeLayer().get(conf, "biomes_layer");
    landCoverLayer().get(conf, "landcover_layer");

    const ConfigSet mappings_conf = conf.child("landcover_mappings").children("mapping");
    for (const auto& c : mappings_conf)
        landCoverMappings().emplace_back(c);
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
    options().landCoverLayer().addedToMap(map);
    options().landCoverDictionary().addedToMap(map);

    // not specified; try to find it
    if (!getBiomeLayer())
        setBiomeLayer(map->getLayer<BiomeLayer>());

    if (!getLandCoverDictionary())
        setLandCoverDictionary(map->getLayer<LandCoverDictionary>());
    
    Status biomeStatus = options().biomeLayer().open(getReadOptions());
    if (biomeStatus.isOK())
    {
        OE_INFO << LC << "Using biome layer \"" << getBiomeLayer()->getName() << "\"" << std::endl;

        if (getBiomeLayer())
        {
            const BiomeCatalog* cat = getBiomeLayer()->getBiomeCatalog();
            if (cat)
            {
                const AssetCatalog* assets = cat->getAssets();
                if (assets)
                {
                    _loadMaterialsJob = std::async(
                        &LifeMapLayer::loadMaterials,
                        this,
                        assets);
                }
            }
        }
#if 0
        const Biome* biome = getBiomeLayer()->getBiomeCatalog()->getAssets();
        if (biome)
        {
            for (const auto tex : biome->textures())
            {
                loadMaterials(tex->uri()->full());
            }
        }
#endif
    }

    _map = map;

    // make the land cover code LUT
    if (getLandCoverDictionary())
    {
        for (const auto& mapping : options().landCoverMappings())
        {
            const LandCoverClass* lcc = getLandCoverDictionary()->getClassByName(mapping.className().get());
            if (lcc)
                _lcLUT[lcc->getValue()] = &mapping;
        }
    }
}

void
LifeMapLayer::removedFromMap(const Map* map)
{
    _map = nullptr;
    options().biomeLayer().removedFromMap(map);
    options().landCoverLayer().removedFromMap(map);
    options().landCoverDictionary().removedFromMap(map);
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
LifeMapLayer::setLandCoverLayer(LandCoverLayer* layer)
{
    options().landCoverLayer().setLayer(layer);
}

LandCoverLayer*
LifeMapLayer::getLandCoverLayer() const
{
    return options().landCoverLayer().getLayer();
}

void
LifeMapLayer::setLandCoverDictionary(LandCoverDictionary* layer)
{
    options().landCoverDictionary().setLayer(layer);
}

LandCoverDictionary*
LifeMapLayer::getLandCoverDictionary() const
{
    return options().landCoverDictionary().getLayer();
}

GeoImage
LifeMapLayer::createImageImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    // sync to the async material loader
    if (_loadMaterialsJob.valid())
        _loadMaterialsJob.wait();

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

    // if we have landcover data, fetch that now
    GeoImage landcover;

#if 1
    if (getLandCoverLayer())
    {
        TileKey lckey = key;
        for (TileKey lckey=key; !landcover.valid() && lckey.valid(); lckey.makeParent())
        {
            landcover = getLandCoverLayer()->createImage(lckey);
        }
    }
    GeoImageReader lc_reader(landcover);
#endif

    // assemble the image:
    ImageUtils::PixelWriter write(image.get());
    osg::Vec4 pixel, temp, lcpixel;
    ControlVectors<osg::Vec4>::ResultSet lc_results;
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

            // exaggerate the slope value
            slope = harden(harden(1.0 - (normal * up)));

#if 1

            pixel[RUGGED] =
                //lerpstep(1600.0f, 5000.0f, elevation) +
                elevTile->getRuggedness(x, y);

            // Randomize it
            pixel[DENSITY] =
                (0.3*noise[0][RANDOM]) +
                (0.3*noise[1][SMOOTH]) +
                (0.4*noise[2][CLUMPY]);

            // Compensate for low density
            pixel[DENSITY] *= 1.1;

            // Discourage density in highly sloped areas
            pixel[DENSITY] -= (0.8*slope);

            // moisture is fairly arbitrary
            float moisture = 1.0f - lerpstep(250.0f, 3000.0f, elevation);
            moisture += (0.2 * noise[1][SMOOTH]);
            moisture -= 0.5*slope;

            // temperature decreases with altitude
            float temperature = 1.0f - lerpstep(0.0f, 4000.0f, elevation);
            temperature += (0.2 * noise[1][SMOOTH]);

            // encode moisture and temperature together (4 bits each)
            pixel[MOISTURE] = ((float)(((int)(moisture*16.0f) << 4) | ((int)(temperature*16.0f)))) / 255.0f;

            // biome lookup
            float biomeNoise = fract(
                noise[1][RANDOM] +
                noise[2][SMOOTH]);

            pixel[BIOME] = (float)lookupBiome(x, y, biomeNoise) / 255.0f;

            // clamp to legal range (not the biome though)
            for (int i = 0; i < 3; ++i)
            {
                pixel[i] = clamp(pixel[i], 0.0f, 1.0f);
            }

#else

            if (landcover.valid())
            {
                lc_reader.read(temp, x, y);  
                const LandCoverLifeMapping* m = lookupLandCoverLifeMapping((int)(temp.r()));
                if (m)
                {
                    pixel[RUGGED] = m->rugged().get();
                    pixel[DENSITY] = m->density().get();
                    pixel[MOISTURE] = m->moisture().get();
                }
            }

            //pixel[RUGGED] = mix(
            //    pixel[RUGGED],
            //    elevTile->getRuggedness(x, y),
            //    noise[2][CLUMPY]);

            pixel[RUGGED] *=
                (0.5f) +
                (0.3f*noise[0][RANDOM]) +
                (0.3f*noise[1][SMOOTH]) +
                (0.4f*noise[2][CLUMPY]);

            // Randomize it
            pixel[DENSITY] *=
                (0.5f) +
                (0.3f*noise[0][RANDOM]) +
                (0.3f*noise[1][SMOOTH]) +
                (0.4f*noise[2][CLUMPY]);

            // Compensate for low density
            //pixel[DENSITY] *= 1.4;

            // Discourage density in highly sloped areas
            pixel[DENSITY] -= (0.8f*slope);

            // moisture is fairly arbitrary
            float moisture = 1.0f - lerpstep(250.0f, 3000.0f, elevation);
            moisture += (0.2f * noise[1][SMOOTH]);
            moisture -= 0.5f*slope;

            // temperature decreases with altitude
            float temperature = 1.0f - lerpstep(0.0f, 4000.0f, elevation);
            temperature += (0.2f * noise[1][SMOOTH]);

            // encode moisture and temperature together (4 bits each)
            pixel[MOISTURE] = ((float)(((int)(moisture*16.0f) << 4) | ((int)(temperature*16.0f)))) / 255.0f;

            // biome lookup
            float biomeNoise = fract(
                noise[1][RANDOM] +
                noise[2][SMOOTH]);

            pixel[BIOME] = (float)lookupBiome(x, y, biomeNoise) / 255.0f;

            // clamp to legal range (not the biome though)
            for (int i = 0; i < 3; ++i)
            {
                pixel[i] = clamp(pixel[i], 0.0f, 1.0f);
            }
#endif



            write(pixel, s, t);
        }
    }

    return GeoImage(image.get(), key.getExtent());
}

osg::Vec4
LifeMapLayer::lookupLandCover(float noise, void* thing) const
{
    ControlVectors<osg::Vec4>::ResultSet* r = static_cast<
        ControlVectors<osg::Vec4>::ResultSet*>(thing);

    const double strength = 9.0;
    double total = 0.0;
    for (auto& i : *r)
        total += pow(1.0 / i.weight, strength);

    double runningTotal = 0.0;
    double pick = total * (double)noise;
    for (auto& i : *r)
    {
        runningTotal += pow(1.0 / i.weight, strength);
        if (pick < runningTotal)
            return *i.data;
    }

    // return the last one by default
    return *r->rbegin()->data;
}

int
LifeMapLayer::lookupBiome(double x, double y, float noise) const
{    
    BiomeLayer* layer = getBiomeLayer();
    if (layer)
    {
        std::set<BiomeLayer::SearchResult> results;
        layer->getNearestBiomes(x, y, 4, results);
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

const LifeMapLayer::LandCoverLifeMapping*
LifeMapLayer::lookupLandCoverLifeMapping(int code) const
{
    const auto i = _lcLUT.find(code);
    return i != _lcLUT.end() ? i->second : nullptr;
}
