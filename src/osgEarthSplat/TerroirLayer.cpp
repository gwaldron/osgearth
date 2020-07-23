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
#include "TerroirLayer"
#include "SplatShaders"
#include "NoiseTextureFactory"
#include <osgEarth/TextureArena>
#include <osgEarth/Map>
#include <osgEarth/ElevationPool>

#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>

#define LC "[TerroirLayer] " << getName() << ": "

using namespace osgEarth;
using namespace osgEarth::Splat;

REGISTER_OSGEARTH_LAYER(terroir, TerroirLayer);

//........................................................................

Config
TerroirLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    return conf;
}

void
TerroirLayer::Options::fromConfig(const Config& conf)
{
}

//........................................................................

namespace
{
    // The four components of a terroir pixel
    constexpr unsigned LIFE = 0;
    constexpr unsigned MOISTURE = 1;
    constexpr unsigned ROUGHNESS = 2;
    constexpr unsigned BIOME = 3;

    // noise channels
    constexpr unsigned SMOOTH = 0;
    constexpr unsigned RANDOM1 = 1;
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
        const osg::Vec4 bias(1,1,1,1);
        read(noise, coords.x(), coords.y());
        noise *= 2.0;
        noise -= bias;
    }

    float unitmap(float x, float lo, float hi)
    {
        if (x <= lo) return 0.0f;
        else if (x >= hi) return 1.0f;
        else return (x-lo)/(hi-lo);
    }

    float remap(float x, float lo, float hi)
    {
        return lo+x*(hi-lo);
    }

    float decel(float x)
    {
        return 1.0-(1.0-x)*(1.0-x);
    }

    float accel(float x)
    {
        return x*x;
    }

    float threshold(float x, float thresh, float buf)
    {
        if (x < thresh-buf) return 0.0f;
        else if (x > thresh+buf) return 1.0f;
        else return osg::clampBetween( (x-(thresh-buf)) / (buf*2.0f), 0.0f, 1.0f);
    }

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

                ImageUtils::PixelReader readHeight(height.get());
                ImageUtils::PixelReader readColor(color.get());
                ImageUtils::PixelWriter writeColor(color.get());
                osg::Vec4 temp, temp2;

                for(int t=0; t<writeColor.t(); ++t)
                {
                    for(int s=0; s<writeColor.s(); ++s)
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
                        writeColor(temp, s, t);
                    }
                }

                ImageUtils::compressImageInPlace(color.get());

                return color;
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };



    struct NNSAPseudoLoader : public osgDB::ReaderWriter
    {
        NNSAPseudoLoader() {
            supportsExtension("oe_splat_nnsa", "Normal/Smooth/AO");
        }

        ReadResult readImage(const std::string& uri, const osgDB::Options* options) const override
        {
            std::string ext = osgDB::getLowerCaseFileExtension(uri);
            if (ext == "oe_splat_nnsa")
            {
                URI normalsURI(
                    osgDB::getNameLessExtension(uri) + "_Normal.jpg");

                osg::ref_ptr<osg::Image> normals = 
                    normalsURI.getImage(options);

                if (!normals.valid())
                    return ReadResult::FILE_NOT_FOUND;

                //TODO: smoothness, AO

                osg::ref_ptr<osg::Image> nnsa = new osg::Image();
                nnsa->allocateImage(normals->s(), normals->t(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

                ImageUtils::PixelReader readNormals(normals.get());
                ImageUtils::PixelWriter writeNNSA(nnsa.get());

                osg::Vec3 temp3;
                osg::Vec4 temp;
                osg::Vec4 packed;

                for(int t=0; t<writeNNSA.t(); ++t)
                {
                    for(int s=0; s<writeNNSA.s(); ++s)
                    {
                        readNormals(temp, s, t);
                        temp3.set(temp.x()*2.0-1.0, temp.y()*2.0-1.0, temp.z()*2.0-1.0);
                        NormalMapGenerator::pack(temp3, packed);
                        writeNNSA(packed, s, t);
                    }
                }
                return nnsa;
            }

            return ReadResult::FILE_NOT_HANDLED;
        }
    };
}

REGISTER_OSGPLUGIN(oe_splat_rgbh, RGBHPseudoLoader);
REGISTER_OSGPLUGIN(oe_splat_nnsa, NNSAPseudoLoader);

//........................................................................

void
TerroirLayer::loadMaterials(const std::string& base)
{
    Texture* rgbh = new Texture();
    rgbh->_uri = URI(base + ".oe_splat_rgbh");
    _arena->add(rgbh);

    Texture* nnsa = new Texture();
    nnsa->_uri = URI(base + ".oe_splat_nnsa");
    _arena->add(nnsa);
}

void
TerroirLayer::init()
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

    //loadMaterials("D:/data/splat/Ground033_2K");

    loadMaterials("D:/data/splat/grass_seamless");
}

Status
TerroirLayer::openImplementation() 
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    setProfile(Profile::create("global-geodetic"));
    return Status::OK();
}

Status
TerroirLayer::closeImplementation()
{
    return ImageLayer::closeImplementation();
}

void
TerroirLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);
    _map = map;
}

void
TerroirLayer::removedFromMap(const Map* map)
{
    _map = nullptr;
    ImageLayer::removedFromMap(map);
}

GeoImage
TerroirLayer::createImageImplementation(
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
    const unsigned noiseLOD[4] = { 0u, 9u, 13u, 17u };
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
                if (key.getLOD() >= noiseLOD[n])
                {
                    noiseCoords[n].set(u, v);
                    scaleCoordsToRefLOD(noiseCoords[n], key, noiseLOD[n]);
                    getNoise(noise[n], noiseSampler, noiseCoords[n]);
                }
            }

            elevation = elevTile->getElevation(x, y).elevation().as(Units::METERS);

            normal = elevTile->getNormal(x, y);
            slope = decel(decel(1.0 - (normal * up)));

            // Ramp life down slowly as we increase in altitude.
            pixel[LIFE] = 1.0f - osg::clampBetween(
                elevation / 6500.0f,
                0.0f, 1.0f);

            // Randomize it
            pixel[LIFE] += 
                (0.3*noise[0][RANDOM1]) +
                (0.5*noise[1][CLUMPY]) +
                (0.3*noise[2][SMOOTH]) +
                (0.4*decel(noise[3][CLUMPY]));

            // Discourage life on slopes.
            pixel[LIFE] -= slope;

            // Clamps life variance to a small range...but why?
            //pixel[LIFE] = threshold(pixel[LIFE], 0.2f, 0.1f);


            // moisture is fairly arbitrary, but decreases a bit with slope.
            pixel[MOISTURE] = 0.75f;
            pixel[MOISTURE] +=
                (0.3*noise[0][RANDOM2]) +
                (0.2*noise[1][SMOOTH]) +
                (0.5*noise[2][CLUMPY]) +
                (0.8*decel(noise[3][RANDOM1]));
            pixel[MOISTURE] -= slope;

            // roughess increases with altitude:
            pixel[ROUGHNESS] = unitmap(elevation, 1600.0f, 5000.0f);
            // and increases with slope:
            pixel[ROUGHNESS] += slope;


            pixel[BIOME] = 
                remap(elevation, -100.0f, -1.0f);

            // saturate:
            for(int i=0; i<4; ++i)
                pixel[i] = osg::clampBetween(pixel[i], 0.0f, 1.0f);

            write(pixel, s, t);
        }
    }

    return GeoImage(image.get(), key.getExtent());
}
