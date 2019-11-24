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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/LandCoverLayer>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/SimplexNoise>
#include <osgEarth/Progress>
#include <osgEarth/Random>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[LandCoverLayer] "

REGISTER_OSGEARTH_LAYER(landcover, LandCoverLayer);

namespace
{
    void check(int s, int t, int supper, int tupper) {
        if (s<0 || s>supper-1 || t<0 ||t>tupper-1)
        {
            OE_WARN << "BREAK! OUT OF BOUNDS!" << std::endl;
        }
    }

    osg::Vec2 getSplatCoords(const TileKey& key, float baseLOD, const osg::Vec2& covUV)
    {
        osg::Vec2 out;

        float dL = (float)key.getLOD() - baseLOD;
        float factor = pow(2.0f, dL);
        float invFactor = 1.0/factor;
        out.set( covUV.x()*invFactor, covUV.y()*invFactor ); 

        // For upsampling we need to calculate an offset as well
        if ( factor >= 1.0 )
        {
            unsigned wide, high;
            key.getProfile()->getNumTiles(key.getLOD(), wide, high);

            float tileX = (float)key.getTileX();
            float tileY = (float)(wide-1-key.getTileY()); // swap Y. (not done in the shader version.)

            osg::Vec2 a( floor(tileX*invFactor), floor(tileY*invFactor) );
            osg::Vec2 b( a.x()*factor, a.y()*factor );
            osg::Vec2 c( (a.x()+1.0f)*factor, (a.y()+1.0f)*factor );
            osg::Vec2 offset( (tileX-b.x())/(c.x()-b.x()), (tileY-b.y())/(c.y()-b.y()) );

            out += offset;
        }

        return out;
    }

    osg::Vec2 warpCoverageCoords(const osg::Vec2& covIn, float noise, float warp)
    {
        float n1 = 2.0 * noise - 1.0;
        return osg::Vec2(
            covIn.x() + sin(n1*osg::PI*2.0) * warp,
            covIn.y() + sin(n1*osg::PI*2.0) * warp);
    }

    float getNoise(SimplexNoise& noiseGen, const osg::Vec2& uv)
    {
        // TODO: check that u and v are 0..s and not 0..s-1
        double n = noiseGen.getTiledValue(uv.x(), uv.y());
        n = osg::clampBetween(n, 0.0, 1.0);
        return n;
    }

    struct ILayer 
    {
        GeoImage  image;
        float     scale;
        osg::Vec2 bias;
        bool      valid;
        float     warp;
        ImageUtils::PixelReader* read;
        unsigned* codeTable;
        unsigned loads;

        ILayer() : valid(true), read(0L), scale(1.0f), warp(0.0f), loads(0) { }

        ~ILayer() { if (read) delete read; }

        void load(const TileKey& key, LandCoverCoverageLayer* sourceLayer, ProgressCallback* progress)
        {
            if (sourceLayer->getEnabled())
            {
                ImageLayer* imageLayer = sourceLayer->getImageLayer();

                for(TileKey k = key; k.valid() && !image.valid() && imageLayer->isKeyInLegalRange(k); k = k.createParentKey())
                {
                    // Check is there is the possibility of data for this key
                    // If not, proceed to the parent (since we are potentially mosaicing)
                    // We may want to check isKeyInLegalRange here as well, since putting it
                    // in the for loop will only work for min_level (but that is probably OK)
                    if (imageLayer->mayHaveData(key))
                    {
                        image = imageLayer->createImage(k, progress);
                    }

                    // check for cancelation:
                    if (progress && progress->isCanceled())
                    {
                        break;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
                    }
                } 
            }

            valid = image.valid();

            if ( valid )
            {
                scale = key.getExtent().width() / image.getExtent().width();
                bias.x() = (key.getExtent().xMin() - image.getExtent().xMin()) / image.getExtent().width();
                bias.y() = (key.getExtent().yMin() - image.getExtent().yMin()) / image.getExtent().height();

                read = new ImageUtils::PixelReader(image.getImage());

                // cannot interpolate coverage data:
                read->setBilinear( false );

                warp = sourceLayer->getWarp();
            }
        }
    };

    struct RefinementCache : public osg::Referenced
    {
        std::map<TileKey,GeoImage> lut;
    };
}

//........................................................................

#undef  LC
#define LC "[LandCoverLayerOptions] "

void
LandCoverLayer::Options::fromConfig(const Config& conf)
{
    noiseLOD().init(12u);
    warpFactor().init(0.0f);

    conf.get("warp", warpFactor());
    conf.get("noise_lod", noiseLOD());

    ConfigSet layerConfs = conf.child("coverages").children("coverage");
    for (ConfigSet::const_iterator i = layerConfs.begin(); i != layerConfs.end(); ++i)
    {
        _coverages.push_back(*i);
    }
}

Config
LandCoverLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();

    conf.set("warp", warpFactor() );
    conf.set("noise_lod", noiseLOD() );

    if (_coverages.size() > 0)
    {
        Config images("coverages");
        for (std::vector<ConfigOptions>::const_iterator i = _coverages.begin();
            i != _coverages.end();
            ++i)
        {
            images.add("coverage", i->getConfig());
        }
        conf.set(images);
    }

    return conf;
}

//...........................................................................

#undef  LC
#define LC "[LandCoverLayer] "

OE_LAYER_PROPERTY_IMPL(LandCoverLayer, float, WarpFactor, warpFactor);
OE_LAYER_PROPERTY_IMPL(LandCoverLayer, unsigned, NoiseLOD, noiseLOD);

void
LandCoverLayer::init()
{
    options().coverage() = true;

    ImageLayer::init();

    setRenderType(RENDERTYPE_NONE);

    setTileSourceExpected(false);

    layerHints().L2CacheSize() = 64;
}

void
LandCoverLayer::addCoverage(LandCoverCoverageLayer* value)
{
    _coverageLayers.push_back(value);
}

Status
LandCoverLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    const Profile* profile = getProfile();
    if (!profile)
    {
        profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        setProfile(profile);
    }

    // If there are no coverage layers already set by the user,
    // attempt to instaniate them from the serialized options (i.e. earth file).
    if (_coverageLayers.empty())
    {
        for (unsigned i = 0; i < options().coverages().size(); ++i)
        {
            LandCoverCoverageLayer::Options coverageOptions = options().coverages()[i];
            if (coverageOptions.enabled() == false)
                continue;

            // We never want to cache data from a coverage, because the "parent" layer
            // will be caching the entire result of a multi-coverage composite.
            coverageOptions.cachePolicy() = CachePolicy::NO_CACHE;

            // Create the coverage layer:
            LandCoverCoverageLayer* coverage = new LandCoverCoverageLayer(coverageOptions);
            coverage->setCachePolicy(CachePolicy::NO_CACHE);
            coverage->setReadOptions(getReadOptions());
            addCoverage(coverage);
        }
    }

    DataExtentList combinedExtents;

    // next attempt to open and incorporate the coverage layers:
    for(unsigned i=0; i<_coverageLayers.size(); ++i)
    {
        LandCoverCoverageLayer* coverage = _coverageLayers[i].get();
        if (coverage->getEnabled())
        {
            OE_INFO << LC << "Opening coverage layer \"" << coverage->getName() << std::endl;

            // Open the coverage layer and bail if it fails.
            const Status& coverageStatus = coverage->open();
            if (coverageStatus.isError())
            {
                OE_WARN << LC << "Coverage layer failed to open; aborting" << std::endl;
                return coverageStatus;
            }

            if (coverage->getImageLayer())
            {
                coverage->getImageLayer()->setUpL2Cache(9u);
            }

            _codemaps.resize(_codemaps.size()+1);
        }
    }

    // Normally we would collect and store the layer's DataExtents here.
    // Since this is possibly a composited layer with warping, we just
    // let it default so we can oversample the data with warping.

    return Status::NoError;
}

void
LandCoverLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    // Find a land cover dictionary if there is one.
    // There had better be one, or we are not going to get very far!
    // This is called after createTileSource, so the TileSource should exist at this point.
    // Note. If the land cover dictionary isn't already in the Map...this will fail! (TODO)
    // Consider a LayerReference. (TODO)
    _lcDictionary = map->getLayer<LandCoverDictionary>();
    if (_lcDictionary.valid())
    {
        for(unsigned i=0; i<_coverageLayers.size(); ++i)
        {
            _coverageLayers[i]->addedToMap(map);
            buildCodeMap(_coverageLayers[i].get(), _codemaps[i]);
        }
    }
    else
    {
        OE_WARN << LC << "Did not find a LandCoverDictionary in the Map!\n";
    }
}

void
LandCoverLayer::removedFromMap(const Map* map)
{
    ImageLayer::removedFromMap(map);

    for (unsigned i = 0; i < _coverageLayers.size(); ++i)
    {
        _coverageLayers[i]->removedFromMap(map);
    }
}

bool
LandCoverLayer::readMetaImage(MetaImage& metaImage, const TileKey& key, double u, double v, osg::Vec4f& output, ProgressCallback* progress) const
{
    // Find the key containing the uv coordinates.
    int x = int(floor(u));
    int y = int(floor(v));
    TileKey actualKey = (x != 0 || y != 0)? key.createNeighborKey(x, -y) : key;

    if (actualKey.valid())
    {
        // Transform the uv to be relative to the tile it's actually in.
        // Don't forget: stupid computer requires us to handle negatives by hand.
        u = u >= 0.0 ? fmod(u, 1.0) : 1.0+fmod(u, 1.0);
        v = v >= 0.0 ? fmod(v, 1.0) : 1.0+fmod(v, 1.0);

        MetaImageComponent* comp = 0L;

        MetaImage::iterator i = metaImage.find(actualKey);
        if (i != metaImage.end())
        {
            comp = &i->second;
        }
        else
        {
            // Always use the immediate parent for fractal refinement.
            TileKey bestkey = actualKey.createParentKey();

            // should not need this fallback loop but let's do it anyway
            GeoImage tile = const_cast<LandCoverLayer*>(this)->createImage(bestkey, progress);

            if (tile.valid())
            {
                comp = &metaImage[actualKey];
                comp->image = tile.getImage();
                actualKey.getExtent().createScaleBias(bestkey.getExtent(), comp->scaleBias);
                comp->pixel.setImage(comp->image.get());
                comp->pixel.setBilinear(false);
            }
        }

        if (comp)
        {
            // scale/bias to this tile's extent and sample the image.
            u = u * comp->scaleBias(0, 0) + comp->scaleBias(3, 0);
            v = v * comp->scaleBias(1, 1) + comp->scaleBias(3, 1);
            comp->pixel(output, u, v);
            return true;
        }
    }
    return false;
}

GeoImage
LandCoverLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    LandCoverCoverageLayer* coverage = _coverageLayers[0].get();
    ImageLayer* imageLayer = coverage->getImageLayer();

    TileKey parentKey = key.createParentKey();

    TileKey bestKey = imageLayer->getBestAvailableTileKey(key);
    if (bestKey.valid() == false)
        return GeoImage::INVALID;

    if (bestKey == key || !parentKey.valid())
    {
        GeoImage img = imageLayer->createImage(key, progress);
        if (!img.valid())
            return img;

        osg::ref_ptr<osg::Image> output = new osg::Image();
        output->allocateImage(
            getTileSize(),
            getTileSize(),
            1,
            GL_RED,
            GL_FLOAT);
        output->setInternalTextureFormat(GL_R16F);

        ImageUtils::PixelReader read(img.getImage());
        ImageUtils::PixelWriter write(output.get());

        osg::Vec4 pixel;
        bool wrotePixel;
        unsigned pixelsWritten = 0u;

        for (int t = 0; t < output->t(); ++t)
        {
            for (int s = 0; s < output->s(); ++s)
            {
                read(pixel, s, t);

                wrotePixel = false;

                if (pixel.r() < 1.0f)
                {
                    // normalized code; convert
                    int code = (int)(pixel.r()*255.0f);
                    if (code < _codemaps[0].size())
                    {
                        int value = _codemaps[0][code];
                        if (value >= 0)
                        {
                            pixel.r() = (float)value;

                            write(pixel, s, t);
                            wrotePixel = true;
                            pixelsWritten++;
                        }
                    }
                }
                else
                {
                    // unnormalized
                    int code = (int)pixel.r();
                    if (code < _codemaps[0].size() && _codemaps[0][code] >= 0)
                    {
                        pixel.r() = (float)_codemaps[0][code];
                        write(pixel, s, t);
                        wrotePixel = true;
                        pixelsWritten++;
                    }
                }

                if (!wrotePixel)
                {
                    pixel.r() = NO_DATA_VALUE;
                    write(pixel, s, t);
                }
            }
        }

        if (pixelsWritten > 0)
            return GeoImage(output.get(), key.getExtent());
        else
            return GeoImage::INVALID;
    }

    // No data, but want more levels? Fractal refinement starts here.
    if (getMaxDataLevel() > key.getLOD())
    {
        MetaImage metaImage;

        // Allocate the working image:
        osg::ref_ptr<osg::Image> workspace = new osg::Image();
        workspace->allocateImage(
            getTileSize() + 3,
            getTileSize() + 3,
            1,
            GL_RED,
            GL_FLOAT);
        ImageUtils::PixelWriter writeToWorkspace(workspace.get());
        ImageUtils::PixelReader readFromWorkspace(workspace.get());

        // Allocate the output image:
        osg::ref_ptr<osg::Image> output = new osg::Image();
        output->allocateImage(
            getTileSize(),
            getTileSize(),
            1,
            GL_RED,
            GL_FLOAT);
        output->setInternalTextureFormat(GL_R16F);

#if 0
        // Configure the noise function:
        SimplexNoise noiseGen;
        noiseGen.setNormalize(true);
        noiseGen.setRange(0.0, 1.0);
        noiseGen.setFrequency(4.0);
        noiseGen.setPersistence(0.8);
        noiseGen.setLacunarity(2.2);
        noiseGen.setOctaves(8);
#endif

        osg::Vec2d cov;
        osg::Vec4 pixel;
        osg::Vec4 p0, p1, p2, p3;

        // scales the key's LOD to the noise function's LOD:
        float pdL = pow(2, (float)key.getLOD() - options().noiseLOD().get());

        Random prng(key.getTileX()*key.getTileY()*key.getLOD());

        unsigned r;
        int s, t;
        osg::Vec2d noiseCoords;

        // First pass: loop over the grid and populate even pixels with
        // values from the ancestors.
        for (t = 0; t < workspace->t(); ++t)
        {
            double v = (double)(t - 2) / (double)(output->t() - 1);

            for (s = 0; s < workspace->s(); ++s)
            {
                double u = (double)(s - 2) / (double)(output->s() - 1);

                if ((s & 1) == 0 && (t & 1) == 0)
                {
                    if (readMetaImage(metaImage, key, u, v, pixel, progress))
                        writeToWorkspace(pixel, s, t);
                    else if (progress && progress->isCanceled())
                        return GeoImage::INVALID;
                }
            }

            if (progress && progress->isCanceled())
            {
                OE_DEBUG << LC << key.str() << " canceled" << std::endl;
                return GeoImage::INVALID;
            }
        }

        // Second pass: diamond
        for (t = 1; t < workspace->t()-1; ++t)
        {
            //        double v = (double)(t) / (double)(output->t() - 1);
            for (s = 1; s < workspace->s()-1; ++s)
            {
                //            double u = (double)(s) / (double)(output->s() - 1);
                if ((s & 1) == 1 && (t & 1) == 1)
                {
                    //cov.set(u, v);
                    //noiseCoords = getSplatCoords(key, getNoiseLOD(), cov);
                    //double noise = getNoise(noiseGen, noiseCoords);
                    //r = (unsigned)(noise*3.9999999);
                    r = prng.next(4u);

                    // Diamond: pick one of the four diagonals to copy into the
                    // center pixel, attempting to preserve curves. When there is
                    // no clear choice, go random.
                    check(s-1,t-1,workspace->s(),workspace->t());
                    check(s+1,t-1,workspace->s(),workspace->t());
                    check(s-1,t+1,workspace->s(),workspace->t());
                    check(s+1,t+1,workspace->s(),workspace->t());
                    readFromWorkspace(p0, s - 1, t - 1);
                    readFromWorkspace(p1, s + 1, t - 1);
                    readFromWorkspace(p2, s - 1, t + 1);
                    readFromWorkspace(p3, s + 1, t + 1);
                    if (p0.r() == p1.r() && p1.r() == p2.r()) pixel = p0;
                    else if (p1.r() == p2.r() && p2.r() == p3.r()) pixel = p1;
                    else if (p2.r() == p3.r() && p3.r() == p0.r()) pixel = p2;
                    else if (p3.r() == p0.r() && p0.r() == p1.r()) pixel = p3;
                    else if (p0.r() == p1.r() && p2.r() == p3.r()) pixel = (r <= 1) ? p0 : p2;
                    else if (p1.r() == p2.r() && p3.r() == p0.r()) pixel = (r <= 1) ? p1 : p3;
                    else pixel = r == 0 ? p0 : r == 1 ? p1 : r == 2 ? p2 : p3;
                    check(s,t,workspace->s(),workspace->t());
                    writeToWorkspace(pixel, s, t);
                }
            }
        }

        // Third pass: square
        for (t = 2; t < workspace->t() - 1; ++t)
        {
            //        double v = (double)(t) / (double)(output->t() - 1);
            for (s = 2; s < workspace->s() - 1; ++s)
            {
                //            double u = (double)(s) / (double)(output->s() - 1);
                if (((s & 1) == 1 && (t & 1) == 0) || ((s & 1) == 0 && (t & 1) == 1))
                {
                    //cov.set(u, v);
                    //noiseCoords = getSplatCoords(key, getNoiseLOD(), cov);
                    //double noise = getNoise(noiseGen, noiseCoords);
                    //r = (unsigned)(noise*3.9999999);
                    r = prng.next(4u);

                    // Square: pick one of the four adjacents to copy into the
                    // center pixel, attempting to preserve curves. When there is
                    // no clear choice, go random.
                    check(s-1,t,workspace->s(),workspace->t());
                    check(s,t-1,workspace->s(),workspace->t());
                    check(s+1,t,workspace->s(),workspace->t());
                    check(s,t+1,workspace->s(),workspace->t());
                    readFromWorkspace(p0, s - 1, t);
                    readFromWorkspace(p1, s, t - 1);
                    readFromWorkspace(p2, s + 1, t);
                    readFromWorkspace(p3, s, t + 1);
                    if (p0.r() == p1.r() && p1.r() == p2.r()) pixel = p0;
                    else if (p1.r() == p2.r() && p2.r() == p3.r()) pixel = p1;
                    else if (p2.r() == p3.r() && p3.r() == p0.r()) pixel = p2;
                    else if (p3.r() == p0.r() && p0.r() == p1.r()) pixel = p3;
                    else if (p0.r() == p1.r() && p2.r() == p3.r()) pixel = (r <= 1) ? p0 : p2;
                    else if (p1.r() == p2.r() && p3.r() == p0.r()) pixel = (r <= 1) ? p1 : p3;
                    else pixel = r == 0 ? p0 : r == 1 ? p1 : r == 2 ? p2 : p3;
                    check(s,t,workspace->s(),workspace->t());
                    writeToWorkspace(pixel, s, t);
                }
            }
        }

        ImageUtils::PixelWriter writeToOutput(output.get());

        for (t = 0; t < output->t(); ++t)
        {
            for (s = 0; s < output->s(); ++s)
            {
                readFromWorkspace(pixel, s + 2, t + 2);
                writeToOutput(pixel, s, t);
            }
        }

        if (progress && progress->isCanceled())
        {
            return GeoImage::INVALID;
        }

        return GeoImage(output.get(), key.getExtent());
    }
    else
    {
        return GeoImage::INVALID;
    }
}

GeoImage
LandCoverLayer::createMetaImageComponent(const TileKey& key, ProgressCallback* progress) const
{
    if (_coverageLayers.empty())
        return GeoImage::INVALID;

    std::vector<ILayer> layers(_coverageLayers.size());

    // Allocate the new coverage image; it will contain unnormalized values.
    osg::ref_ptr<osg::Image> out = new osg::Image();

    // Allocate a suitable format:
    GLint internalFormat = GL_R16F;

    int tilesize = getTileSize();

    out->allocateImage(tilesize, tilesize, 1, GL_RGB, GL_FLOAT);
    out->setInternalTextureFormat(internalFormat);

    osg::Vec2 cov;    // coverage coordinates

    ImageUtils::PixelWriter write(out.get());
    write.setNormalize(false);

    float du = 1.0f / (float)(out->s() - 1);
    float dv = 1.0f / (float)(out->t() - 1);

    osg::Vec4 nodata(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE);

    unsigned pixelsWritten = 0u;

    for (float u = 0.0f; u <= 1.0f; u += du)
    {
        for (float v = 0.0f; v <= 1.0f; v += dv)
        {
            bool wrotePixel = false;
            for (int L = layers.size() - 1; L >= 0 && !wrotePixel; --L)
            {
                if (progress && progress->isCanceled())
                {
                    OE_DEBUG << LC << key.str() << " canceled" << std::endl;
                    return GeoImage::INVALID;
                }

                ILayer& layer = layers[L];
                if (!layer.valid)
                    continue;

                if (!layer.image.valid())
                    layer.load(key, _coverageLayers[L].get(), progress);

                if (!layer.valid)
                    continue;

                osg::Vec2 cov(layer.scale*u + layer.bias.x(), layer.scale*v + layer.bias.y());

                if (cov.x() >= 0.0f && cov.x() <= 1.0f && cov.y() >= 0.0f && cov.y() <= 1.0f)
                {
                    osg::Vec4 texel = (*layer.read)(cov.x(), cov.y());

                    if (texel.r() != NO_DATA_VALUE)
                    {
                        // store the warp factor in the green channel
                        texel.g() = layer.warp;

                        // store the layer index in the blue channel
                        texel.b() = (float)L;

                        if (texel.r() < 1.0f)
                        {
                            // normalized code; convert
                            int code = (int)(texel.r()*255.0f);
                            if (code < _codemaps[L].size())
                            {
                                int value = _codemaps[L][code];
                                if (value >= 0)
                                {
                                    texel.r() = (float)value;
                                    write.f(texel, u, v);
                                    wrotePixel = true;
                                    pixelsWritten++;
                                }
                            }
                        }
                        else
                        {
                            // unnormalized
                            int code = (int)texel.r();
                            if (code < _codemaps[L].size() && _codemaps[L][code] >= 0)
                            {
                                texel.r() = (float)_codemaps[L][code];
                                write.f(texel, u, v);
                                wrotePixel = true;
                                pixelsWritten++;
                            }
                        }
                    }
                }
            }

            if (!wrotePixel)
            {
                write.f(nodata, u, v);
            }
        }
    }

    if (pixelsWritten > 0u)
        return GeoImage(out.release(), key.getExtent());
    else
        return GeoImage::INVALID;
}

// Constructs a code map (int to int) for a coverage layer. We will use this
// code map to map coverage layer codes to dictionary codes.
void
LandCoverLayer::buildCodeMap(const LandCoverCoverageLayer* coverage, CodeMap& codemap)
{
    if (!coverage) {
        OE_WARN << LC << "ILLEGAL: coverage not passed to buildCodeMap\n";
        return;
    }
    if (!_lcDictionary.valid()) {
        OE_WARN << LC << "ILLEGAL: coverage dictionary not set in buildCodeMap\n";
        return;
    }

    //OE_INFO << LC << "Building code map for " << coverage->getName() << "..." << std::endl;

    int highestValue = 0;

    for (LandCoverValueMappingVector::const_iterator k = coverage->getMappings().begin();
        k != coverage->getMappings().end();
        ++k)
    {
        const LandCoverValueMapping* mapping = k->get();
        int value = mapping->getValue();
        if (value > highestValue)
            highestValue = value;
    }

    codemap.assign(highestValue + 1, -1);

    for (LandCoverValueMappingVector::const_iterator k = coverage->getMappings().begin();
        k != coverage->getMappings().end();
        ++k)
    {
        const LandCoverValueMapping* mapping = k->get();
        int value = mapping->getValue();
        const LandCoverClass* lcClass = _lcDictionary->getClassByName(mapping->getLandCoverClassName());
        if (lcClass)
        {
            codemap[value] = lcClass->getValue();
            //OE_INFO << LC << "   mapped " << value << " to " << lcClass->getName() << std::endl;
        }
    }
}

//........................................................................

#undef  LC
#define LC "[LandCoverLayerVector] "

LandCoverLayerVector::LandCoverLayerVector()
{
    //nop
}


LandCoverLayerVector::LandCoverLayerVector(const LandCoverLayerVector& rhs) :
    osg::MixinVector< osg::ref_ptr<LandCoverLayer> >( rhs )
{
    //nop
}

// Composites a vector of land cover images into a single image.
bool
LandCoverLayerVector::populateLandCoverImage(
    osg::ref_ptr<osg::Image>& output,
    const TileKey&            key,
    ProgressCallback*         progress ) const
{
    if (empty())
        return false;

    // Special case of one image - no compositing necessary.
    if (size() == 1)
    {
        if (begin()->get()->getEnabled())
        {
            GeoImage r = begin()->get()->createImage(key, progress);
            output = r.getImage();
        }
        return output.valid();
    }

    bool fallback = false;          // whether to fall back on parent tiles for a component
    bool needsClone = false;        // whether to clone the output image

    osg::Vec4 value;
    unsigned numValues = 0u;
    unsigned numNoDataValues = 1u;

    // Iterate backwards since the last image has the highest priority.
    // If we get an image with all valid values (no NO_DATA), we are finished
    for(const_reverse_iterator i = rbegin(); i != rend() && numNoDataValues > 0u; ++i)
    {
        LandCoverLayer* layer = i->get();

        if (!layer->getEnabled())
            continue;

        GeoImage comp;

        osg::Matrixd compScaleBias;
        
        // Fetch the image for the current component. If necessary, fall back on
        // ancestor tilekeys until we get a result. This is necessary if we 
        // already have some data but there are NO DATA values that need filling.
        if (fallback)
        {
            TileKey compKey = key;
            while(comp.valid() == false && compKey.valid())
            {
                comp = layer->createImage(compKey, progress);
                if (!comp.valid())
                    compKey = compKey.createParentKey();
                else
                    key.getExtent().createScaleBias(compKey.getExtent(), compScaleBias);
            }
        }
        else
        {
            comp = layer->createImage(key, progress);
        }

        if (!comp.valid())
            continue;  
        
        ImageUtils::PixelReader readInput(comp.getImage());

        // scale and bias to read an ancestor (fallback) tile if necessary.
        double 
            scale = compScaleBias(0,0), 
            sbias = compScaleBias(3,0)*readInput.s(),
            tbias = compScaleBias(3,1)*readInput.t();

        // If this is the first image, scan the image for NO_DATA values.
        if (!output.valid())
        {
            numNoDataValues = 0u;

            for(int t=0; t<readInput.t() && numNoDataValues == 0u; ++t)
            {
                for(int s=0; s<readInput.s() && numNoDataValues == 0u; ++s)
                {
                    readInput(value, (int)(s*scale+sbias), (int)(t*scale+tbias));
                    if (value.r() == NO_DATA_VALUE)
                    {
                        numNoDataValues++;
                    }
                }
            }

            output = comp.getImage();
            numValues = output->s() * output->t();
            needsClone = true;
            fallback = true;
            continue;
        }

        // The second image to arrive requires that we clone the data
        // since we are going to modify it.
        if (needsClone)
        {
            output = osg::clone(output.get(), osg::CopyOp::DEEP_COPY_ALL);
            needsClone = false;
        }

        // now composite this image under the previous one, 
        // accumulating a count of NO_DATA values along the way.
        ImageUtils::PixelReader readOutput(output.get());
        ImageUtils::PixelWriter writeOutput(output.get());

        numNoDataValues = 0u;

        for(unsigned t=0; t<readOutput.t(); ++t)
        {
            for(unsigned s=0; s<readOutput.s(); ++s)
            {
                readOutput(value, s, t);

                if (value.r() == NO_DATA_VALUE)
                {
                    readInput(value, (int)(s*scale+sbias), (int)(t*scale+tbias));

                    if (value.r() == NO_DATA_VALUE)
                        numNoDataValues++;
                    else
                        writeOutput(value, s, t);
                }
            }
        }
    }

    // If the image is ALL nodata ... return NULL.
    if (numNoDataValues == numValues)
    {
        output = NULL;
        return false;
    }
    else
    {
        return output.valid();
    }
}
