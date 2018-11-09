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
#include <osgEarth/MetaTile>
#include <osgEarth/SimplexNoise>
#include <osgEarth/Progress>

using namespace osgEarth;

#define LC "[LandCoverLayer] "

REGISTER_OSGEARTH_LAYER(land_cover, LandCoverLayer);

namespace
{
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

    
    typedef std::vector<int> CodeMap;

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
            if (sourceLayer->getEnabled() && 
                sourceLayer->isKeyInLegalRange(key) &&
                sourceLayer->mayHaveData(key))
            {
                for(TileKey k = key; k.valid() && !image.valid(); k = k.createParentKey())
                {
                    image = sourceLayer->createImage(k, progress);

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

                warp = sourceLayer->options().warp().get();
            }
        }
    };

    // Constructs a code map (int to int) for a coverage layer. We will use this
    // code map to map coverage layer codes to dictionary codes.
    void buildCodeMap(const LandCoverCoverageLayer* coverage, CodeMap& codemap)
    {
        if (!coverage) {
            OE_WARN << LC << "ILLEGAL: coverage not passed to buildCodeMap\n";
            return;
        }
        if (!coverage->getDictionary()) {
            OE_WARN << LC << "ILLEGAL: coverage dictionary not set in buildCodeMap\n";
            return;
        }

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

        codemap.assign(highestValue+1, -1);

        for (LandCoverValueMappingVector::const_iterator k = coverage->getMappings().begin();
            k != coverage->getMappings().end();
            ++k)
        {
            const LandCoverValueMapping* mapping = k->get();
            int value = mapping->getValue();
            const LandCoverClass* lcClass = coverage->getDictionary()->getClassByName(mapping->getLandCoverClassName());
            if (lcClass)
            {
                codemap[value] = lcClass->getValue();
            }
        }
    }

    typedef std::vector<osg::ref_ptr<LandCoverCoverageLayer> > LandCoverCoverageLayerVector;

    /*
     * TileSource that provides GeoImage's to the LandCoverLayer.
     */
    class LandCoverTileSource : public TileSource
    {
    public:
        LandCoverTileSource(const LandCoverLayerOptions& options);

    public: // TileSource
        Status initialize(const osgDB::Options* readOptions);

        osg::Image* createImage(const TileKey& key, ProgressCallback* progress);
        
        CachePolicy getCachePolicyHint() const {
            return CachePolicy::NO_CACHE;
        }

        const LandCoverLayerOptions* _options;
        const LandCoverLayerOptions& options() const { return *_options; }

        void setDictionary(LandCoverDictionary*);
        
        // image layers, one per data source
        LandCoverCoverageLayerVector _coverages;

        // code maps (vector index is the source code; value is the destination code)
        std::vector<CodeMap> _codemaps;

        // todo
        std::vector<float> _warps;

        osg::ref_ptr<osgDB::Options> _readOptions;  

        osg::ref_ptr<LandCoverDictionary> _lcDictionary;

        friend class LandCoverLayer;
    };


    LandCoverTileSource::LandCoverTileSource(const LandCoverLayerOptions& options) :
        TileSource(options),
        _options(&options)
    {
        // Increase the L2 cache size since the parent LandCoverLayer is going to be
        // using meta-tiling to create mosaics for warping
        setDefaultL2CacheSize(64);
    }

    Status
    LandCoverTileSource::initialize(const osgDB::Options* readOptions)
    {
        const Profile* profile = getProfile();
        if ( !profile )
        {
            profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
            setProfile( profile );
        }

        for(unsigned i=0; i<options().coverages().size(); ++i)
        {
            LandCoverCoverageLayerOptions coverageOptions = options().coverages()[i];
            if (coverageOptions.enabled() == false)
                continue;

            // We never want to cache data from a coverage, because the "parent" layer
            // will be caching the entire result of a multi-coverage composite.
            coverageOptions.cachePolicy() = CachePolicy::NO_CACHE;

            // Create the coverage layer:
            LandCoverCoverageLayer* layer = new LandCoverCoverageLayer( coverageOptions );

            // Set up and open it.
            layer->setTargetProfileHint( profile );
            layer->setReadOptions(readOptions);
            const Status& s = layer->open();
            if (s.isOK())
            {
                _coverages.push_back(layer);
                _codemaps.resize(_codemaps.size()+1);
                OE_INFO << LC << "Opened coverage \"" << layer->getName() << "\"\n";
            }
            else
            {
                OE_WARN << "Layer \"" << layer->getName() << "\": " << s.toString() << std::endl;
            }

            // Integrate data extents into this tile source.
            const DataExtentList& de = layer->getDataExtents();
            for(DataExtentList::const_iterator dei = de.begin(); dei != de.end(); ++dei)
            {
                if (!profile || dei->getSRS()->isHorizEquivalentTo(profile->getSRS()))
                {
                    getDataExtents().push_back(*dei);
                }
                else
                {
                    // Transform the data extents to the layer profile
                    GeoExtent ep = dei->transform(profile->getSRS());
                    DataExtent de(ep);
                    if (dei->minLevel().isSet())
                        de.minLevel() = profile->getEquivalentLOD(layer->getProfile(), dei->minLevel().get());
                    if (dei->maxLevel().isSet())
                        de.maxLevel() = profile->getEquivalentLOD(layer->getProfile(), dei->maxLevel().get());
                    getDataExtents().push_back(de);
                }
            }
        }

        return STATUS_OK;
    }

    void
    LandCoverTileSource::setDictionary(LandCoverDictionary* lcd)
    {
        _lcDictionary = lcd;

        for (unsigned i = 0; i<_coverages.size(); ++i)
        {
            _coverages[i]->setDictionary(lcd);
            buildCodeMap(_coverages[i].get(), _codemaps[i]);
        }
    }
    
    //TODO: overriding createImage directly like this will bypass caching.
    //      This is a temporary solution; need to refactor.
    osg::Image*
    LandCoverTileSource::createImage(const TileKey& key, ProgressCallback* progress)
    {
        if ( _coverages.empty() )
            return 0L;

        std::vector<ILayer> layers(_coverages.size());

        // Allocate the new coverage image; it will contain unnormalized values.
        osg::ref_ptr<osg::Image> out = new osg::Image();
        ImageUtils::markAsUnNormalized(out.get(), true);

        // Allocate a suitable format:
        GLint internalFormat = GL_R16F;

        int tilesize = getPixelsPerTile();

        out->allocateImage(tilesize, tilesize, 1, GL_RGB, GL_FLOAT);
        out->setInternalTextureFormat(internalFormat);

        osg::Vec2 cov;    // coverage coordinates

        ImageUtils::PixelWriter write( out.get() );

        float du = 1.0f / (float)(out->s()-1);
        float dv = 1.0f / (float)(out->t()-1);

        osg::Vec4 nodata(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE);

        unsigned pixelsWritten = 0u;

        for(float u=0.0f; u<=1.0f; u+=du)
        {
            for(float v=0.0f; v<=1.0f; v+=dv)
            {
                bool wrotePixel = false;
                for(int L = layers.size()-1; L >= 0 && !wrotePixel; --L)
                {
                    if (progress && progress->isCanceled())
                    {
                        OE_DEBUG << LC << key.str() << " canceled" << std::endl;
                        return 0L;
                    }

                    ILayer& layer = layers[L];
                    if ( !layer.valid )
                        continue;

                    if ( !layer.image.valid() )
                        layer.load(key, _coverages[L].get(), progress);

                    if (!layer.valid)
                        continue;

                    osg::Vec2 cov(layer.scale*u + layer.bias.x(), layer.scale*v + layer.bias.y());

                    if ( cov.x() >= 0.0f && cov.x() <= 1.0f && cov.y() >= 0.0f && cov.y() <= 1.0f )
                    {
                        osg::Vec4 texel = (*layer.read)(cov.x(), cov.y());

                        if ( texel.r() != NO_DATA_VALUE )
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

                if ( !wrotePixel )
                {
                    write.f(nodata, u, v);
                }
            }
        }

        return pixelsWritten > 0u? out.release() : 0L;
    }
}

//........................................................................

#undef  LC
#define LC "[LandCoverLayerOptions] "

LandCoverLayerOptions::LandCoverLayerOptions(const ConfigOptions& options) :
ImageLayerOptions(options),
_noiseLOD(12u),
_warp(0.0f)
{
    fromConfig(_conf);
}

void
LandCoverLayerOptions::fromConfig(const Config& conf)
{
    conf.get("warp", _warp);
    conf.get("noise_lod", _noiseLOD);

    ConfigSet layerConfs = conf.child("coverages").children("coverage");
    for (ConfigSet::const_iterator i = layerConfs.begin(); i != layerConfs.end(); ++i)
    {
        _coverages.push_back(LandCoverCoverageLayerOptions(*i));
    }
}

Config
LandCoverLayerOptions::getConfig() const
{
    Config conf = ImageLayerOptions::getConfig();

    conf.set("warp", _warp);
    conf.set("noise_lod", _noiseLOD);

    if (_coverages.size() > 0)
    {
        Config images("coverages");
        for (std::vector<LandCoverCoverageLayerOptions>::const_iterator i = _coverages.begin();
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


LandCoverLayer::LandCoverLayer() :
ImageLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

LandCoverLayer::LandCoverLayer(const LandCoverLayerOptions& options) :
ImageLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
LandCoverLayer::init()
{
    options().coverage() = true;
    options().visible() = false;
    options().shared() = true;
    ImageLayer::init();
}

void
LandCoverLayer::addedToMap(const Map* map)
{
    // Find a land cover dictionary if there is one.
    // There had better be one, or we are not going to get very far!
    // This is called after createTileSource, so the TileSource should exist at this point.
    // Note. If the land cover dictionary isn't already in the Map...this will fail! (TODO)
    // Consider a LayerListener. (TODO)
    _lcDictionary = map->getLayer<LandCoverDictionary>();
    if (_lcDictionary.valid() && getTileSource())
    {
        static_cast<LandCoverTileSource*>(getTileSource())->setDictionary(_lcDictionary.get());
    }
    else
    {
        OE_WARN << LC << "Did not find a LandCoverDictionary in the Map!\n";
    }
}

TileSource*
LandCoverLayer::createTileSource()
{
    return new LandCoverTileSource(options());
}

GeoImage
LandCoverLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress)
{
    if (true) // warping enabled
    {
        MetaImage metaImage;

        for (int x = -1; x <= 1; ++x)
        {
            for (int y = -1; y <= 1; ++y)
            {
                // compute the neighoring key:
                TileKey subkey = key.createNeighborKey(x, y);
                if (subkey.valid())
                {
                    // compute the closest ancestor key with actual data for the neighbor key:
                    TileKey bestkey = getBestAvailableTileKey(subkey);
                    if (bestkey.valid())
                    {
                        // load the image and store it to the metaimage.
                        GeoImage tile = ImageLayer::createImageImplementation(bestkey, progress);
                        if (tile.valid())
                        {
                            osg::Matrix scaleBias;
                            subkey.getExtent().createScaleBias(bestkey.getExtent(), scaleBias);
                            metaImage.setImage(x, y, tile.getImage(), scaleBias);
                        }
                    }
                }

                if (progress && progress->isCanceled())
                {
                    OE_DEBUG << LC << key.str() << " canceled" << std::endl;
                    return GeoImage::INVALID;
                }
            }
        }

        const osg::Image* mainImage = metaImage.getImage(0, 0);
        if ( !mainImage)
            return GeoImage::INVALID;

        // new image for the warped data:
        osg::ref_ptr<osg::Image> image = new osg::Image();
        image->allocateImage(
            mainImage->s(),
            mainImage->t(),
            mainImage->r(),
            mainImage->getPixelFormat(),
            mainImage->getDataType(),
            mainImage->getPacking());
        image->setInternalTextureFormat(mainImage->getInternalTextureFormat());
        ImageUtils::markAsUnNormalized(image.get(), true);

        ImageUtils::PixelWriter write(image.get());

        // Configure the noise function:
        SimplexNoise noiseGen;
        noiseGen.setNormalize(true);
        noiseGen.setRange(0.0, 1.0);
        noiseGen.setFrequency(4.0);
        noiseGen.setPersistence(0.8);
        noiseGen.setLacunarity(2.2);
        noiseGen.setOctaves(8);

        osg::Vec2d cov;
        osg::Vec2 noiseCoords;
        osg::Vec4 pixel;
        osg::Vec4 nodata(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE);
        
        float pdL = pow(2, (float)key.getLOD() - options().noiseLOD().get());

        for (int t = 0; t < image->t(); ++t)
        {
            double v = (double)t / (double)(image->t() - 1);
            for (int s = 0; s < image->s(); ++s)
            {
                double u = (double)s / (double)(image->s() - 1);

                cov.set(u, v);

                // first read the unwarped pixel to get the warping value.
                // (warp is stored in pixel.g)
                metaImage.read(cov.x(), cov.y(), pixel);
                float warp = pixel.g() * pdL;

                noiseCoords = getSplatCoords(key, options().noiseLOD().get(), cov);
                double noise = getNoise(noiseGen, noiseCoords);
                cov = warpCoverageCoords(cov, noise, warp);

                if (metaImage.read(cov.x(), cov.y(), pixel))
                {
                    // only apply the warping if the location of the warped pixel
                    // came from the same source layer. Otherwise you will get some
                    // unsavory speckling. (Layer index is stored in pixel.b)
                    osg::Vec4 unwarpedPixel;
                    if (metaImage.read(u, v, unwarpedPixel) && pixel.b() != unwarpedPixel.b())
                        write(unwarpedPixel, s, t);
                    else
                        write(pixel, s, t);
                }
                else
                {
                    write(nodata, s, t);
                }
                
                if (progress && progress->isCanceled())
                {
                    OE_DEBUG << LC << key.str() << " canceled" << std::endl;
                    return GeoImage::INVALID;
                }
            }
        }

        return GeoImage(image.get(), key.getExtent());
    }

    else
    {
        return ImageLayer::createImageImplementation(key, progress);
    }
}

const LandCoverClass*
LandCoverLayer::getClassByUV(const GeoImage& tile, double u, double v) const
{
    if (!tile.valid())
        return 0L;

    if (!_lcDictionary.valid())
        return 0L;

    ImageUtils::PixelReader read(tile.getImage());
    read.setBilinear(false); // nearest neighbor only!
    float value = read(u, v).r();

    return _lcDictionary->getClassByValue((int)value);
}
