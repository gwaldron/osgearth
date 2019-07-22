/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthUtil/FractalElevationLayer>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/SimplexNoise>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[FractalElevationLayer] "

#define OE_TEST OE_DEBUG

REGISTER_OSGEARTH_LAYER(fractal_elevation, FractalElevationLayer);

//............................................................................

namespace
{
    void scaleCoordsToLOD(double& u, double& v, int baseLOD, const TileKey& key)
    {
        double dL = (double)((int)key.getLOD() - baseLOD);
        double factor = pow(2.0, dL); //exp2(dL) .. exp2 not available on some platforms
        double invFactor = 1.0 / factor;

        u *= invFactor;
        v *= invFactor;

        if (factor >= 1.0)
        {
            unsigned nx, ny;
            key.getProfile()->getNumTiles(key.getLOD(), nx, ny);

            double tx = (double)key.getTileX();
            double ty = (double)ny - (double)key.getTileY() - 1.0;

            double ax = floor(tx * invFactor);
            double ay = floor(ty * invFactor);

            double bx = ax * factor;
            double by = ay * factor;

            double cx = bx + factor;
            double cy = by + factor;

            u += (tx - bx) / (cx - bx);
            v += (ty - by) / (cy - by);
        }
    }
}

//............................................................................

FractalElevationLayerOptions::FractalElevationLayerOptions(const ConfigOptions& co) :
ElevationLayerOptions(co)
{
    _baseLOD.init(11u);
    _amplitude.init(5.0f);
    _frequency.init(64.0f);
    _persistence.init(0.5f);
    _lacunarity.init(2.0f);
    fromConfig(_conf);
}

void
FractalElevationLayerOptions::fromConfig(const Config& conf)
{
    conf.get("base_lod", _baseLOD);
    conf.get("amplitude", _amplitude);
    conf.get("frequency", _frequency);
    conf.get("persistence", _persistence);
    conf.get("lacunarity", _lacunarity);
    conf.get("noise_image", _noiseImageURI);

    const ConfigSet& lcmap = conf.child("land_cover_mappings").children();
    for (ConfigSet::const_iterator i = lcmap.begin(); i != lcmap.end(); ++i)
    {
        const Config& mapping = *i;
        FractalElevationLayerLandCoverMapping m;
        m.className = mapping.value("class");
        mapping.get("amplitude", m.amplitude);
        if (!m.className.empty() && m.amplitude.isSet())
            _lcMap[m.className] = m;
    }
}

Config
FractalElevationLayerOptions::getConfig() const
{
    Config conf = ElevationLayerOptions::getConfig();
    conf.set("base_lod", _baseLOD);
    conf.set("amplitude", _amplitude);
    conf.set("frequency", _frequency);
    conf.set("persistence", _persistence);
    conf.set("lacunarity", _lacunarity);
    conf.set("noise_image", _noiseImageURI);

    if (!_lcMap.empty())
    {
        Config mappings("land_cover_mappings");
        for (FractalElevationLayerLandCoverMap::const_iterator i = _lcMap.begin(); i != _lcMap.end(); ++i)
        {
            Config mapping("mapping");
            mapping.set("class", i->first);
            mapping.set("amplitude", i->second.amplitude.get());
            mappings.add(mapping);
        }
        conf.set(mappings);
    }

    return conf;
}

//............................................................................

FractalElevationLayer::FractalElevationLayer() :
ElevationLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

FractalElevationLayer::FractalElevationLayer(const FractalElevationLayerOptions& options) :
ElevationLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

FractalElevationLayer::~FractalElevationLayer()
{
    //todo
}

void
FractalElevationLayer::init()
{
    _debug = false;

    // No tile source; we will override createImplementation
    setTileSourceExpected(false);

    // Global WGS84 profile
    setProfile(Profile::create("global-geodetic"));
    
    // Default the tile size to 257
    if (!options().tileSize().isSet())
        options().tileSize().init(257u);

    // If we try to use this layer 5 LODs beyond it's baseLOD, there is a 
    // quantization that happens and the normaly become faceted. Need to track
    // this down, but in the meantime, limit the output to baseLOD+5.
    if (options().maxDataLevel().isSet())
    {
        if (options().maxDataLevel().get() - options().baseLOD().get() > 5)
        {
            options().maxDataLevel() = options().baseLOD().get() + 5;
        }
    }
    else
    {
        options().maxDataLevel() = options().baseLOD().get() + 5;
    }


    // Build the first noise image in memory.
    SimplexNoise noise;
    noise.setFrequency(options().frequency().get());
    noise.setPersistence(options().persistence().get());
    noise.setLacunarity(options().lacunarity().get());
    noise.setOctaves(12u);
    _noiseImage1 = noise.createSeamlessImage(1024u);

    // Try to load a secondary noise image:
    if (options().noiseImageURI().isSet())
    {
        _noiseImage2 = options().noiseImageURI()->getImage(getReadOptions());
        if (!_noiseImage2.valid())
        {
            //return Status::Error(Status::ServiceUnavailable, "Failed to load noise image");
        }
    }

    // Print info about land cover mappings.
    if (!options().landCoverMap().empty())
    {
        OE_INFO << LC << "Land cover to amplitude mappings:\n";
        for(FractalElevationLayerLandCoverMap::const_iterator i = options().landCoverMap().begin();
            i != options().landCoverMap().end();
            ++i)
        {
            const FractalElevationLayerLandCoverMapping& mapping = i->second;
            OE_INFO << LC << "   " << i->second.className << " => " << i->second.amplitude.get() << "\n";
        }
    }

    ElevationLayer::init();
}

const Status&
FractalElevationLayer::open()
{
    return ElevationLayer::open();
}

void
FractalElevationLayer::addedToMap(const Map* map)
{
    if (map)
    {
        _landCover = map->getLayer<LandCoverLayer>();
        if (_landCover.valid())
            OE_INFO << LC << "Found land cover layer.\n";

        _landCoverDict = map->getLayer<LandCoverDictionary>();
        if (_landCoverDict.valid())
            OE_INFO << LC << "Found land cover dictionary.\n";
    }
}

void
FractalElevationLayer::removedFromMap(const Map* map)
{
    _landCover = 0L;
    _landCoverDict = 0L;
}

void
FractalElevationLayer::createImplementation(const TileKey& key,
                                            osg::ref_ptr<osg::HeightField>& out_hf,
                                            osg::ref_ptr<NormalMap>& out_normalMap,
                                            ProgressCallback* progress)
{
    double min_n = FLT_MAX, max_n = -FLT_MAX;
    double min_h = FLT_MAX, max_h = -FLT_MAX;
    double h_mean = 0.0;

    ImageUtils::PixelReader noise1(_noiseImage1.get());
    noise1.setBilinear(true);

    ImageUtils::PixelReader noise2(_noiseImage2.get());
    noise2.setBilinear(true);

    osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(
        key.getExtent(), getTileSize(), getTileSize(), 0u);

    // land cover tile
    GeoImage lcTile;

    osg::ref_ptr<LandCoverLayer> lcLayer;
    _landCover.lock(lcLayer);

    if (lcLayer.valid())
    {
        lcTile = lcLayer->createImage(key, progress);
    }

    for (int s = 0; s < getTileSize(); ++s)
    {
        for (int t = 0; t < getTileSize(); ++t)
        {
            double u = (double)s / (double)(getTileSize() - 1);
            double v = (double)t / (double)(getTileSize() - 1);

            double n = 0.0;
            double uScaled, vScaled;

            double finalScale = 4.0;

            // Step 1
            if (_noiseImage1.valid())
            {
                uScaled = u, vScaled = v;
                scaleCoordsToLOD(uScaled, vScaled, options().baseLOD().get(), key);

                double uMod = fmod(uScaled, 1.0);
                double vMod = fmod(vScaled, 1.0);

                n += noise1(uMod, vMod).r() - 0.5;
                finalScale *= 0.5;
            }

            if (_noiseImage2.valid())
            {
                uScaled = u, vScaled = v;
                scaleCoordsToLOD(uScaled, vScaled, options().baseLOD().get() + 3, key);

                double uMod = fmod(uScaled, 1.0);
                double vMod = fmod(vScaled, 1.0);
                n += noise2(uMod, vMod).r() - 0.5;
                finalScale *= 0.5;
            }

            n *= finalScale;

            // default amplitude:
            float amp = options().amplitude().get();

            // if we have land cover mappings, use them:
            if (lcTile.valid())
            {
                const LandCoverClass* lcClass = lcLayer->getClassByUV(lcTile, u, v);
                if (lcClass)
                {
                    const FractalElevationLayerLandCoverMapping* mapping = getMapping(lcClass);
                    if (mapping)
                    {
                        amp = mapping->amplitude.getOrUse(amp);
                    }
                }
            }

            hf->setHeight(s, t, n * amp);

            if (_debug)
            {
                h_mean += hf->getHeight(s, t);
                min_n = osg::minimum(min_n, n);
                max_n = osg::maximum(max_n, n);
                min_h = osg::minimum(min_h, (double)hf->getHeight(s, t));
                max_h = osg::maximum(max_h, (double)hf->getHeight(s, t));
            }
        }
    }

    if (_debug)
    {
        h_mean /= double(getTileSize()*getTileSize());
        double q_mean = 0.0;

        for (int s = 0; s < getTileSize(); ++s)
        {
            for (int t = 0; t < getTileSize(); ++t)
            {
                double q = hf->getHeight(s, t) - h_mean;
                q_mean += q*q;
            }
        }

        double stdev = sqrt(q_mean / double(getTileSize()*getTileSize()));

        OE_INFO << LC << "Tile " << key.str() << " Hmean=" << h_mean
            << ", stdev=" << stdev << ", n[" << min_n << ", " << max_n << "] "
            << "h[" << min_h << ", " << max_h << "]\n";
    }

    out_hf = hf.release();
    out_normalMap = 0L;
}

const FractalElevationLayerLandCoverMapping*
FractalElevationLayer::getMapping(const LandCoverClass* lcc) const
{
    if (!lcc) return 0L;
    FractalElevationLayerLandCoverMap::const_iterator i = options().landCoverMap().find(lcc->getName());
    return i != options().landCoverMap().end() ? &(i->second) : 0L;
}
