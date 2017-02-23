/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/Map>
#include <osgEarthUtil/SimplexNoise>
#include <cstdlib> // for getenv

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[FractalElevationLayer] "

#define OE_TEST OE_DEBUG

REGISTER_OSGEARTH_LAYER(fractal_elevation, FractalElevationLayer);

namespace
{
    //! TileSource that provided elevation fiends to the FratalElevationLayer.
    class FractalElevationTileSource : public TileSource
    {
    public:
        // Ctor
        FractalElevationTileSource(const FractalElevationLayerOptions& layerOptions) :
            TileSource(layerOptions),
            _options(layerOptions)
        {
            _debug = (::getenv("OSGEARTH_NOISE_DEBUG") != 0L);
        }

        virtual ~FractalElevationTileSource() { }

    public: // TileSource

        // Make sure everything is OK and return open status
        Status initialize(const osgDB::Options* readOptions)
        {

            setProfile(Profile::createNamed("global-geodetic"));

            return Status::OK();
        }

        int getPixelsPerTile() const
        {
            return 257;
        }

        // Generate a flattening heightfield for a key
        osg::HeightField* createHeightField(const TileKey& key, ProgressCallback* progress)
        {
            double min_n = FLT_MAX, max_n = -FLT_MAX;

            // Configure the noise function:
            SimplexNoise noise;
            noise.setNormalize(true);
            noise.setRange(0.0, 1.0);
            noise.setFrequency(_options.frequency().get());
            noise.setPersistence(_options.persistence().get());
            noise.setLacunarity(_options.lacunarity().get());
            noise.setOctaves(std::min(12u, key.getLOD()-_options.baseLOD().get()));

            osg::HeightField* hf = HeightFieldUtils::createReferenceHeightField(key.getExtent(), getPixelsPerTile(), getPixelsPerTile(), 0u);
            for (int s = 0; s < getPixelsPerTile(); ++s)
            {
                for (int t = 0; t < getPixelsPerTile(); ++t)
                {
                    double u = (double)s / (double)(getPixelsPerTile() - 1);
                    double v = (double)t / (double)(getPixelsPerTile() - 1);

                    scaleCoordsToLOD(u, v, _options.baseLOD().get(), key);

                    // The 4.0 multiplier stretched the amplitude into +/-.
                    double n = 4.0*(noise.getTiledValue(u, v)-0.5);

                    hf->setHeight(s, t, n * _options.amplitude().get());

                    if (_debug)
                    {
                        min_n = std::min(min_n, n);
                        max_n = std::max(max_n, n);
                    }
                }
            }
            //OE_INFO << "Make HF for " << key.str() << std::endl;

            if (_debug)
            {
                OE_INFO << LC << "Tile " << key.str() << " N=(" << min_n << ", " << max_n << ")\n";
            }

            return hf;
        }

    private:

        void scaleCoordsToLOD(double& u, double& v, int baseLOD, const TileKey& key)
        {
            double dL = (double)((int)key.getLOD() - baseLOD);
            double factor = pow(2.0, dL); //exp2(dL);
            double invFactor = 1.0/factor;

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

        const FractalElevationLayerOptions& _options;
        SimplexNoise _noise;
        bool _debug;
    };
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
    conf.getIfSet("base_lod", _baseLOD);
    conf.getIfSet("amplitude", _amplitude);
    conf.getIfSet("frequency", _frequency);
    conf.getIfSet("persistence", _persistence);
    conf.getIfSet("lacunarity", _lacunarity);
}

Config
FractalElevationLayerOptions::getConfig() const
{
    Config conf = ElevationLayerOptions::getConfig();
    conf.key() = "fractal_elevation";
    conf.addIfSet("base_lod", _baseLOD);
    conf.addIfSet("amplitude", _amplitude);
    conf.addIfSet("frequency", _frequency);
    conf.addIfSet("persistence", _persistence);
    conf.addIfSet("lacunarity", _lacunarity);
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
    ElevationLayer::init();
}

TileSource*
FractalElevationLayer::createTileSource()
{
    return new FractalElevationTileSource(options());
}

const Status&
FractalElevationLayer::open()
{
    return ElevationLayer::open();
}

void
FractalElevationLayer::addedToMap(const Map* map)
{
    //todo
}

void
FractalElevationLayer::removedFromMap(const Map* map)
{
    //todo
}
