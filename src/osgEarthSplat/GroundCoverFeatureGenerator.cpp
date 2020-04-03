/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include "GroundCoverFeatureGenerator"
#include "GroundCover"
#include "NoiseTextureFactory"
#include <osgEarth/ImageUtils>

using namespace osgEarth;
using namespace osgEarth::Splat;

#define LC "[GroundCoverFeatureGenerator] "

//...................................................................

namespace
{
    // GLSL fract
    inline float fract(float x)
    {
        return fmodf(x, 1.0f);
    }

    // GLSL clamp
    inline float clamp(float x, float m0, float m1)
    {
        return osg::clampBetween(x, m0, m1);
    }

    // Sample a texture with a scale/bias matrix
    inline void sample(osg::Vec4f& output, ImageUtils::PixelReader& texture, const osg::Matrixf& matrix, float u, float v)
    {
        u = clamp(u*matrix(0, 0) + matrix(3, 0), 0.0f, 1.0f);
        v = clamp(v*matrix(1, 1) + matrix(3, 1), 0.0f, 1.0f);
        return texture(output, u, v);
    }

    // Noise channels
    const int NOISE_SMOOTH = 0;
    const int NOISE_RANDOM = 1;
    const int NOISE_RANDOM_2 = 2;
    const int NOISE_CLUMPY = 3;

    // biome weighted index lookup table
    struct BillboardLUTEntry
    {
        float width;
        float height;
        float sizeVariation;
    };

    typedef std::vector<BillboardLUTEntry> BillboardLUT;

    typedef UnorderedMap<const GroundCoverBiome*, BillboardLUT> BiomeLUT;

    void buildLUT(const GroundCover* gc, BiomeLUT& lut)
    {
        for(GroundCoverBiomes::const_iterator b = gc->getBiomes().begin();
            b != gc->getBiomes().end();
            ++b)
        {
            BillboardLUT& billboards = lut[b->get()];

            for (GroundCoverObjects::const_iterator i = b->get()->getObjects().begin();
                i != b->get()->getObjects().end();
                ++i)
            {
                const GroundCoverBillboard* bb = static_cast<const GroundCoverBillboard*>(i->get());
                if (bb)
                {
                    unsigned weight = 1u;
                    BillboardLUTEntry entry;
                    if (bb->_symbol.valid())
                    {
                        entry.width = bb->_symbol->width().get();
                        entry.height = bb->_symbol->height().get();
                        entry.sizeVariation = bb->_symbol->sizeVariation().get();
                        weight = bb->_symbol->selectionWeight().get();
                    }
                    for(unsigned w=0; w<weight; ++w)
                    {
                        billboards.push_back(entry);
                    }               
                }
            }
        }
    }

    // custom featurelist cursor that lets us populate the list directly
    class MyFeatureListCursor : public FeatureListCursor
    {
    public:
        MyFeatureListCursor() : FeatureListCursor(FeatureList()) { }
        FeatureList& features() { return _features; }
    };
}

//...................................................................

GroundCoverFeatureGenerator::GroundCoverFeatureGenerator() :
    _status(Status::ConfigurationError)
{
    //nop
}

void
GroundCoverFeatureGenerator::setMap(const Map* map)
{
    _map = map;
    initialize();
}

void
GroundCoverFeatureGenerator::setFactory(TerrainTileModelFactory* value)
{
    _factory = value;
    initialize();
}

void
GroundCoverFeatureGenerator::setLayer(GroundCoverLayer* layer)
{
    _gclayer = layer;
    initialize();
}

const Status&
GroundCoverFeatureGenerator::getStatus() const
{
    return _status;
}

void
GroundCoverFeatureGenerator::initialize()
{
    if (!_map.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required Map");
        return;
    }

    if (_map->getProfile() == NULL)
    {
        _status.set(Status::ConfigurationError, "Map has no Profile set");
        return;
    }

    if (!_factory.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required TerrainTileModelFactory");
        return;
    }

    if (!_gclayer.valid())
    {
        _status.set(Status::ConfigurationError, "Missing required GroundCoverLayer");
        return;
    }

    // optional:
    _lclayer = _map->getLayer<LandCoverLayer>();

    // required IF _lclayer is found:
    _lcdict = _map->getLayer<LandCoverDictionary>();
    if (_lclayer.valid() && !_lcdict.valid())
    {
        _status.set(Status::ConfigurationError, "No LandCoverDictionary found in the map");
        return;
    }
    
    // optional:
    _masklayer = _gclayer->getMaskLayer();

    // open the landcover layer
    if (_lclayer.valid() && _lclayer->open().isError())
    {
        _status.set(_lclayer->getStatus().code(), Stringify() << "Opening landcover layer: " << _lclayer->getStatus().message());
        return;
    }

    // open the mask layer
    if (_masklayer.valid() && _masklayer->open().isError())
    {
        _status.set(_masklayer->getStatus().code(), Stringify() << "Opening mask layer: " << _masklayer->getStatus().message());
        return;
    }

    // open the groundcover layer
    if (_gclayer->open().isError())
    {
        _status.set(_gclayer->getStatus().code(), Stringify() << "Opening groundcover layer: " << _gclayer->getStatus().message());
        return;
    }

    // open the elevation layers
    ElevationLayerVector elevLayers;
    _map->getLayers(elevLayers);
    if (elevLayers.empty() == false)
    {
        for(ElevationLayerVector::iterator i = elevLayers.begin();
            i != elevLayers.end();
            ++i)
        {
            Status s = i->get()->open();
            if (s.isError())
            {
                _status.set(s.code(), Stringify() << "Opening elevation layer: " << s.message());
                return;
            }
        }
    }

    // create noise texture
    NoiseTextureFactory noise;
    _noiseTexture = noise.create(256u, 4u);

    // layers we're going to request
    if (_lclayer.valid()) 
        _manifest.insert(_lclayer.get());
    if (_masklayer.valid()) 
        _manifest.insert(_masklayer.get());
    if (_gclayer.valid()) 
        _manifest.insert(_gclayer.get());
    if (elevLayers.empty() == false)
        _manifest.insert(elevLayers.front().get()); // one is sufficient

    _status.set(Status::NoError);
}

void
GroundCoverFeatureGenerator::getFeatures(const TileKey& key, FeatureList& output) const
{
    osg::Vec4f landCover, mask, elev;

    // Populate the model, falling back on lower-LOD keys as necessary
    osg::ref_ptr<TerrainTileModel> model = _factory->createStandaloneTileModel(_map.get(), key, _manifest, NULL, NULL);
    if (!model.valid())
    {
        OE_INFO << LC << "createStandaloneTileModel returned NULL for " << key.str() << std::endl;
        return;
    }

    // for now, default to zone 0
    Zone* zone = _gclayer->getZones()[0].get();
    if (!zone)
    {
        OE_INFO << LC << "getZones returned NULL for " << key.str() << std::endl;
        return;
    }

    GroundCover* groundcover = zone->getGroundCover();
    if (!groundcover)
    {
        OE_INFO << LC << "getGroundCover returned NULL for " << key.str() << std::endl;
        return;
    }

    // noise sampler:
    ImageUtils::PixelReader sampleNoise;
    sampleNoise.setTexture(_noiseTexture.get());

    // mask texture/matrix:
    osg::Texture* maskTex = NULL;
    osg::Matrix maskMat;
    if (_masklayer.valid())
    {
        maskTex = model->getTexture(_masklayer->getUID());
        osg::RefMatrixf* r = model->getMatrix(_masklayer->getUID());
        if (r) maskMat = *r;
    }
    ImageUtils::PixelReader maskSampler;
    maskSampler.setTexture(maskTex);

    // landcover texture/matrix:
    osg::Texture* lcTex = NULL;
    osg::Matrixf lcMat;
    if (_lclayer.valid())
    {
        lcTex = model->getLandCoverTexture();
        if (!lcTex)
        {
            // TODO: how to handle this?
            OE_WARN << "No land cover texture for " << key.str() << std::endl;
            return;
        }
        osg::RefMatrixf* r = model->getLandCoverTextureMatrix();
        if (r) lcMat = *r;
    }
    ImageUtils::PixelReader lcSampler;
    lcSampler.setTexture(lcTex);

    // elevation
    osg::Texture* elevTex = NULL;
    osg::Matrix elevMat;
    if (model->elevationModel().valid())
    {
        elevTex = model->getElevationTexture();
        osg::RefMatrixf* r = model->getElevationTextureMatrix();
        if (r) elevMat = *r;
    }
    ImageUtils::PixelReader elevSampler;
    elevSampler.setTexture(elevTex);

    // because in the shader oe_terrain_getElevation adjusts the sampling
    // with scale coefficients:
    elevSampler.setSampleAsTexture(false);

    // build the lookup table for the biomes.
    // TODO: we could do this in initialize(), but we want to leave the door open
    // for supporting multiple zones in the future -GW
    BiomeLUT biomeLUT;
    buildLUT(groundcover, biomeLUT);

    // calculate instance count based on tile extents
    unsigned lod = _gclayer->getLOD();
    unsigned tx, ty;
    _map->getProfile()->getNumTiles(lod, tx, ty);
    GeoExtent e = TileKey(lod, tx / 2, ty / 2, _map->getProfile()).getExtent();
    GeoCircle c = e.computeBoundingGeoCircle();
    double tileWidth_m = 2.0 * c.getRadius() / 1.4142;
    float spacing_m = groundcover->getSpacing();
    unsigned vboTileSize = (unsigned)(tileWidth_m / spacing_m);

    // from here on out, we are mimicing the GroundCover.VS.glsl shader logic.
    int numInstancesX = vboTileSize;
    int numInstancesY = vboTileSize;
    unsigned totalNumInstances = numInstancesX * numInstancesY;

    osg::Vec2f offset, halfSpacing, tilec, shift;
    osg::Vec4f noise(0,0,0,0);

    for(unsigned instanceID = 0; instanceID < totalNumInstances; ++instanceID)
    {
        offset.set(
            (float)(instanceID % numInstancesX),
            (float)(instanceID / numInstancesY));

        halfSpacing.set(
            0.5f / (float)numInstancesX,
            0.5f / (float)numInstancesY);

        tilec.set(
            halfSpacing.x() + offset.x() / (float)numInstancesX,
            halfSpacing.y() + offset.y() / (float)numInstancesY);

        sampleNoise(noise, tilec.x(), tilec.y());

        shift.set(
            fract(noise[NOISE_RANDOM]*1.5)*2.0f - 1.0f,
            fract(noise[NOISE_RANDOM_2]*1.5)*2.0f - 1.0f);

        tilec.x() += shift.x()*halfSpacing.x();
        tilec.y() += shift.y()*halfSpacing.y();

        // check the land cover
        const GroundCoverBiome* biome = NULL;
        const LandCoverClass* lcclass = NULL;
        if (lcTex)
        {
            sample(landCover, lcSampler, lcMat, tilec.x(), tilec.y());
            lcclass = _lcdict->getClassByValue((int)landCover.r());
            if (lcclass == NULL)
                continue;
            biome = groundcover->getBiome(lcclass);
            if (!biome)
                continue;
        }

        // check the mask
        if (maskTex)
        {
            sample(mask, maskSampler, maskMat, tilec.x(), tilec.y());
            if (mask.r() > 0.0)
                continue;
        }

        // check the fill
        float fill =
            biome && biome->fill().isSet() ? biome->fill().get() :
            groundcover->options().fill().get();

        if (noise[NOISE_SMOOTH] > fill)
            continue;
        else
            noise[NOISE_SMOOTH] /= fill;

        // clamp
        float z = 0.0;
        if (elevTex)
        {
            sample(elev, elevSampler, elevMat, tilec.x(), tilec.y());
            //if (fabs(elev.r()) > 30000.0f)
            //{
            //    OE_WARN << "That's a problem.. elevation value is " << elev.r() << std::endl;
            //    osg::Vec4f val;
            //    sample(val, elevSampler, elevMat, tilec.x(), tilec.y());
            //    OE_WARN << "val.r() = " << val.r() << std::endl;
            //    
            //}
            if (elev.r() != NO_DATA_VALUE)
            {
                z = elev.r();
            }
        }

        // keeper
        Point* point = new Point();

        point->push_back(osg::Vec3d(
            key.getExtent().xMin() + tilec.x()*key.getExtent().width(),
            key.getExtent().yMin() + tilec.y()*key.getExtent().height(),
            0.0));

        osg::ref_ptr<Feature> feature = new Feature(point, key.getExtent().getSRS());
        feature->set("elevation", z);

        // Resolve the symbol so we can add attributes
        if (biome)
        {
            BillboardLUT& bblut = biomeLUT[biome];
            unsigned index = (unsigned)clamp(noise[NOISE_RANDOM], 0.0, 0.9999999) * (float)(bblut.size());
            BillboardLUTEntry& bb = bblut[index];
            float sizeScale = bb.sizeVariation * (noise[NOISE_RANDOM_2] * 2.0 - 1.0);
            float width = bb.width + bb.width*sizeScale;
            float height = bb.height + bb.height*sizeScale;
            feature->set("width", width);
            feature->set("height", height);
        }

        output.push_back(feature.get());
    }
}
