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
#include "BiomeLayer"
#include <osgEarth/Random>
#include <osgEarth/MetaTile>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(biomes, BiomeLayer);

#define LC "[BiomeLayer] "

//...................................................................

void
BiomeLayer::Options::fromConfig(const Config& conf)
{
    blendRadius().setDefault(0.0f);

    biomeCatalog() = std::make_shared<BiomeCatalog>(conf.child("biomecatalog"));
    landCoverLayer().get(conf, "landcover_layer");
    biomeBaseLayer().get(conf, "biome_base_layer");
    conf.get("blend_radius", blendRadius());
}

Config
BiomeLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    OE_DEBUG << LC << __func__ << " not yet implemented" << std::endl;
    landCoverLayer().set(conf, "landcover_layer");
    biomeBaseLayer().set(conf, "biome_base_layer");
    //TODO - biomeCatalog
    conf.set("blend_radius", blendRadius());
    return conf;
}

//...................................................................

#undef LC
#define LC "[BiomeLayer] " << getName() << ": "

namespace
{
    // Token for keeping track of biome images so we can page their
    // asset data in and out
    struct BiomeTrackerToken : public osg::Object
    {
        META_Object(osgEarth, BiomeTrackerToken);
        BiomeTrackerToken() { }
        BiomeTrackerToken(std::set<int>&& seen) : _biome_index_set(seen) { }
        BiomeTrackerToken(const BiomeTrackerToken& rhs, const osg::CopyOp& op) { }
        std::set<int> _biome_index_set;
    };
}

void
BiomeLayer::init()
{
    ImageLayer::init();

    // BiomeLayer is invisible AND shared by default.
    options().visible().setDefault(false);
    options().shared().setDefault(true);
    options().minFilter().setDefault(osg::Texture::LINEAR);
    options().magFilter().setDefault(osg::Texture::NEAREST);
    options().textureCompression().setDefault("none");

    _autoBiomeManagement = true;

    _tracker.setName("BiomeLayer.tracker(OE)");

    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
}

Status
BiomeLayer::openImplementation()
{
    Status p = ImageLayer::openImplementation();
    if (p.isError())
        return p;

    Status csStatus = options().biomeBaseLayer().open(getReadOptions());
    if (csStatus.isError())
        return csStatus;

    options().landCoverLayer().open(getReadOptions());

    // Warn the poor user if the configuration is missing
    if (getBiomeCatalog() == nullptr)
    {
        OE_WARN << LC << "No biome catalog found - could be trouble" << std::endl;
    }
    else if (getBiomeCatalog()->getAssets().empty())
    {
        OE_WARN << LC << "No asset catalog found - could be trouble" << std::endl;
    }

    

    return Status::OK();
}

Status
BiomeLayer::closeImplementation()
{
    options().landCoverLayer().close();
    options().biomeBaseLayer().close();

    // cache:
    {
        ScopedMutexLock lock(_imageCache);
        _imageCache.clear();
    }

    return ImageLayer::closeImplementation();
}

void
BiomeLayer::addedToMap(const Map* map)
{
    options().landCoverLayer().addedToMap(map);
    options().biomeBaseLayer().addedToMap(map);

    if (!getLandCoverLayer() && !getBiomeBaseLayer())
    {
        setStatus(Status::ResourceUnavailable, "No source data available");
        return;
    }

    // Initialize the biome creator
    if (getBiomeBaseLayer() && getBiomeBaseLayer()->isOpen())
    {
        _biomeCreator = std::unique_ptr< CoverageLayer::CoverageCreator<BiomeSample> >(
            new CoverageLayer::CoverageCreator<BiomeSample>(getBiomeBaseLayer())
            );
    }

    // Initialize the landcover creator
    if (getLandCoverLayer() && getLandCoverLayer()->isOpen())
    {
        _landCoverCreator = std::unique_ptr< CoverageLayer::CoverageCreator<LandCoverSample> >(
            new CoverageLayer::CoverageCreator<LandCoverSample>(getLandCoverLayer())
            );
    }
}

void
BiomeLayer::removedFromMap(const Map* map)
{
    options().landCoverLayer().removedFromMap(map);
    options().biomeBaseLayer().removedFromMap(map);
}

CoverageLayer*
BiomeLayer::getLandCoverLayer() const
{
    return options().landCoverLayer().getLayer();
}

CoverageLayer*
BiomeLayer::getBiomeBaseLayer() const
{
    return options().biomeBaseLayer().getLayer();
}

std::shared_ptr<const BiomeCatalog>
BiomeLayer::getBiomeCatalog() const
{
    return options().biomeCatalog();
}

void
BiomeLayer::setAutoBiomeManagement(bool value)
{
    _autoBiomeManagement = value;

    if (_autoBiomeManagement == false)
    {
        _biomeMan.reset();
    }
}

bool
BiomeLayer::getAutoBiomeManagement() const
{
    return _autoBiomeManagement;
}

GeoImage
BiomeLayer::createImageImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    OE_PROFILING_ZONE;

    if (getBiomeCatalog() == nullptr)
    {
        return GeoImage::INVALID;
    }

    // check the cache:
    {
        ScopedMutexLock lock(_imageCache);
        auto iter = _imageCache.find(key);
        osg::ref_ptr<osg::Image> image;
        if (iter != _imageCache.end() && iter->second.lock(image))
        {
            return GeoImage(image.get(), key.getExtent());
        }
    }

    // allocate a 16-bit image so we can represent 32K biomes
    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(
        getTileSize(),
        getTileSize(),
        1,
        GL_RED,
        GL_FLOAT);

    image->setInternalTextureFormat(GL_R16F);


    ImageUtils::PixelWriter write(image.get());
    osg::Vec4 value;
    float noise = 1.0f;

    Random prng(key.hash());
    double radius = options().blendRadius().get();
    std::set<int> biome_indices_seen;
    const GeoExtent& ex = key.getExtent();
    GeoImage temp(image.get(), ex);
    GeoImageIterator iter(temp);
    std::unordered_set<std::string> missing_biomes;

    

    // Use meta-tiling to read coverage data with access to the 
    // neighboring tiles - to support the blend radius.
    MetaTile<GeoCoverage<LandCoverSample>> landcover;    

    if (_landCoverCreator)
    {
        landcover.setCreateTileFunction(
            [&](const TileKey& key, ProgressCallback* p) -> GeoCoverage<LandCoverSample>
            {
                return _landCoverCreator->createCoverage(key, p);
            });

        landcover.setCenterTileKey(key, progress);
    }

    // Use meta-tiling to read biome coverage data with access to the 
    // neighboring tiles
    MetaTile<GeoCoverage<BiomeSample>> biomeMetaTile;

    if (_biomeCreator)
    {
        biomeMetaTile.setCreateTileFunction(
            [&](const TileKey& key, ProgressCallback* p) -> GeoCoverage<BiomeSample>
            {            
                return _biomeCreator->createCoverage(key, p);
            });

        biomeMetaTile.setCenterTileKey(key, progress);
    }

    iter.forEachPixelOnCenter([&]()
        {
            int biome_index = 0;
            std::string traits;

            double x = iter.x();
            double y = iter.y();

            // randomly permute the coordinates in order to blend across biomes
            if (radius > temp.getUnitsPerPixel())
            {
                x += radius * (prng.next() * 2.0 - 1.0);
                y += radius * (prng.next() * 2.0 - 1.0);
            }

            // convert the x,y to u,v
            double u = (x - ex.xMin()) / ex.width();
            double v = (y - ex.yMin()) / ex.height();

            // First try the biome base layer
            if (biomeMetaTile.valid())
            {
                const BiomeSample* sample = biomeMetaTile.read(u, v);

                if (sample)
                {
                    if (sample->biomeid().isSet())
                    {
                        const Biome* biome = getBiomeCatalog()->getBiome(sample->biomeid().get());
                        if (biome)
                        {
                            biome_index = biome->index();
                        }
                    }
                }
            }

            // Next try the landcover layer.
            if (landcover.valid())
            {
                const LandCoverSample* sample = landcover.read(u, v);
                if (sample)
                {
                    if (sample->biomeid().isSet())
                    {
                        const Biome* biome = getBiomeCatalog()->getBiome(sample->biomeid().get());
                        if (biome)
                        {
                            biome_index = biome->index();
                        }
                    }
                    else if (sample->traits().isSet())
                    {                     
                        traits = sample->traits().get();
                    }

                    // NB: lifemap values are handled by the LifeMapLayer (ignored here)
                }
            }

            

            // if we found a valid one, insert it into the set
            if (biome_index > 0)
            {
                biome_indices_seen.insert(biome_index);
            }

            // write it to the raster
            value.r() = (float)biome_index;
            write(value, iter.s(), iter.t());
        });

    GeoImage result(image.get(), key.getExtent());

    // Set up tracking on all discovered biomes in this image. 
    // This will allow us to page out when all references to a biome
    // expire from the scene
    trackImage(result, key, biome_indices_seen);

    // report any errors
    if (missing_biomes.empty() == false)
    {
        std::ostringstream buf;
        buf << "Undefined biomes detected: ";
        for (auto i : missing_biomes)
            buf << i << ' ';
        OE_WARN << LC << buf.str() << std::endl;
    }

    // cache:
    {
        ScopedMutexLock lock(_imageCache);
        _imageCache[key] = image.get();
    }

    return result;

    return GeoImage::INVALID;
}

void
BiomeLayer::postCreateImageImplementation(
    GeoImage& createdImage,
    const TileKey& key,
    ProgressCallback* progress) const
{
    // (This runs post-caching)
    // When a new biome raster arrives, scan it to find a set of all
    // biome indices that it contains, and register this set with the
    // tracker.
    if (getAutoBiomeManagement() &&
        createdImage.getTrackingToken() == nullptr)
    {
        // if there's no tracking token (e.g., this image came from the cache)
        // build and attach one now.
        GeoImageIterator iter(createdImage);
        ImageUtils::PixelReader read(createdImage.getImage());

        std::set<int> biome_indices_seen;
        osg::Vec4 pixel;

        // known format (GL_RED/GL_FLOAT) - traverse manually for speed
        const osg::Image* image = createdImage.getImage();
        OE_IF_SOFT_ASSERT(image && image->getPixelFormat() == GL_RED && image->getDataType() == GL_FLOAT)
        {
            unsigned size = image->s() * image->t();
            const float* ptr = (const float*)(image->data());
            int biome_index;

            for (unsigned i = 0; i < size; ++i)
            {
                biome_index = (int)(*ptr++);
                if (biome_index > 0)
                    biome_indices_seen.insert(biome_index);
            }

            trackImage(createdImage, key, biome_indices_seen);
        }
    }
}

void
BiomeLayer::trackImage(
    GeoImage& image,
    const TileKey& key,
    std::set<int>& biome_index_set) const
{
    // inform the biome manager that we are using the biomes corresponding
    // to the set of biome indices collected from the raster.
    for (auto biome_index : biome_index_set)
    {
        const Biome* biome = getBiomeCatalog()->getBiomeByIndex(biome_index);
        if (biome)
            const_cast<BiomeManager*>(&_biomeMan)->ref(biome);
    }

    // Create a "token" object that we can track for destruction.
    // This will inform us when the image created by this call
    // destructs, and we can unref the usage in the BiomeManager accordingly.
    // This works, but reverses the flow of control, so maybe
    // there is a better solution -gw
    osg::Object* token = new BiomeTrackerToken(std::move(biome_index_set));
    token->setName(Stringify() << "BiomeLayer " << key.str());
    image.setTrackingToken(token);
    token->addObserver(const_cast<BiomeLayer*>(this));
    _tracker.scoped_lock([&]() { _tracker[token] = key; });
}

void
BiomeLayer::objectDeleted(void* value)
{
    // Invoked when the biome index raster destructs.
    // Inform the BiomeManager that the indices referenced by this
    // image are no longer in use (unreference the count).
    BiomeTrackerToken* token = static_cast<BiomeTrackerToken*>(value);

    _tracker.scoped_lock([&]()
        {
            OE_DEBUG << LC << "Unloaded " << token->getName() << std::endl;

            if (getAutoBiomeManagement())
            {
                for (auto biome_index : token->_biome_index_set)
                {
                    const Biome* biome = getBiomeCatalog()->getBiomeByIndex(biome_index);
                    if (biome)
                    {
                        _biomeMan.unref(biome);
                    }
                }
            }
            _tracker.erase(token);
        });
}
