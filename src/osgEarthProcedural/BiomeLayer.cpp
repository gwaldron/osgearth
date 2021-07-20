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
#include "Random"
#include <osgEarth/rtree.h>

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(biomes, BiomeLayer);

#define LC "[BiomeLayer] "

//...................................................................

void
BiomeLayer::Options::fromConfig(const Config& conf)
{
    blendRadius().setDefault(0.02);
    biomeidField().setDefault("biomeid");

    biomeCatalog() = std::make_shared<BiomeCatalog>(conf.child("biomecatalog"));
    controlVectors().get(conf, "control_vectors");
    conf.get("blend_radius", blendRadius());
    conf.get("biomeid_field", biomeidField());
}

Config
BiomeLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    OE_DEBUG << LC << __func__ << " not yet implemented" << std::endl;
    controlVectors().set(conf, "control_vectors");
    //TODO - biomeCatalog
    conf.set("blend_radius", blendRadius());
    conf.set("biomeid_field", biomeidField());
    return conf;
}

//...................................................................

#undef LC
#define LC "[BiomeLayer] " << getName() << ": "

namespace
{
    struct Record
    {
        Segment2d _segment;
        int _biomeid;
        double _radius;
    };

    typedef std::shared_ptr<Record> RecordPtr;

    typedef RTree<RecordPtr, double, 2> MySpatialIndex;
    
    struct BiomeTrackerToken : public osg::Object
    {
        META_Object(osgEarth, BiomeTrackerToken);
        BiomeTrackerToken() { }
        BiomeTrackerToken(std::set<int>&& seen) : _biomeids(seen) { }
        BiomeTrackerToken(const BiomeTrackerToken& rhs, const osg::CopyOp& op) { }
        std::set<int> _biomeids;
    };
}

void
BiomeLayer::init()
{
    ImageLayer::init();

    // BiomeLayer is invisible AND shared by default.
    options().visible().setDefault(false);
    options().shared().setDefault(true);

    _index = nullptr;
    setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
}

Status
BiomeLayer::openImplementation()
{
    Status p = ImageLayer::openImplementation();
    if (p.isError())
        return p;

    Status csStatus = options().controlVectors().open(getReadOptions());
    if (csStatus.isError())
        return csStatus;

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
    if (_index)
        delete static_cast<MySpatialIndex*>(_index);
    _index = nullptr;

    options().controlVectors().close();

    return ImageLayer::closeImplementation();
}

void
BiomeLayer::addedToMap(const Map* map)
{
    options().controlVectors().addedToMap(map);

    if (!getControlSet())
    {
        setStatus(Status::ConfigurationError, "No control set found");
        return;
    }

    MySpatialIndex* index = new MySpatialIndex();
    _index = index;

    const std::string& biomeid_field = options().biomeidField().get();

    OE_INFO << LC << "Loading control set..." << std::endl;

    // Populate the in-memory spatial index with all the control points
    int count = 0;
    osg::ref_ptr<FeatureCursor> cursor = getControlSet()->createFeatureCursor(Query(), nullptr);
    if (cursor.valid())
    {
        cursor->fill(_features);

        for (auto& feature : _features)
        {
            int biomeid = feature->getInt(biomeid_field);
            double radius = feature->getDouble("radius", 0.0);

            const Geometry* g = feature->getGeometry();

            if (g->isPointSet())
            {
                ConstGeometryIterator iter(g);
                while (iter.hasMore())
                {
                    const Geometry* part = iter.next();
                    for (auto& p : *part)
                    {
                        double a_m[2] = { p.x(), p.y() };
                        index->Insert(a_m, a_m, RecordPtr(new Record{ Segment2d(p, p), biomeid, radius }));
                    }
                }
            }
            else
            {
                ConstGeometryIterator g_iter(g);
                while (g_iter.hasMore())
                {
                    ConstSegmentIterator s_iter(g_iter.next());
                    while (s_iter.hasMore())
                    {
                        Segment s = s_iter.next();
                        double a_min[2] = { std::min(s.first.x(), s.second.x()), std::min(s.first.y(), s.second.y()) };
                        double a_max[2] = { std::max(s.first.x(), s.second.x()), std::max(s.first.y(), s.second.y()) };
                        index->Insert(a_min, a_max, RecordPtr(new Record{ Segment2d(s.first, s.second), biomeid, radius }));
                    }
                }
            }
        }
    }
    OE_INFO << LC << "Loaded control set and found " << count << " features" << std::endl;
}

FeatureSource*
BiomeLayer::getControlSet() const
{
    return options().controlVectors().getLayer();
}

std::shared_ptr<const BiomeCatalog>
BiomeLayer::getBiomeCatalog() const
{
    return options().biomeCatalog();
}

const Biome*
BiomeLayer::getBiome(int id) const
{
    const auto cat = getBiomeCatalog();
    if (cat)
        return cat->getBiome(id);
    else
        return nullptr;
}

GeoImage
BiomeLayer::createImageImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    MySpatialIndex* index = static_cast<MySpatialIndex*>(_index);
    if (index == nullptr)
        return GeoImage::INVALID;

    // allocate an 8-bit image:
    osg::ref_ptr<osg::Image> image = new osg::Image();
    image->allocateImage(
        getTileSize(),
        getTileSize(),
        1,
        GL_RED,
        GL_UNSIGNED_BYTE);

    ImageUtils::PixelWriter write(image.get());

    osg::Vec4 value;
    float noise = 1.0f;
    std::vector<RecordPtr> hits;
    std::vector<double> ranges_squared;
    Random prng(key.hash());
    double radius = options().blendRadius().get();
    std::set<int> biomeids_seen;

    GeoImageIterator iter(GeoImage(image.get(), key.getExtent()));

    iter.forEachPixelOnCenter([&]()
        {
            int biomeid = 0;

            // randomly permute the coordinates in order to blend across biomes
            double x = iter.x() + radius * (prng.next()*2.0 - 1.0);
            double y = iter.y() + radius * (prng.next()*2.0 - 1.0);

            // find the closest biome vector to the point:
            index->KNNSearch(
                osg::Vec3d(x,y,0).ptr(),
                &hits,
                nullptr,
                1u,
                0.0);

            if (hits.size() > 0)
            {
                biomeid = hits[0]->_biomeid;

                if (biomeid > 0)
                    biomeids_seen.insert(biomeid);
            }

            value.r() = (float)biomeid / 255.0f;

            write(value, iter.s(), iter.t());
        });

    GeoImage result(image.get(), key.getExtent());
    
    trackImage(result, key, biomeids_seen);

    return std::move(result);
}

void
BiomeLayer::postCreateImageImplementation(
    GeoImage& createdImage,
    const TileKey& key,
    ProgressCallback* progress) const
{
    if (createdImage.getTrackingToken() == nullptr)
    {
        // if there's no tracking token (e.g., this image came from the cache)
        // build and attach one now.
        GeoImageIterator iter(createdImage);
        ImageUtils::PixelReader read(createdImage.getImage());

        std::set<int> biomeids_seen;
        osg::Vec4 pixel;

        iter.forEachPixel([&]()
            {
                int biomeid = 0;
                read(pixel, iter.s(), iter.t());
                int biome = (int)(pixel.r()*255.0f);
                biomeids_seen.insert(biome);
            });

        trackImage(createdImage, key, biomeids_seen);
    }
}

void
BiomeLayer::trackImage(
    GeoImage& image,
    const TileKey& key,
    std::set<int>& biomeids) const
{
    // inform the biome manager that we are using the biomes corresponding
    // to the biome ID's we collected
    for (auto biomeid : biomeids)
    {
        const Biome* biome = getBiomeCatalog()->getBiome(biomeid);
        if (biome)
            const_cast<BiomeManager*>(&_biomeMan)->ref(biome);
    }

    // Create a "token" object that we can track for destruction.
    // This will inform us when the image created by this call
    // destructs, and we can unref the usage in the BiomeManager accordingly.
    // This works, but reverses the flow of control, so maybe
    // there is a better solution -gw
    osg::Object* token = new BiomeTrackerToken(std::move(biomeids));
    token->setName(Stringify() << "BiomeLayer " << key.str());
    image.setTrackingToken(token);
    token->addObserver(const_cast<BiomeLayer*>(this));
    _tracker.scoped_lock([&]() { _tracker[token] = key; });
}

void
BiomeLayer::objectDeleted(void* value)
{
    BiomeTrackerToken* token = static_cast<BiomeTrackerToken*>(value);

    _tracker.scoped_lock([&]() 
        {
            OE_DEBUG << LC << "Unloaded " << token->getName() << std::endl;
            for (auto biomeid : token->_biomeids)
            {
                const Biome* biome = getBiomeCatalog()->getBiome(biomeid);
                if (biome)
                    _biomeMan.unref(biome);
            }
            _tracker.erase(token);
        });
}
