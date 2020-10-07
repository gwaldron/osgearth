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
    numSamples().setDefault(1u);
    tightness().setDefault(9.0f);

    biomeCatalog() = new BiomeCatalog(conf.child("biomecatalog"));
    controlVectors().get(conf, "control_vectors");
    conf.get("num_samples", numSamples());
    conf.get("tightness", tightness());
}

Config
BiomeLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    OE_DEBUG << LC << __func__ << " not yet implemented" << std::endl;
    //TODO
    conf.set("num_samples", numSamples());
    conf.set("tightness", tightness());
    return conf;
}

//...................................................................

#undef LC
#define LC "[BiomeLayer] " << getName() << ": "

namespace
{
    typedef RTree<BiomeLayer::RecordPtr, double, 2> MySpatialIndex;
}

void
BiomeLayer::init()
{
    ImageLayer::init();
    _index = nullptr;
    setProfile(Profile::create("global-geodetic"));
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

    MySpatialIndex* index = new MySpatialIndex();
    _index = index;

    // Populate the in-memory spatial index with all the control points
    int count = 0;
    osg::ref_ptr<FeatureCursor> cursor = getControlSet()->createFeatureCursor(Query(), nullptr);
    if (cursor.valid())
    {
        cursor->fill(_features);

        for (auto& feature : _features)
        {
            int biomeid = feature->getInt("wwf_mhtnum");
            //int biomeid = feature->getInt("biomeid");
            double radius = feature->getDouble("radius", 0.0);

            const Geometry* g = feature->getGeometry();

            if (true) //g->isPointSet())
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

    // Warn the poor user if the configuration is missing
    if (getBiomeCatalog() == nullptr)
    {
        OE_WARN << LC << "No biome catalog found - could be trouble" << std::endl;
    }
    else if (getBiomeCatalog()->getAssets() == nullptr)
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

FeatureSource*
BiomeLayer::getControlSet() const
{
    return options().controlVectors().getLayer();
}

const BiomeCatalog*
BiomeLayer::getBiomeCatalog() const
{
    return options().biomeCatalog().get();
}

const Biome*
BiomeLayer::getBiome(int id) const
{
    const BiomeCatalog* cat = getBiomeCatalog();
    if (cat)
        return cat->getBiome(id);
    else
        return nullptr;
}

void
BiomeLayer::getNearestBiomes(
    double x,
    double y,
    unsigned maxCount,
    std::vector<RecordPtr>& hits,
    std::vector<double>& ranges_squared) const
{
    hits.clear();
    ranges_squared.clear();

    MySpatialIndex* index = static_cast<MySpatialIndex*>(_index);
    if (index == nullptr)
        return;

    osg::Vec3d point(x, y, 0);

    std::unordered_set<int> unique_biomeids;

    std::function<bool(const RecordPtr& rec)> acceptor = [unique_biomeids] (const RecordPtr& rec) mutable {
        if (unique_biomeids.find(rec->_biomeid) == unique_biomeids.end()) {
            unique_biomeids.insert(rec->_biomeid);
            return true;
        }
        return false;
    };
    
    index->KNNSearch(
        point.ptr(),
        &hits,
        &ranges_squared,
        maxCount,
        0.0,
        acceptor);

    //TODO: still need to look at the results and see which N are closest...?

#if 0
    for (int i = 0; i < hits.size(); ++i)
    {
        double precise_range_squared = hits[i]->_segment.squaredDistanceTo(point);

        if (hits[i]->_radius <= 0.0 || precise_range_squared < hits[0]->_radius)
        {
            results.emplace(SearchResult{
                hits[i]->_biomeid,
                precise_range_squared });
        }
    }
#endif
}

GeoImage
BiomeLayer::createImageImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
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
    unsigned samples = options().numSamples().get();
    float tightness = options().tightness().get();
    std::vector<RecordPtr> hits;
    std::vector<double> ranges_squared;
    Random prng(key.hash());

    GeoImageIterator iter(image.get(), key.getExtent());

    iter.forEachPixel([&]()
        {
            int biomeid = 0;

            getNearestBiomes(iter.x(), iter.y(), samples, hits, ranges_squared);

            if (hits.size() > 0)
            {
                if (hits.size() > 1)
                {
                    double total = 0.0;
                    for (int i = 0; i < ranges_squared.size(); ++i)
                    {
                        total += pow(1.0 / ranges_squared[i], tightness);
                    }

                    double runningTotal = 0.0;
                    double pick = total * (double)prng.next();
                    for (int i = 0; i < hits.size(); ++i)
                    {
                        runningTotal += pow(1.0 / ranges_squared[i], tightness);
                        if (pick < runningTotal)
                        {
                            biomeid = hits[i]->_biomeid;
                            break;
                        }
                    }
                }

                // return the last one by default
                if (biomeid == 0)
                {
                    biomeid = hits[hits.size() - 1]->_biomeid;
                }
            }

            value.r() = (float)biomeid / 255.0f;

            write(value, iter.s(), iter.t());
        });

    return GeoImage(image.get(), key.getExtent());
}
