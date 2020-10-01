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
#include <osgEarth/rtree.h>

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(biomes, BiomeLayer);

#define LC "[BiomeLayer] "

//...................................................................

void
BiomeLayer::Options::fromConfig(const Config& conf)
{
    biomeCatalog() = new BiomeCatalog(conf.child("biomecatalog"));
    controlVectors().get(conf, "control_vectors");
}

Config
BiomeLayer::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();
    OE_DEBUG << LC << __func__ << " not yet implemented" << std::endl;
    //TODO
    return conf;
}

//...................................................................

#undef LC
#define LC "[BiomeLayer] " << getName() << ": "

namespace
{
    typedef int BiomeID;
    typedef RTree<BiomeID, double, 2> MySpatialIndex;
}

void
BiomeLayer::init()
{
    Layer::init();
    _index = nullptr;
}

Status
BiomeLayer::openImplementation()
{
    Status p = Layer::openImplementation();
    if (p.isError())
        return p;

    Status csStatus = options().controlVectors().open(getReadOptions());
    if (csStatus.isError())
        return csStatus;

    MySpatialIndex* index = new MySpatialIndex();
    _index = index;

    osg::ref_ptr<FeatureCursor> cursor = getControlSet()->createFeatureCursor(Query(), nullptr);
    while (cursor.valid() && cursor->hasMore())
    {
        const Feature* f = cursor->nextFeature();
        if (f)
        {
            double p[2] = { f->getGeometry()->begin()->x(), f->getGeometry()->begin()->y() };
            index->Insert(p, p, { (int)f->getInt("biomeid") });
        }
    }
    OE_INFO << LC << "Loaded control set" << std::endl; // and found " << _index->size() << " features" << std::endl;

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

    return Layer::closeImplementation();
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
    std::set<SearchResult>& results) const
{
    results.clear();

    //TODO: replace this brute-force search with a proper
    // spatial index.

    MySpatialIndex* index = static_cast<MySpatialIndex*>(_index);
    if (index == nullptr)
        return;

    std::vector<BiomeID> hits;
    std::vector<double> ranges_squared;
    double point[2] = { x, y };
    
    index->KNNSearch(
        point,
        &hits,
        &ranges_squared,
        maxCount,
        0.0);

    for (int i = 0; i < hits.size(); ++i)
    {
        results.emplace(SearchResult{
            hits[i],
            ranges_squared[i] });
    }
}
