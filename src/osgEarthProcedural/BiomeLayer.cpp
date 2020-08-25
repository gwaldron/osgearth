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

using namespace osgEarth;
using namespace osgEarth::Procedural;

REGISTER_OSGEARTH_LAYER(biomes, BiomeLayer);

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
    //TODO
    return conf;
}

//...................................................................

#undef LC
#define LC "[BiomeLayer] " << getName() << ": "

void
BiomeLayer::init()
{
    Layer::init();
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

    osg::ref_ptr<FeatureCursor> cursor = getControlSet()->createFeatureCursor(Query(), nullptr);
    while (cursor.valid() && cursor->hasMore())
    {
        const Feature* f = cursor->nextFeature();
        if (f)
        {
            _index.emplace_back(ControlPoint());
            _index.back().x = f->getGeometry()->begin()->x();
            _index.back().y = f->getGeometry()->begin()->y();
            _index.back().biomeid = f->getInt("biomeid");
        }
    }
    OE_INFO << LC << "Loaded control set and found " << _index.size() << " features" << std::endl;

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
    _index.clear();

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

    double farthest2 = 0.0;
    std::vector<SearchResult>::iterator farthest2_iter;

    for (const auto& cp : _index)
    {
        double range2 = (cp.x - x)*(cp.x - x) + (cp.y - y)*(cp.y - y);

        if (results.size() < maxCount)
        {
            SearchResult sr;
            sr.biomeid = cp.biomeid;
            sr.range2 = range2;
            results.insert(sr);
        }
        else
        {
            if (range2 < results.rbegin()->range2)
            {
                SearchResult sr;
                sr.biomeid = cp.biomeid;
                sr.range2 = range2;
                results.insert(sr);
                results.erase(std::prev(results.end()));
            }
        }
    }
}
