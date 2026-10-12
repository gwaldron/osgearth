/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#include <osgEarth/RiverLayer>
#include "RiverHydrology.h"
#include "RiverRendering.h"
#include <osgEarth/Map>
#include <osgEarth/ElevationPool>
#include <osgEarth/Shaders>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/TerrainResources>
#include <osgEarth/CameraUtils>
#include <osgEarth/Utils>
#include <osgEarth/Threading>
#include <osgEarth/JsonUtils>
#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osg/Depth>
#include <osg/BlendFunc>
#include <osg/Geometry>
#include <osgUtil/CullVisitor>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <fstream>
#include <sstream>
#include <climits>
#include <list>
#include <set>
#include <atomic>
#include <iomanip>

#define LC "[RiverLayer] \"" << getName() << "\" "

using namespace osgEarth;
using namespace osgEarth::RiverDetail;
using namespace osgEarth::Threading;

REGISTER_OSGEARTH_LAYER(river, RiverLayer);
REGISTER_OSGEARTH_LAYER(rivers, RiverLayer);
REGISTER_OSGEARTH_LAYER(rivercarve, RiverCarveLayer);
REGISTER_OSGEARTH_LAYER(river_carve, RiverCarveLayer);
REGISTER_OSGEARTH_LAYER(riverconstraint, RiverConstraintLayer);
REGISTER_OSGEARTH_LAYER(river_constraint, RiverConstraintLayer);

namespace
{
    // Render order: opaque terrain draws in bin 0, Prestige vegetation in 3 and its sky
    // background in 5; transparent geometry starts at 10. The scene copy goes right
    // before the water so both see everything opaque; the spray blends over the water.
    const int SCENE_COPY_BIN = 8;
    const int WATER_BIN = 9;
    const int SPRAY_BIN = 10;

    // Water within this many meters of the tile data is gathered to build a tile's hydrology.
    const double CONTEXT_BUFFER = 300.0;

    // User waterway IDs start here, well above OpenStreetMap way IDs.
    const FeatureID FIRST_USER_FID = (FeatureID)1 << 50;

    //! Default physical parameters for the OSM waterway classes.
    std::map<std::string, RiverLayer::WaterwayClass> defaultWaterways()
    {
        std::map<std::string, RiverLayer::WaterwayClass> classes;
        auto add = [&](const char* type, float width, float depth, float speed, float rapids)
        {
            RiverLayer::WaterwayClass c;
            c.width = width, c.depth = depth, c.speed = speed, c.rapids = rapids;
            classes[type] = c;
        };
        add("river", 20.0f, 2.0f, 1.0f, 1.0f);
        add("stream", 3.5f, 0.5f, 0.75f, 0.8f);
        add("canal", 12.0f, 2.0f, 0.25f, 0.0f);
        add("brook", 2.0f, 0.35f, 0.6f, 0.7f);
        add("ditch", 1.5f, 0.4f, 0.25f, 0.1f);
        add("drain", 2.5f, 0.6f, 0.35f, 0.1f);
        add("tidal_channel", 10.0f, 1.5f, 0.25f, 0.0f);
        return classes;
    }

    //! Linear-light version of an sRGB color (for colors authored in the GUI).
    osg::Vec3f toLinear(const Color& c)
    {
        return osg::Vec3f(std::pow(c.r(), 2.2f), std::pow(c.g(), 2.2f), std::pow(c.b(), 2.2f));
    }

    //! Spherical mercator coordinates of a geographic point.
    osg::Vec2d geoToMerc(double lon, double lat)
    {
        const double R = 6378137.0;
        lat = std::min(std::max(lat, -85.05), 85.05);
        return osg::Vec2d(R * osg::DegreesToRadians(lon), R * std::log(std::tan(osg::PI_4 + osg::DegreesToRadians(lat) * 0.5)));
    }

    //! Real meters per mercator meter at a mercator y.
    double mercScale(double y) { return 1.0 / std::cosh(y / 6378137.0); }

    //! Repeats per 1024 m texture period nearest to a target repeat length: integer counts
    //! keep the tiling seamless across tiles.
    float repeatFrequency(double period, double scale)
    {
        double n = std::max(1.0, std::floor(1024.0 / (period * std::max(scale, 0.05)) + 0.5));
        return (float)(n / 1024.0);
    }

    //! Text of a color as #RRGGBBAA.
    std::string colorString(const Color& c) { return c.toHTML(); }

    //! Number of triangles in a built tile node.
    unsigned countTriangles(osg::Node* node)
    {
        unsigned count = 0u;
        if (!node) return 0u;
        osg::Group* group = node->asGroup();
        if (group)
        {
            for (unsigned i = 0; i < group->getNumChildren(); ++i)
                count += countTriangles(group->getChild(i));
        }
        osg::Geometry* geom = node->asGeometry();
        if (geom)
        {
            for (unsigned i = 0; i < geom->getNumPrimitiveSets(); ++i)
                count += geom->getPrimitiveSet(i)->getNumIndices() / 3u;
        }
        return count;
    }

    //! Number of spray particles (two triangles each) in a built tile node.
    unsigned countParticles(osg::Node* node)
    {
        unsigned count = 0u;
        if (!node) return 0u;
        osg::Group* group = node->asGroup();
        if (group)
        {
            for (unsigned i = 0; i < group->getNumChildren(); ++i)
                count += countParticles(group->getChild(i));
        }
        osg::Geometry* geom = node->asGeometry();
        if (geom && geom->getName() == "River spray")
        {
            for (unsigned i = 0; i < geom->getNumPrimitiveSets(); ++i)
                count += geom->getPrimitiveSet(i)->getNumIndices() / 6u;
        }
        return count;
    }
}

//...................................................................
// Options

Config
RiverLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    features().set(conf, "features");
    conf.set("level", level());
    conf.set("max_range", maxRange());
    conf.set("mesh_resolution", meshResolution());
    conf.set("level_smoothing", levelSmoothing());
    conf.set("depth_scale", depthScale());
    conf.set("lakes", lakes());
    conf.set("waterfalls", waterfalls());
    conf.set("edits", editsFile());

    for (auto& entry : waterways)
    {
        Config w("waterway");
        w.set("type", entry.first);
        w.set("width", entry.second.width);
        w.set("depth", entry.second.depth);
        w.set("speed", entry.second.speed);
        w.set("rapids", entry.second.rapids);
        conf.add(w);
    }

    Config s("shading");
    s.set("flow_speed", shading.flowSpeed);
    s.set("wave_strength", shading.waveStrength);
    s.set("wave_scale", shading.waveScale);
    s.set("clarity", shading.clarity);
    s.set("turbidity", shading.turbidity);
    s.set("deep_color", colorString(shading.deepColor));
    s.set("sediment_color", colorString(shading.sedimentColor));
    s.set("foam", shading.foam);
    s.set("shore_foam", shading.shoreFoam);
    s.set("rapids_foam", shading.rapidsFoam);
    s.set("reflection", shading.reflection);
    s.set("refraction", shading.refraction);
    s.set("caustics", shading.caustics);
    s.set("sun_glint", shading.sunGlint);
    s.set("edge_softness", shading.edgeSoftness);
    s.set("wet_banks", shading.wetBanks);
    s.set("sky_color", colorString(shading.skyColor));
    s.set("screen_space_reflections", shading.screenSpaceReflections);
    s.set("time_scale", shading.timeScale);
    s.set("spray", shading.spray);
    s.set("rainbows", shading.rainbows);
    conf.add(s);
    return conf;
}

void
RiverLayer::Options::fromConfig(const Config& conf)
{
    waterways = defaultWaterways();

    features().get(conf, "features");
    conf.get("level", level());
    conf.get("max_range", maxRange());
    conf.get("mesh_resolution", meshResolution());
    conf.get("level_smoothing", levelSmoothing());
    conf.get("depth_scale", depthScale());
    conf.get("lakes", lakes());
    conf.get("waterfalls", waterfalls());
    conf.get("edits", editsFile());

    for (auto& w : conf.children("waterway"))
    {
        std::string type = w.value("type");
        if (type.empty()) continue;
        WaterwayClass c = waterways.count(type) ? waterways[type] : WaterwayClass();
        w.get("width", c.width);
        w.get("depth", c.depth);
        w.get("speed", c.speed);
        w.get("rapids", c.rapids);
        waterways[type] = c;
    }

    const Config& s = conf.child("shading");
    if (!s.empty())
    {
        s.get("flow_speed", shading.flowSpeed);
        s.get("wave_strength", shading.waveStrength);
        s.get("wave_scale", shading.waveScale);
        s.get("clarity", shading.clarity);
        s.get("turbidity", shading.turbidity);
        if (s.hasValue("deep_color")) shading.deepColor = Color(s.value("deep_color"));
        if (s.hasValue("sediment_color")) shading.sedimentColor = Color(s.value("sediment_color"));
        s.get("foam", shading.foam);
        s.get("shore_foam", shading.shoreFoam);
        s.get("rapids_foam", shading.rapidsFoam);
        s.get("reflection", shading.reflection);
        s.get("refraction", shading.refraction);
        s.get("caustics", shading.caustics);
        s.get("sun_glint", shading.sunGlint);
        s.get("edge_softness", shading.edgeSoftness);
        s.get("wet_banks", shading.wetBanks);
        if (s.hasValue("sky_color")) shading.skyColor = Color(s.value("sky_color"));
        s.get("screen_space_reflections", shading.screenSpaceReflections);
        s.get("time_scale", shading.timeScale);
        s.get("spray", shading.spray);
        s.get("rainbows", shading.rainbows);
    }
}

Config
RiverCarveLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    riverLayer().set(conf, "river_layer");
    conf.set("bank_width", bankWidth());
    conf.set("bank_height", bankHeight());
    return conf;
}

void
RiverCarveLayer::Options::fromConfig(const Config& conf)
{
    minLevel().setDefault(13u);
    maxDataLevel().setDefault(18u);
    // Carved heights derive from the river data and the other elevation layers; caching
    // them would serve stale channels after any edit or upstream change. (init() makes
    // these explicit, which is what the base classes check.)
    cachePolicy().setDefault(CachePolicy::NO_CACHE);
    riverLayer().get(conf, "river_layer");
    conf.get("bank_width", bankWidth());
    conf.get("bank_height", bankHeight());
}

Config
RiverConstraintLayer::Options::getConfig() const
{
    Config conf = TerrainConstraintLayer::Options::getConfig();
    riverLayer().set(conf, "river_layer");
    return conf;
}

void
RiverConstraintLayer::Options::fromConfig(const Config& conf)
{
    // Constrained tiles cost far more to mesh than plain ones, and the banks only read as
    // sharp lines up close: with 33-vertex tiles a level 16 tile shows within a couple of km.
    minLevel().setDefault(16u);
    riverLayer().get(conf, "river_layer");
}

//...................................................................
// Implementation

namespace osgEarth
{
    class RiverTileManager;
}

//! Shared state of a RiverLayer. Background jobs hold it by weak pointer so a layer
//! can be destroyed while tiles are still building.
struct RiverLayer::Impl : public std::enable_shared_from_this<RiverLayer::Impl>
{
    RiverLayer* layer = nullptr;
    osg::observer_ptr<const Map> map;
    osg::ref_ptr<const Profile> dataProfile;
    unsigned dataLevel = 14u;
    osg::observer_ptr<TerrainEngineNode> engine;

    // settings snapshot for background builds
    mutable std::mutex settingsMutex;
    BuildSettings settings;

    // user edits
    mutable std::mutex editsMutex;
    FeatureList userWaterways; // spherical mercator
    std::set<FeatureID> hidden;
    std::map<FeatureID, float> widths;
    FeatureID nextFID = FIRST_USER_FID;

    // hydrology cache
    struct FieldEntry
    {
        osg::ref_ptr<TileField> field;
        std::list<TileKey>::iterator lru;
    };
    mutable std::mutex fieldMutex;
    std::map<TileKey, FieldEntry> fields;
    std::list<TileKey> fieldLRU;
    std::size_t maxFields = 160u;
    std::atomic<unsigned> revision{ 0u };
    Gate<TileKey> fieldGate;

    // rendering
    osg::ref_ptr<RiverTileManager> tiles;
    osg::ref_ptr<SceneCopyDrawable> sceneCopy;
    osg::ref_ptr<osg::StateSet> waterState; // program and render state of the water meshes
    osg::ref_ptr<osg::StateSet> sprayState; // program and render state of the spray particles
    osg::ref_ptr<osg::Texture2D> waveTex;
    osg::ref_ptr<osg::Texture2D> foamTex;
    osg::ref_ptr<osg::Texture2D> bedTex;
    double time = 0.0;
    double lastSimTime = -1.0;
    int debugView = 0;

    //! Elevation layers that define the natural terrain (everything except carving).
    ElevationLayerVector baseLayers(const Map* m) const
    {
        ElevationLayerVector all, out;
        m->getLayers(all);
        for (auto& l : all)
            if (l->isOpen() && dynamic_cast<RiverCarveLayer*>(l.get()) == nullptr)
                out.push_back(l);
        return out;
    }

    //! Heights of the natural terrain on a mercator lattice, for hydrology builds.
    HeightSampler makeHeightSampler()
    {
        std::weak_ptr<Impl> weak = shared_from_this();
        return [weak](double x0, double y0, double dx, double dy, unsigned cols, unsigned rows,
            std::vector<float>& out, ProgressCallback* progress) -> bool
        {
            std::shared_ptr<Impl> impl = weak.lock();
            if (!impl) return false;
            osg::ref_ptr<const Map> m;
            if (!impl->map.lock(m)) return false;

            out.assign((std::size_t)cols * rows, 0.0f);
            ElevationLayerVector layers = impl->baseLayers(m.get());
            if (layers.empty())
                return true;

            const SpatialReference* merc = impl->dataProfile->getSRS();
            const SpatialReference* mapSRS = m->getSRS();
            std::vector<osg::Vec3d> points((std::size_t)cols * rows);
            for (unsigned j = 0; j < rows; ++j)
                for (unsigned i = 0; i < cols; ++i)
                    points[(std::size_t)j * cols + i].set(x0 + i * dx, y0 + j * dy, 0.0);
            if (!merc->transform(points, mapSRS))
                return false;

            ElevationPool::WorkingSet ws;
            ws.setElevationLayers(layers);
            ElevationPool::Envelope env;
            const osg::Vec3d& c = points[points.size() / 2];
            double resolution = dx * mercScale(y0 + 0.5 * rows * dy);
            if (!m->getElevationPool()->prepareEnvelope(env, GeoPoint(mapSRS, c.x(), c.y(), 0.0, ALTMODE_ABSOLUTE),
                Distance(resolution, Units::METERS), &ws))
                return false;
            int count = env.sampleMapCoords(points.begin(), points.end(), progress, NO_DATA_VALUE);
            if (count < 0)
                return false;
            std::size_t missing = 0;
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                out[i] = (float)points[i].z();
                if (out[i] == NO_DATA_VALUE) ++missing;
            }
            // Holes would read as sea level and drag water levels (and the carved beds under
            // them) down by hundreds of meters. Fill small gaps from their neighbors; when
            // much is missing (a failed fetch, typically) fail so the build runs again later.
            if (missing > points.size() / 4)
                return false;
            return missing == 0 || fillHoles(out, cols, rows);
        };
    }

    //! Replaces NO_DATA_VALUE cells of a cols x rows lattice with the mean of their valid
    //! 4-neighbors, growing inward pass by pass. Returns false if any hole remains.
    static bool fillHoles(std::vector<float>& h, unsigned cols, unsigned rows)
    {
        for (unsigned pass = 0; pass < cols + rows; ++pass)
        {
            std::vector<float> next = h;
            bool remaining = false;
            for (unsigned j = 0; j < rows; ++j)
            {
                for (unsigned i = 0; i < cols; ++i)
                {
                    std::size_t k = (std::size_t)j * cols + i;
                    if (h[k] != NO_DATA_VALUE)
                        continue;
                    float sum = 0.0f;
                    int n = 0;
                    if (i > 0 && h[k - 1] != NO_DATA_VALUE) sum += h[k - 1], ++n;
                    if (i + 1 < cols && h[k + 1] != NO_DATA_VALUE) sum += h[k + 1], ++n;
                    if (j > 0 && h[k - cols] != NO_DATA_VALUE) sum += h[k - cols], ++n;
                    if (j + 1 < rows && h[k + cols] != NO_DATA_VALUE) sum += h[k + cols], ++n;
                    if (n > 0) next[k] = sum / (float)n;
                    else remaining = true;
                }
            }
            h.swap(next);
            if (!remaining)
                return true;
        }
        return false;
    }

    //! Waterway and water-area features around a data tile, with edits applied, in mercator.
    //! includeHidden keeps hidden source waterways (for picking them so they can be restored).
    FeatureList gatherFeatures(const TileKey& key, double bufferMeters, ProgressCallback* progress,
        bool includeHidden = false) const
    {
        FeatureList out;
        const SpatialReference* merc = dataProfile->getSRS();

        std::set<FeatureID> hide;
        std::map<FeatureID, float> widthOverrides;
        FeatureList user;
        {
            std::lock_guard<std::mutex> lock(editsMutex);
            hide = hidden;
            widthOverrides = widths;
            user = userWaterways;
        }

        FeatureSource* fs = layer->getFeatureSource();
        if (fs && fs->getStatus().isOK())
        {
            Query query(key);
            if (bufferMeters > 0.0)
                query.buffer() = Distance(bufferMeters, Units::METERS);
            osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(query, FeatureFilterChain(), nullptr, progress);
            while (cursor.valid() && cursor->hasMore())
            {
                osg::ref_ptr<Feature> f = cursor->nextFeature();
                if (!f.valid() || !f->getGeometry())
                    continue;
                if (f->getString("waterway").empty() && !isWaterArea(f.get()))
                    continue;
                if (!includeHidden && hide.count(f->getFID()))
                    continue;
                auto w = widthOverrides.find(f->getFID());
                if (w != widthOverrides.end())
                    f->set("width", (double)w->second);
                if (!f->getSRS()->isHorizEquivalentTo(merc))
                    f->transform(merc);
                out.push_back(f);
            }
        }

        if (!user.empty())
        {
            GeoExtent ex = key.getExtent();
            double k = mercScale(ex.yMin() + 0.5 * ex.height());
            double pad = (bufferMeters + 100.0) / k;
            ex.expand(pad, pad);
            for (auto& f : user)
            {
                if (ex.intersects(f->getExtent()))
                    out.push_back(new Feature(*f));
            }
        }
        return out;
    }

    //! Hydrology of a data tile from the cache, building it if needed. Safe from any thread.
    osg::ref_ptr<TileField> getField(const TileKey& key, ProgressCallback* progress)
    {
        {
            std::lock_guard<std::mutex> lock(fieldMutex);
            auto i = fields.find(key);
            if (i != fields.end())
            {
                fieldLRU.splice(fieldLRU.begin(), fieldLRU, i->second.lru);
                return i->second.field;
            }
        }

        ScopedGate<TileKey> gate(fieldGate, key);
        {
            std::lock_guard<std::mutex> lock(fieldMutex);
            auto i = fields.find(key);
            if (i != fields.end())
                return i->second.field;
        }

        unsigned rev = revision;
        BuildSettings s;
        {
            std::lock_guard<std::mutex> lock(settingsMutex);
            s = settings;
        }

        FeatureList features = gatherFeatures(key, CONTEXT_BUFFER, progress);
        if (progress && progress->isCanceled())
            return nullptr;

        osg::ref_ptr<TileField> field = buildField(key, features, s, makeHeightSampler(), CONTEXT_BUFFER, progress);
        if (!field.valid())
            return nullptr;

        if (rev == revision)
        {
            std::lock_guard<std::mutex> lock(fieldMutex);
            fieldLRU.push_front(key);
            FieldEntry& entry = fields[key];
            entry.field = field;
            entry.lru = fieldLRU.begin();
            while (fields.size() > maxFields)
            {
                fields.erase(fieldLRU.back());
                fieldLRU.pop_back();
            }
        }
        return field;
    }

    //! Builds the renderable water of one data tile. Returns an empty group when the tile
    //! has no water and null when the build was canceled.
    osg::ref_ptr<osg::Node> buildTile(const TileKey& key, ProgressCallback* progress)
    {
        osg::ref_ptr<const Map> m;
        if (!map.lock(m))
            return nullptr;
        osg::ref_ptr<TileField> field = getField(key, progress);
        if (!field.valid())
            return nullptr;
        osg::ref_ptr<osg::Group> group = new osg::Group();
        if (field->hasMesh() || field->hasBank())
        {
            unsigned triangles = 0u;
            osg::ref_ptr<osg::Node> mesh = buildMesh(*field, m->getSRS(), progress, &triangles);
            if (progress && progress->isCanceled())
                return nullptr;
            if (mesh.valid())
            {
                mesh->setStateSet(waterState.get());
                group->addChild(mesh.get());
            }
        }
        unsigned particles = 0u;
        osg::ref_ptr<osg::Node> spray = buildSpray(*field, m->getSRS(), &particles);
        if (spray.valid())
        {
            spray->setStateSet(sprayState.get());
            group->addChild(spray.get());
        }
        return group;
    }

    //! Data tiles intersecting an extent (any SRS).
    void dataKeys(const GeoExtent& extent, std::vector<TileKey>& out) const
    {
        out.clear();
        if (!extent.isValid() || !dataProfile.valid())
            return;
        GeoExtent ex = dataProfile->clampAndTransformExtent(extent);
        if (ex.isValid())
            dataProfile->getIntersectingTiles(ex, dataLevel, out);
    }

    //! Drops cached hydrology intersecting a mercator extent (all if invalid).
    void dropFields(const GeoExtent& mercExtent)
    {
        ++revision;
        std::lock_guard<std::mutex> lock(fieldMutex);
        for (auto i = fields.begin(); i != fields.end(); )
        {
            GeoExtent tile = i->first.getExtent();
            if (!mercExtent.isValid() || tile.intersects(mercExtent))
            {
                fieldLRU.erase(i->second.lru);
                i = fields.erase(i);
            }
            else ++i;
        }
    }
};

//...................................................................
// Tile paging

namespace osgEarth
{
    /**
     * Pages the water mesh of every data tile within range of the camera, builds new
     * tiles in background jobs, and rebuilds invalidated tiles while keeping the old
     * mesh on screen until the replacement is ready.
     */
    class RiverTileManager : public osg::Group
    {
    public:
        //! Constructs a tile manager for a layer implementation.
        RiverTileManager(std::shared_ptr<RiverLayer::Impl> impl) : _impl(impl)
        {
            setCullingActive(false);
            _copyGroup = new osg::Group();
            // Stage-level bins: nested under the water's bin, the copy would draw after the water.
            _copyGroup->getOrCreateStateSet()->setRenderBinDetails(SCENE_COPY_BIN, "RenderBin");
            _copyGroup->getOrCreateStateSet()->setNestRenderBins(false);
            _pool = jobs::get_pool("oe.rivers", 4u);
        }

        //! Sets the drawable that copies the scene before the water draws.
        void setSceneCopy(SceneCopyDrawable* copy)
        {
            _copyGroup->removeChildren(0, _copyGroup->getNumChildren());
            _sceneCopy = copy;
            if (copy) _copyGroup->addChild(copy);
        }

        //! Marks tiles intersecting a mercator extent (all if invalid) for rebuilding.
        void invalidate(const GeoExtent& mercExtent)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (auto& entry : _tiles)
            {
                if (!mercExtent.isValid() || entry.first.getExtent().intersects(mercExtent))
                {
                    entry.second.dirty = true;
                    ++entry.second.revision;
                    entry.second.job = jobs::future<osg::ref_ptr<osg::Node>>();
                }
            }
        }

        //! Drops every tile.
        void clear()
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _tiles.clear();
        }

        //! Current paging statistics.
        RiverLayer::Stats stats() const
        {
            RiverLayer::Stats s;
            std::lock_guard<std::mutex> lock(_mutex);
            for (auto& entry : _tiles)
            {
                if (entry.second.node.valid())
                {
                    ++s.tilesLoaded;
                    s.triangles += entry.second.triangles;
                    s.sprayParticles += entry.second.particles;
                }
                if (entry.second.job.working())
                    ++s.tilesPending;
            }
            s.tilesVisible = _visible;
            return s;
        }

        //! Takes finished builds and retires stale tiles. Call from the update traversal.
        void update(unsigned frame)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (auto i = _tiles.begin(); i != _tiles.end(); )
            {
                Tile& t = i->second;
                if (t.job.available())
                {
                    osg::ref_ptr<osg::Node> node = t.job.value();
                    if (node.valid())
                    {
                        t.node = node;
                        t.triangles = countTriangles(node.get());
                        t.particles = countParticles(node.get());
                        t.built = true;
                        if (t.jobRevision == t.revision)
                            t.dirty = false;
                    }
                    t.job = jobs::future<osg::ref_ptr<osg::Node>>();
                }

                if (frame - t.lastFrame > 240u && _tiles.size() > 200u)
                    i = _tiles.erase(i);
                else
                    ++i;
            }
        }

        //! Pages and draws tiles during cull. Other visitors (picking, GL object compiling)
        //! see every loaded tile; the update visitor is skipped since tiles need no updates.
        void traverse(osg::NodeVisitor& nv) override
        {
            if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
            {
                cull(static_cast<osgUtil::CullVisitor*>(&nv));
            }
            else if (nv.getVisitorType() != osg::NodeVisitor::UPDATE_VISITOR)
            {
                std::vector<osg::ref_ptr<osg::Node>> nodes;
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    for (auto& entry : _tiles)
                        if (entry.second.node.valid())
                            nodes.push_back(entry.second.node);
                }
                for (auto& node : nodes)
                    node->accept(nv);
            }
        }

        osg::BoundingSphere computeBound() const override { return osg::BoundingSphere(); }

    private:
        struct Tile
        {
            osg::ref_ptr<osg::Node> node;
            jobs::future<osg::ref_ptr<osg::Node>> job;
            unsigned lastFrame = 0u;
            unsigned revision = 0u;     // bumped by invalidate
            unsigned jobRevision = 0u;  // revision the pending job started at
            unsigned triangles = 0u;
            unsigned particles = 0u;
            bool built = false;
            bool dirty = false;
        };

        std::shared_ptr<RiverLayer::Impl> _impl;
        mutable std::mutex _mutex;
        std::map<TileKey, Tile> _tiles;
        osg::ref_ptr<osg::Group> _copyGroup;
        osg::ref_ptr<SceneCopyDrawable> _sceneCopy;
        jobs::jobpool* _pool = nullptr;
        std::atomic<unsigned> _visible{ 0u };
        std::atomic<unsigned> _lastStep{ ~0u };

        //! Once per frame: advance the water clock and take finished tile builds. Runs in
        //! the cull traversal so it works whether or not the app updates this subgraph.
        void frameStep(unsigned frame, double simulationTime)
        {
            if (_impl->lastSimTime >= 0.0 && _impl->layer)
            {
                double dt = std::min(std::max(simulationTime - _impl->lastSimTime, 0.0), 0.25);
                _impl->time += dt * _impl->layer->getShading().timeScale;
                // wrap well before float precision suffers; the flow phases cycle every 1.7 s
                if (_impl->time > 1.7 * 20000.0) _impl->time -= 1.7 * 20000.0;
            }
            _impl->lastSimTime = simulationTime;
            update(frame);
        }

        //! Requests and draws the tiles within range of this camera. Runs once per camera per
        //! frame; the first camera of a frame also advances paging and the animation clock.
        void cull(osgUtil::CullVisitor* cv)
        {
            const osg::Camera* camera = cv->getCurrentCamera();
            if (!camera || CameraUtils::isShadowCamera(camera) || CameraUtils::isDepthCamera(camera) ||
                CameraUtils::isPickCamera(camera))
                return;

            osg::ref_ptr<const Map> map;
            if (!_impl->map.lock(map) || !_impl->dataProfile.valid())
                return;

            // Where is the eye, in mercator?
            osg::Vec3d eyeWorld = cv->getEyePoint();
            GeoPoint eye;
            eye.fromWorld(map->getSRS()->getGeographicSRS(), eyeWorld);
            if (!eye.isValid() || std::abs(eye.y()) > 85.0)
                return;
            osg::Vec2d m = geoToMerc(eye.x(), eye.y());
            double k = mercScale(m.y());
            double range = _impl->layer->getMaxRange();
            double altitude = std::max(eye.z(), 0.0);
            if (altitude > range * 1.5)
                return;

            double reach = range / k;
            GeoExtent area(_impl->dataProfile->getSRS(), m.x() - reach, m.y() - reach, m.x() + reach, m.y() + reach);
            std::vector<TileKey> keys;
            _impl->dataProfile->getIntersectingTiles(area, _impl->dataLevel, keys);

            const osg::FrameStamp* stamp = cv->getFrameStamp();
            const unsigned frame = stamp ? stamp->getFrameNumber() : 0u;
            if (_lastStep.exchange(frame) != frame)
                frameStep(frame, stamp ? stamp->getSimulationTime() : 0.0);

            std::vector<osg::ref_ptr<osg::Node>> visible;
            {
                std::lock_guard<std::mutex> lock(_mutex);
                for (auto& key : keys)
                {
                    const GeoExtent& ex = key.getExtent();
                    double dx = std::max(std::max(ex.xMin() - m.x(), m.x() - ex.xMax()), 0.0) * k;
                    double dy = std::max(std::max(ex.yMin() - m.y(), m.y() - ex.yMax()), 0.0) * k;
                    double distance = std::sqrt(dx * dx + dy * dy + altitude * altitude * 0.25);
                    if (distance > range)
                        continue;

                    Tile& t = _tiles[key];
                    t.lastFrame = frame;
                    bool needsBuild = !t.built || t.dirty;
                    if (needsBuild && t.job.empty())
                        dispatch(key, t, (float)distance);
                    if (t.node.valid() && t.node->asGroup() && t.node->asGroup()->getNumChildren() > 0)
                        visible.push_back(t.node);
                }
            }

            _visible = (unsigned)visible.size();
            if (visible.empty())
                return;

            // Per-camera values for the shader.
            osg::ref_ptr<osg::StateSet> ss = new osg::StateSet();
            const osg::Viewport* vp = camera->getViewport();
            osg::Vec4f viewport(0, 0, 1, 1);
            if (vp) viewport.set(vp->x(), vp->y(), vp->width(), vp->height());
            ss->addUniform(new osg::Uniform("oe_river_viewport", viewport));
            bool valid = _sceneCopy.valid() && _sceneCopy->isValidFor(camera);
            ss->addUniform(new osg::Uniform("oe_river_screenValid", valid ? 1.0f : 0.0f));
            ss->addUniform(new osg::Uniform("oe_river_time", (float)_impl->time));
            cv->pushStateSet(ss.get());
            if (_sceneCopy.valid())
                _copyGroup->accept(*cv);
            for (auto& node : visible)
                node->accept(*cv);
            cv->popStateSet();
        }

        //! Starts a background build of a tile's water, nearest tiles first. The job holds the
        //! implementation weakly, so it does nothing once the layer is gone.
        void dispatch(const TileKey& key, Tile& t, float distance)
        {
            std::weak_ptr<RiverLayer::Impl> weak = _impl;
            jobs::context context;
            context.name = "River tile " + key.str();
            context.pool = _pool;
            context.priority = [distance]() { return -distance; };
            t.jobRevision = t.revision;
            t.job = jobs::dispatch([weak, key](Cancelable& c) -> osg::ref_ptr<osg::Node>
                {
                    std::shared_ptr<RiverLayer::Impl> impl = weak.lock();
                    if (!impl || c.canceled())
                        return nullptr;
                    osg::ref_ptr<ProgressCallback> progress = new ProgressCallback(&c);
                    return impl->buildTile(key, progress.get());
                }, context);
        }
    };
}

//...................................................................
// RiverLayer

RiverLayer::~RiverLayer()
{
    if (_impl)
    {
        if (_impl->tiles.valid())
            _impl->tiles->clear();
        _impl->layer = nullptr;
    }
}

void
RiverLayer::init()
{
    VisibleLayer::init();

    _impl = std::make_shared<Impl>();
    _impl->layer = this;
    _impl->dataProfile = Profile::create(Profile::SPHERICAL_MERCATOR);

    _root = new osg::Group();
    _root->setName("RiverLayer");
    _root->setStateSet(getOrCreateStateSet());

    _impl->tiles = new RiverTileManager(_impl);
    _root->addChild(_impl->tiles.get());

    buildStateSet();
    applyShading();
}

Status
RiverLayer::openImplementation()
{
    Status parent = VisibleLayer::openImplementation();
    if (parent.isError())
        return parent;

#ifndef OSGEARTH_HAVE_GEOS
    return Status(Status::ResourceUnavailable, "RiverLayer requires osgEarth built with GEOS");
#endif

    Status fs = options().features().open(getReadOptions());
    if (fs.isError())
        return fs;

    _impl->dataLevel = options().level().get();
    {
        std::lock_guard<std::mutex> lock(_impl->settingsMutex);
        _impl->settings.classes = options().waterways;
        _impl->settings.meshResolution = options().meshResolution().get();
        _impl->settings.levelSmoothing = options().levelSmoothing().get();
        _impl->settings.depthScale = options().depthScale().get();
        _impl->settings.lakes = options().lakes().get();
        _impl->settings.waterfalls = options().waterfalls().get();
    }

    if (options().editsFile().isSet() && !options().editsFile()->empty())
    {
        std::string path = options().editsFile().get();
        if (getReadOptions() && !osgDB::isAbsolutePath(path))
        {
            std::string base = options().referrer();
            if (!base.empty()) path = osgDB::concatPaths(osgDB::getFilePath(base), path);
        }
        if (osgDB::fileExists(path))
        {
            Status s = loadEdits(path);
            if (s.isError())
                OE_WARN << LC << "Failed to load edits from " << path << ": " << s.message() << std::endl;
        }
    }

    return Status::NoError;
}

Status
RiverLayer::closeImplementation()
{
    if (_impl->tiles.valid())
        _impl->tiles->clear();
    _impl->dropFields(GeoExtent::INVALID);
    return VisibleLayer::closeImplementation();
}

void
RiverLayer::addedToMap(const Map* map)
{
    VisibleLayer::addedToMap(map);
    options().features().addedToMap(map);
    _impl->map = map;

    if (!getFeatureSource())
        OE_WARN << LC << "No feature source; only user-drawn waterways will appear" << std::endl;
}

void
RiverLayer::removedFromMap(const Map* map)
{
    options().features().removedFromMap(map);
    if (_impl->tiles.valid())
        _impl->tiles->clear();
    _impl->map = nullptr;
    VisibleLayer::removedFromMap(map);
}

void
RiverLayer::prepareForRendering(TerrainEngine* engine)
{
    VisibleLayer::prepareForRendering(engine);

    _impl->engine = dynamic_cast<TerrainEngineNode*>(engine);

    TerrainResources* res = engine ? engine->getResources() : nullptr;
    if (!res)
        return;

    osg::StateSet* ss = getOrCreateStateSet();

    if (!_waveUnit.valid() && res->reserveTextureImageUnitForLayer(_waveUnit, this, "River waves") &&
        res->reserveTextureImageUnitForLayer(_foamUnit, this, "River foam") &&
        res->reserveTextureImageUnitForLayer(_bedUnit, this, "River bed"))
    {
        _impl->waveTex = createDataTexture(createWaveImage(512u).get());
        _impl->foamTex = createDataTexture(createFoamImage(512u).get());
        _impl->bedTex = createDataTexture(createBedImage(512u).get());
        ss->setTextureAttribute(_waveUnit.unit(), _impl->waveTex.get());
        ss->setTextureAttribute(_foamUnit.unit(), _impl->foamTex.get());
        ss->setTextureAttribute(_bedUnit.unit(), _impl->bedTex.get());
        ss->addUniform(new osg::Uniform("oe_river_waveTex", _waveUnit.unit()));
        ss->addUniform(new osg::Uniform("oe_river_foamTex", _foamUnit.unit()));
        ss->addUniform(new osg::Uniform("oe_river_bedTex", _bedUnit.unit()));
    }

    // The scene copy binds raw GL textures, so it needs units nothing else will touch.
    if (!_colorUnit.valid() && res->reserveTextureImageUnit(_colorUnit, "River scene color") &&
        res->reserveTextureImageUnit(_depthUnit, "River scene depth"))
    {
        _impl->sceneCopy = new SceneCopyDrawable(_colorUnit.unit(), _depthUnit.unit());
        _impl->tiles->setSceneCopy(_impl->sceneCopy.get());
        ss->addUniform(new osg::Uniform("oe_river_sceneColor", _colorUnit.unit()));
        ss->addUniform(new osg::Uniform("oe_river_sceneDepth", _depthUnit.unit()));
        ss->setDefine("OE_RIVER_SCREEN");
    }
}

osg::Node*
RiverLayer::getNode() const
{
    return _root.get();
}

Config
RiverLayer::getConfig() const
{
    return VisibleLayer::getConfig();
}

void
RiverLayer::buildStateSet()
{
    // The layer's own state set carries what the water and the spray share (textures, uniforms,
    // defines); each of them gets a state set of its own with its program and render state.
    Shaders shaders;
    getOrCreateStateSet()->setDefine("OE_RIVER_SSR");

    osg::ref_ptr<osg::StateSet> water = new osg::StateSet();
    VirtualProgram* vp = VirtualProgram::getOrCreate(water.get());
    vp->setName("River water");
    shaders.load(vp, shaders.River);
    vp->addBindAttribLocation("oe_river_flowAttr", FLOW_ATTRIB);
    vp->addBindAttribLocation("oe_river_dataAttr", DATA_ATTRIB);
    water->setRenderBinDetails(WATER_BIN, "RenderBin");
    water->setNestRenderBins(false);
    water->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, true), osg::StateAttribute::ON);
    water->setMode(GL_BLEND, osg::StateAttribute::OFF);
    water->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    _impl->waterState = water;

    // Spray blends over everything opaque and over the water without
    // writing depth.
    osg::ref_ptr<osg::StateSet> spray = new osg::StateSet();
    VirtualProgram* sprayVP = VirtualProgram::getOrCreate(spray.get());
    sprayVP->setName("River spray");
    shaders.load(sprayVP, shaders.RiverSpray);
    spray->setRenderBinDetails(SPRAY_BIN, "RenderBin");
    spray->setNestRenderBins(false);
    spray->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false), osg::StateAttribute::ON);
    spray->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
    spray->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    _impl->sprayState = spray;
}

void
RiverLayer::applyShading()
{
    const Shading& s = options().shading;
    osg::StateSet* ss = getOrCreateStateSet();
    ss->getOrCreateUniform("oe_river_p0", osg::Uniform::FLOAT_VEC4)->set(
        osg::Vec4f(s.flowSpeed, s.waveStrength, s.waveScale, s.clarity));
    ss->getOrCreateUniform("oe_river_p1", osg::Uniform::FLOAT_VEC4)->set(
        osg::Vec4f(s.turbidity, s.foam, s.shoreFoam, s.rapidsFoam));
    ss->getOrCreateUniform("oe_river_p2", osg::Uniform::FLOAT_VEC4)->set(
        osg::Vec4f(s.reflection, s.refraction, s.caustics, s.sunGlint));
    ss->getOrCreateUniform("oe_river_p3", osg::Uniform::FLOAT_VEC4)->set(
        osg::Vec4f(s.edgeSoftness, (float)_impl->debugView, s.screenSpaceReflections ? 1.0f : 0.0f,
            options().maxRange().get()));
    ss->getOrCreateUniform("oe_river_p4", osg::Uniform::FLOAT_VEC4)->set(
        osg::Vec4f(s.wetBanks, 0.0f, 0.0f, 0.0f));
    ss->getOrCreateUniform("oe_river_p5", osg::Uniform::FLOAT_VEC4)->set(
        osg::Vec4f(s.spray, s.rainbows, 0.0f, 0.0f));

    // Wave octaves: ~12 m swells, ~4 m ripples (rotated, |M| = 5), ~1.3 m chop (rotated, |M| = sqrt 5).
    // Frequencies are whole repeats per 1024 texture units so tiles join seamlessly.
    double scale = s.waveScale;
    ss->getOrCreateUniform("oe_river_freq", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4f(
        repeatFrequency(12.0, scale),
        repeatFrequency(4.0 * 5.0, scale),
        repeatFrequency(1.3 * 2.2360679, scale),
        repeatFrequency(8.0, scale)));
    ss->getOrCreateUniform("oe_river_freq2", osg::Uniform::FLOAT_VEC4)->set(osg::Vec4f(
        repeatFrequency(3.5, 1.0),   // caustics
        repeatFrequency(80.0, 1.0),  // phase/patch noise
        repeatFrequency(6.0, 1.0),   // bed stones
        0.0f));

    ss->getOrCreateUniform("oe_river_deepColor", osg::Uniform::FLOAT_VEC3)->set(toLinear(s.deepColor));
    ss->getOrCreateUniform("oe_river_sedimentColor", osg::Uniform::FLOAT_VEC3)->set(toLinear(s.sedimentColor));
    ss->getOrCreateUniform("oe_river_skyColor", osg::Uniform::FLOAT_VEC3)->set(toLinear(s.skyColor));
    ss->getOrCreateUniform("oe_river_depthBias", osg::Uniform::FLOAT)->set(0.0015f);

    if (s.screenSpaceReflections)
        ss->setDefine("OE_RIVER_SSR");
    else
        ss->removeDefine("OE_RIVER_SSR");
}

void
RiverLayer::setFeatureSource(FeatureSource* value)
{
    if (getFeatureSource() != value)
    {
        options().features().setLayer(value);
        invalidate(GeoExtent::INVALID);
    }
}

FeatureSource*
RiverLayer::getFeatureSource() const
{
    return options().features().getLayer();
}

void
RiverLayer::setShading(const Shading& value)
{
    options().shading = value;
    applyShading();
}

const RiverLayer::Shading&
RiverLayer::getShading() const
{
    return options().shading;
}

void
RiverLayer::setWaterwayClass(const std::string& type, const WaterwayClass& value)
{
    options().waterways[type] = value;
    {
        std::lock_guard<std::mutex> lock(_impl->settingsMutex);
        _impl->settings.classes = options().waterways;
    }
    invalidate(GeoExtent::INVALID);
}

const std::map<std::string, RiverLayer::WaterwayClass>&
RiverLayer::getWaterwayClasses() const
{
    return options().waterways;
}

void
RiverLayer::setMaxRange(float value)
{
    options().maxRange() = value;
    applyShading();
}

void
RiverLayer::setWaterfalls(bool value)
{
    if (options().waterfalls().get() == value)
        return;
    options().waterfalls() = value;
    {
        std::lock_guard<std::mutex> lock(_impl->settingsMutex);
        _impl->settings.waterfalls = value;
    }
    invalidate(GeoExtent::INVALID);
}

bool
RiverLayer::getWaterfalls() const
{
    return options().waterfalls().get();
}

float
RiverLayer::getMaxRange() const
{
    return options().maxRange().get();
}

void
RiverLayer::setDebugView(int value)
{
    _impl->debugView = value;
    applyShading();
}

int
RiverLayer::getDebugView() const
{
    return _impl->debugView;
}

bool
RiverLayer::getWaterSample(const GeoPoint& point, WaterSample& out, ProgressCallback* progress) const
{
    out = WaterSample();
    if (!isOpen() || !point.isValid())
        return false;
    GeoPoint p = point.transform(_impl->dataProfile->getSRS());
    if (!p.isValid())
        return false;
    TileKey key = _impl->dataProfile->createTileKey(p.x(), p.y(), _impl->dataLevel);
    if (!key.valid())
        return false;
    osg::ref_ptr<TileField> field = _impl->getField(key, progress);
    if (!field.valid() || !field->hasWater())
        return false;
    return field->sample(field->toLocal(p.x(), p.y()), 0, 60.0f, out);
}

bool
RiverLayer::sampleWaterGrid(
    const GeoExtent& extent,
    unsigned cols,
    unsigned rows,
    float margin,
    std::vector<WaterSample>& out,
    ProgressCallback* progress) const
{
    out.assign((std::size_t)cols * rows, WaterSample());
    if (!isOpen() || !extent.isValid() || cols < 2 || rows < 2)
        return false;

    std::vector<TileKey> keys;
    GeoExtent padded = extent;
    _impl->dataKeys(padded, keys);
    if (keys.empty())
        return false;

    // Fetch the hydrology of every data tile involved; bail early if none has water.
    std::map<TileKey, osg::ref_ptr<TileField>> fields;
    bool any = false;
    for (auto& key : keys)
    {
        osg::ref_ptr<TileField> field = _impl->getField(key, progress);
        if (progress && progress->isCanceled())
            return false;
        if (field.valid() && field->hasWater())
        {
            fields[key] = field;
            any = true;
        }
    }
    if (!any)
        return false;

    // Grid points (edges inclusive, row 0 = south) in mercator.
    std::vector<osg::Vec3d> points((std::size_t)cols * rows);
    for (unsigned j = 0; j < rows; ++j)
        for (unsigned i = 0; i < cols; ++i)
            points[(std::size_t)j * cols + i].set(
                extent.xMin() + extent.width() * i / (double)(cols - 1),
                extent.yMin() + extent.height() * j / (double)(rows - 1),
                0.0);
    if (!extent.getSRS()->transform(points, _impl->dataProfile->getSRS()))
        return false;

    bool found = false;
    for (std::size_t n = 0; n < points.size(); ++n)
    {
        const osg::Vec3d& p = points[n];
        TileKey key = _impl->dataProfile->createTileKey(p.x(), p.y(), _impl->dataLevel);
        auto f = fields.find(key);
        if (f == fields.end())
            continue;
        if (f->second->sample(f->second->toLocal(p.x(), p.y()), 0, margin, out[n]))
            found = true;
        if ((n & 4095) == 0 && progress && progress->isCanceled())
            return false;
    }
    return found;
}

bool
RiverLayer::getWaterOutlines(const TileKey& key, FeatureList& out, ProgressCallback* progress) const
{
    if (!isOpen())
        return false;

    // Only what touches the tile (plus a sliver): the mesher transforms every point it is
    // given, so handing it whole data tiles of shoreline made each terrain tile pay for
    // kilometers of rivers it never uses.
    GeoExtent ex = key.getExtent().transform(_impl->dataProfile->getSRS());
    if (!ex.isValid())
        return false;
    double margin = 0.02 * std::max(ex.width(), ex.height()) + 1.0;
    osg::Vec2d clipMin(ex.xMin() - margin, ex.yMin() - margin), clipMax(ex.xMax() + margin, ex.yMax() + margin);

    // Detail finer than about a quarter of a terrain grid cell only adds vertices. The
    // tolerance is snapped to a power of two so all tiles of a level share one simplification
    // (and so agree where lines cross their shared edges).
    int toleranceKey = (int)std::lround(std::log2(std::max(ex.height(), 1e-3) / 128.0));
    double tolerance = std::ldexp(1.0, toleranceKey);

    // Long bank segments can belong to a data tile just beyond the terrain tile, so look a
    // little further for the fields that might own them.
    GeoExtent search = ex;
    search.expand(100.0, 100.0);
    std::vector<TileKey> keys;
    _impl->dataKeys(search, keys);
    for (auto& dataKey : keys)
    {
        osg::ref_ptr<TileField> field = _impl->getField(dataKey, progress);
        if (field.valid() && field->hasWater())
            getOutlines(*field, clipMin, clipMax, toleranceKey, tolerance, out);
    }
    return !out.empty();
}

FeatureID
RiverLayer::addWaterway(Feature* feature)
{
    if (!feature || !feature->getGeometry())
        return 0;
    osg::ref_ptr<Feature> f = new Feature(*feature);
    f->transform(_impl->dataProfile->getSRS());
    f->set("river:user", std::string("yes"));
    FeatureID fid;
    {
        std::lock_guard<std::mutex> lock(_impl->editsMutex);
        fid = _impl->nextFID++;
        f->setFID(fid);
        _impl->userWaterways.push_back(f);
    }
    invalidate(f->getExtent());
    return fid;
}

bool
RiverLayer::updateWaterway(Feature* feature)
{
    if (!feature || !feature->getGeometry())
        return false;
    osg::ref_ptr<Feature> f = new Feature(*feature);
    f->transform(_impl->dataProfile->getSRS());
    f->set("river:user", std::string("yes"));
    GeoExtent affected;
    {
        std::lock_guard<std::mutex> lock(_impl->editsMutex);
        auto i = std::find_if(_impl->userWaterways.begin(), _impl->userWaterways.end(),
            [&](const osg::ref_ptr<Feature>& u) { return u->getFID() == f->getFID(); });
        if (i == _impl->userWaterways.end())
            return false;
        affected = (*i)->getExtent();
        *i = f;
    }
    affected.expandToInclude(f->getExtent());
    invalidate(affected);
    return true;
}

bool
RiverLayer::removeWaterway(FeatureID fid)
{
    GeoExtent affected;
    {
        std::lock_guard<std::mutex> lock(_impl->editsMutex);
        auto i = std::find_if(_impl->userWaterways.begin(), _impl->userWaterways.end(),
            [&](const osg::ref_ptr<Feature>& u) { return u->getFID() == fid; });
        if (i == _impl->userWaterways.end())
            return false;
        affected = (*i)->getExtent();
        _impl->userWaterways.erase(i);
    }
    invalidate(affected);
    return true;
}

FeatureList
RiverLayer::getWaterways() const
{
    std::lock_guard<std::mutex> lock(_impl->editsMutex);
    return _impl->userWaterways;
}

void
RiverLayer::setSourceWaterwayHidden(FeatureID fid, bool hidden)
{
    {
        std::lock_guard<std::mutex> lock(_impl->editsMutex);
        if (hidden) _impl->hidden.insert(fid);
        else _impl->hidden.erase(fid);
    }
}

bool
RiverLayer::isSourceWaterwayHidden(FeatureID fid) const
{
    std::lock_guard<std::mutex> lock(_impl->editsMutex);
    return _impl->hidden.count(fid) > 0;
}

void
RiverLayer::setSourceWaterwayWidth(FeatureID fid, float width)
{
    std::lock_guard<std::mutex> lock(_impl->editsMutex);
    if (width > 0.0f) _impl->widths[fid] = width;
    else _impl->widths.erase(fid);
}

float
RiverLayer::getSourceWaterwayWidth(FeatureID fid) const
{
    std::lock_guard<std::mutex> lock(_impl->editsMutex);
    auto i = _impl->widths.find(fid);
    return i != _impl->widths.end() ? i->second : 0.0f;
}

bool
RiverLayer::pickWaterway(
    const GeoPoint& point,
    float radius,
    osg::ref_ptr<Feature>& out,
    bool& isUserWaterway,
    ProgressCallback* progress) const
{
    out = nullptr;
    isUserWaterway = false;
    if (!isOpen() || !point.isValid())
        return false;
    GeoPoint p = point.transform(_impl->dataProfile->getSRS());
    TileKey key = _impl->dataProfile->createTileKey(p.x(), p.y(), _impl->dataLevel);
    if (!key.valid())
        return false;

    // include hidden source waterways so they can be restored
    FeatureList features = _impl->gatherFeatures(key, (double)radius, progress, true);

    double k = mercScale(p.y());
    double best = radius / k;
    BuildSettings s;
    {
        std::lock_guard<std::mutex> lock(_impl->settingsMutex);
        s = _impl->settings;
    }
    osg::Vec2d q(p.x(), p.y());
    for (auto& f : features)
    {
        WaterwayClass params;
        if (!getChannelParams(f.get(), s, params))
            continue;
        ConstGeometryIterator iter(f->getGeometry(), false);
        while (iter.hasMore())
        {
            const Geometry* part = iter.next();
            for (std::size_t i = 0; i + 1 < part->size(); ++i)
            {
                osg::Vec2d a((*part)[i].x(), (*part)[i].y()), b((*part)[i + 1].x(), (*part)[i + 1].y());
                osg::Vec2d ab = b - a;
                double t = ab.length2() > 0.0 ? std::min(std::max(((q - a) * ab) / ab.length2(), 0.0), 1.0) : 0.0;
                double d = (q - (a + ab * t)).length();
                if (d < best)
                {
                    best = d;
                    out = f;
                }
            }
        }
    }
    if (out.valid())
        isUserWaterway = out->hasAttr("river:user");
    return out.valid();
}

void
RiverLayer::invalidate(const GeoExtent& extent)
{
    GeoExtent mercExtent;
    if (extent.isValid())
    {
        mercExtent = extent.transform(_impl->dataProfile->getSRS());
        // the hydrology of a tile depends on everything within its context buffer
        double k = mercScale(mercExtent.yMin() + 0.5 * mercExtent.height());
        double pad = (CONTEXT_BUFFER + 150.0) / k;
        mercExtent.expand(pad, pad);
    }

    _impl->dropFields(mercExtent);
    if (_impl->tiles.valid())
        _impl->tiles->invalidate(mercExtent);

    // Terrain carving and bank constraints depend on the same hydrology.
    osg::ref_ptr<TerrainEngineNode> engine;
    if (_impl->engine.lock(engine))
    {
        if (mercExtent.isValid())
            engine->invalidateRegion(mercExtent, 12u);
        else
            engine->invalidateRegion(GeoExtent::INVALID, 12u);
    }

    onInvalidate.fire(mercExtent);
}

RiverLayer::Stats
RiverLayer::getStats() const
{
    Stats s;
    if (_impl->tiles.valid())
        s = _impl->tiles->stats();
    std::lock_guard<std::mutex> lock(_impl->fieldMutex);
    s.fieldsCached = (unsigned)_impl->fields.size();
    for (auto& entry : _impl->fields)
    {
        const TileField* field = entry.second.field.get();
        if (!field)
            continue;
        // count each fall once: in the field whose tile holds its foot
        for (const Fall& fall : field->falls)
        {
            const osg::Vec2d& p = field->channels[fall.channel].p[fall.foot];
            if (p.x() >= 0.0 && p.y() >= 0.0 && p.x() < field->size.x() && p.y() < field->size.y())
                ++s.falls;
        }
    }
    return s;
}

Status
RiverLayer::saveEdits(const std::string& path) const
{
    FeatureList user;
    std::set<FeatureID> hidden;
    std::map<FeatureID, float> widths;
    {
        std::lock_guard<std::mutex> lock(_impl->editsMutex);
        user = _impl->userWaterways;
        hidden = _impl->hidden;
        widths = _impl->widths;
    }

    const SpatialReference* wgs84 = SpatialReference::get("wgs84");
    Json::Value root(Json::objectValue);
    root["type"] = "FeatureCollection";
    Json::Value features(Json::arrayValue);
    for (auto& f : user)
    {
        osg::ref_ptr<Feature> copy = new Feature(*f);
        copy->transform(wgs84);
        Json::Value feature(Json::objectValue);
        feature["type"] = "Feature";
        Json::Value props(Json::objectValue);
        for (auto& attr : copy->getAttrs())
        {
            if (attr.first == "river:user") continue;
            const AttributeValue& value = attr.second;
            switch (value.getType())
            {
            case ATTRTYPE_DOUBLE: props[attr.first] = value.get<double>(); break;
            case ATTRTYPE_INT: // this JSON library has 32-bit integers
                if (value.get<std::int64_t>() >= INT_MIN && value.get<std::int64_t>() <= INT_MAX)
                    props[attr.first] = (int)value.get<std::int64_t>();
                else
                    props[attr.first] = (double)value.get<std::int64_t>();
                break;
            case ATTRTYPE_BOOL: props[attr.first] = value.get<bool>(); break;
            default: props[attr.first] = value.getAsString(); break;
            }
        }
        feature["properties"] = props;
        Json::Value geometry(Json::objectValue);
        Json::Value coords(Json::arrayValue);
        bool polygon = copy->getGeometry()->isPolygon();
        ConstGeometryIterator iter(copy->getGeometry(), false);
        Json::Value parts(Json::arrayValue);
        while (iter.hasMore())
        {
            const Geometry* part = iter.next();
            Json::Value line(Json::arrayValue);
            for (auto& v : *part)
            {
                Json::Value c(Json::arrayValue);
                c.append(v.x());
                c.append(v.y());
                line.append(c);
            }
            if (polygon && !line.empty())
                line.append(line[0u]);
            parts.append(line);
        }
        if (polygon)
        {
            geometry["type"] = "Polygon";
            geometry["coordinates"] = parts;
        }
        else
        {
            geometry["type"] = parts.size() == 1 ? "LineString" : "MultiLineString";
            geometry["coordinates"] = parts.size() == 1 ? parts[0u] : parts;
        }
        feature["geometry"] = geometry;
        features.append(feature);
    }
    root["features"] = features;

    Json::Value overrides(Json::objectValue);
    Json::Value hiddenList(Json::arrayValue);
    for (auto fid : hidden)
        hiddenList.append(Json::Value(std::to_string(fid)));
    overrides["hidden"] = hiddenList;
    Json::Value widthMap(Json::objectValue);
    for (auto& w : widths)
        widthMap[std::to_string(w.first)] = w.second;
    overrides["widths"] = widthMap;
    root["osgearth_river_overrides"] = overrides;

    std::ofstream out(path.c_str());
    if (!out.is_open())
        return Status(Status::ResourceUnavailable, "Cannot write " + path);
    Json::StyledWriter writer;
    out << writer.write(root);
    return out.good() ? Status::NoError : Status(Status::GeneralError, "Write failed: " + path);
}

Status
RiverLayer::loadEdits(const std::string& path)
{
    std::ifstream in(path.c_str());
    if (!in.is_open())
        return Status(Status::ResourceUnavailable, "Cannot read " + path);
    std::stringstream buf;
    buf << in.rdbuf();

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(buf.str(), root) || !root.isObject())
        return Status(Status::ConfigurationError, "Not a valid GeoJSON file: " + path);

    const SpatialReference* wgs84 = SpatialReference::get("wgs84");
    FeatureList user;
    const Json::Value& features = root["features"];
    for (unsigned i = 0; features.isArray() && i < features.size(); ++i)
    {
        const Json::Value& jf = features[i];
        const Json::Value& geometry = jf["geometry"];
        std::string type = geometry["type"].asString();
        const Json::Value& coords = geometry["coordinates"];
        osg::ref_ptr<Geometry> geom;
        auto readLine = [](const Json::Value& line, Geometry* out)
        {
            for (unsigned j = 0; j < line.size(); ++j)
                out->push_back(line[j][0u].asDouble(), line[j][1u].asDouble(), 0.0);
        };
        if (type == "LineString")
        {
            geom = new LineString();
            readLine(coords, geom.get());
        }
        else if (type == "MultiLineString")
        {
            osg::ref_ptr<MultiGeometry> multi = new MultiGeometry();
            for (unsigned j = 0; j < coords.size(); ++j)
            {
                osg::ref_ptr<LineString> line = new LineString();
                readLine(coords[j], line.get());
                multi->add(line.get());
            }
            geom = multi;
        }
        else if (type == "Polygon" && coords.size() > 0)
        {
            osg::ref_ptr<Polygon> poly = new Polygon();
            readLine(coords[0u], poly.get());
            if (poly->size() > 1) poly->pop_back();
            geom = poly;
        }
        if (!geom.valid() || geom->size() + geom->getTotalPointCount() < 2)
            continue;

        osg::ref_ptr<Feature> f = new Feature(geom.get(), wgs84);
        const Json::Value& props = jf["properties"];
        for (auto& name : props.getMemberNames())
        {
            const Json::Value& value = props[name];
            if (value.isBool())
                f->set(name, value.asBool());
            else if (value.isIntegral())
                f->set(name, (std::int64_t)value.asDouble()); // asInt would assert on large unsigned values
            else if (value.isNumeric())
                f->set(name, value.asDouble());
            else if (!value.isNull())
                f->set(name, value.asString());
        }
        f->transform(_impl->dataProfile->getSRS());
        f->set("river:user", std::string("yes"));
        user.push_back(f);
    }

    std::set<FeatureID> hidden;
    std::map<FeatureID, float> widths;
    const Json::Value& overrides = root["osgearth_river_overrides"];
    if (overrides.isObject())
    {
        const Json::Value& h = overrides["hidden"];
        for (unsigned i = 0; h.isArray() && i < h.size(); ++i)
            hidden.insert((FeatureID)std::atoll(h[i].asString().c_str()));
        const Json::Value& w = overrides["widths"];
        if (w.isObject())
            for (auto& name : w.getMemberNames())
                widths[(FeatureID)std::atoll(name.c_str())] = (float)w[name].asDouble();
    }

    {
        std::lock_guard<std::mutex> lock(_impl->editsMutex);
        for (auto& f : user)
            f->setFID(_impl->nextFID++);
        _impl->userWaterways.swap(user);
        _impl->hidden.swap(hidden);
        _impl->widths.swap(widths);
    }
    invalidate(GeoExtent::INVALID);
    return Status::NoError;
}

//...................................................................
// RiverCarveLayer

void
RiverCarveLayer::init()
{
    ElevationLayer::init();
    // the carve only makes sense in the channels it carves
    setName(getName().empty() ? "River channels" : getName());

    // The base classes honor only explicitly set options, not defaults: make ours explicit.
    // Carved heights must not be cached (they follow the river data and the edits), and
    // coarse tiles are not worth carving.
    if (!options().cachePolicy().isSet())
        options().cachePolicy() = CachePolicy::NO_CACHE;
    if (!options().minLevel().isSet())
        options().minLevel() = 13u;
}

Status
RiverCarveLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    Status rs = options().riverLayer().open(getReadOptions());
    if (rs.isError())
        return rs;

    const Profile* profile = getProfile();
    if (!profile)
    {
        profile = Profile::create(Profile::GLOBAL_GEODETIC);
        setProfile(profile);
    }
    addDataExtent(DataExtent(profile->getExtent(), getMinLevel(), getMaxDataLevel()));
    return Status::NoError;
}

void
RiverCarveLayer::addedToMap(const Map* map)
{
    ElevationLayer::addedToMap(map);
    options().riverLayer().addedToMap(map);
    _map = map;
    if (!getRiverLayer())
        OE_WARN << "[RiverCarveLayer] \"" << getName() << "\" has no river layer to carve under" << std::endl;
}

void
RiverCarveLayer::removedFromMap(const Map* map)
{
    options().riverLayer().removedFromMap(map);
    _map = nullptr;
    ElevationLayer::removedFromMap(map);
}

Config
RiverCarveLayer::getConfig() const
{
    return ElevationLayer::getConfig();
}

void
RiverCarveLayer::setRiverLayer(RiverLayer* value)
{
    options().riverLayer().setLayer(value);
}

RiverLayer*
RiverCarveLayer::getRiverLayer() const
{
    return options().riverLayer().getLayer();
}

void
RiverCarveLayer::setBankWidth(float value)
{
    options().bankWidth() = value;
}

float
RiverCarveLayer::getBankWidth() const
{
    return options().bankWidth().get();
}

void
RiverCarveLayer::setBankHeight(float value)
{
    options().bankHeight() = value;
}

float
RiverCarveLayer::getBankHeight() const
{
    return options().bankHeight().get();
}

GeoHeightField
RiverCarveLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    RiverLayer* rivers = getRiverLayer();
    osg::ref_ptr<const Map> map;
    if (!rivers || !rivers->isOpen() || !_map.lock(map))
        return GeoHeightField::INVALID;

    // Channels are too small to matter on coarse tiles, and a coarse tile would need the
    // hydrology of hundreds of data tiles. (The base class only enforces an explicitly set
    // min_level, not our default.)
    if (key.getLOD() < options().minLevel().get())
        return GeoHeightField::INVALID;

    const unsigned size = ELEVATION_TILE_SIZE;
    const float bankWidth = std::max(getBankWidth(), 0.5f);
    const float bankHeight = getBankHeight();

    std::vector<RiverLayer::WaterSample> samples;
    if (!rivers->sampleWaterGrid(key.getExtent(), size, size, bankWidth, samples, progress))
        return GeoHeightField::INVALID;

    // Natural terrain at this tile from the layers beneath (never ourselves).
    ElevationLayerVector all, base;
    map->getLayers(all);
    for (auto& l : all)
        if (l->isOpen() && dynamic_cast<RiverCarveLayer*>(l.get()) == nullptr)
            base.push_back(l);
    if (base.empty())
        return GeoHeightField::INVALID;

    ElevationPool::WorkingSet ws;
    ws.setElevationLayers(base);
    osg::ref_ptr<ElevationTile> natural;
    if (!map->getElevationPool()->getTile(key, true, natural, &ws, progress) || !natural.valid())
        return GeoHeightField::INVALID;

    osg::ref_ptr<osg::HeightField> hf = new osg::HeightField();
    hf->allocate(size, size);
    for (auto& h : hf->getHeightList())
        h = NO_DATA_VALUE;

    const GeoExtent& ex = key.getExtent();
    const osg::HeightField* nhf = natural->getHeightField();
    const bool sameGrid = nhf && nhf->getNumColumns() == size && nhf->getNumRows() == size &&
        natural->getExtent() == ex;

    auto smoothstep = [](float e0, float e1, float x)
    {
        float t = std::min(std::max((x - e0) / (e1 - e0), 0.0f), 1.0f);
        return t * t * (3.0f - 2.0f * t);
    };

    bool wrote = false;
    for (unsigned row = 0; row < size; ++row)
    {
        for (unsigned col = 0; col < size; ++col)
        {
            const RiverLayer::WaterSample& s = samples[(std::size_t)row * size + col];
            if (!s.valid)
                continue;
            // Water pouring down over the rock leaves the terrain as it is.
            if (s.draped)
                continue;

            float h;
            if (sameGrid)
            {
                h = nhf->getHeight(col, row);
            }
            else
            {
                double x = ex.xMin() + ex.width() * col / (double)(size - 1);
                double y = ex.yMin() + ex.height() * row / (double)(size - 1);
                h = natural->getElevation(x, y).elevation().as(Units::METERS);
            }
            if (h == NO_DATA_VALUE)
                continue;

            float out;
            if (s.water)
            {
                // The bed: never above the designed depth, a hair under the surface at the edge.
                // Above a river's falls it is built up into the shelf the water runs over (the
                // terrain model smears the cliff into a slope that would leave the water hanging).
                const float bed = s.level - std::max(s.depth, 0.0f) - 0.08f;
                out = s.shelf ? bed : std::min(h, bed);
            }
            else
            {
                // Banks: raised just enough to hold the water, fading back into the natural
                // terrain across the bank width. Right at the water the bank is also laid back
                // to a steep-but-not-vertical slope over a few meters, so the channel never
                // ends in a sheer wall; further out the natural terrain is left alone (lowering
                // whole valley sides would cut trenches along every mountain stream).
                // The raise is capped: ground far below the water beside it is a drop-off (a
                // ditch along a terrace edge, say), not a bank to build up into a wall.
                float d = -s.shore;
                float lo = s.level + bankHeight * smoothstep(0.0f, 0.3f * bankWidth, d);
                float w = 1.0f - smoothstep(0.5f * bankWidth, bankWidth, d);
                // (beside the shelf above a fall the banks rise with it, however far)
                float raise = std::max(lo - h, 0.0f);
                out = h + (s.shelf ? raise : std::min(raise, bankHeight + 1.0f)) * w;
                const float lip = std::min(bankWidth, 4.0f);
                float hi = s.level + bankHeight + 1.2f * d;
                float wl = 1.0f - smoothstep(0.4f * lip, lip, d);
                out = out + std::min(hi - out, 0.0f) * wl;
            }
            hf->setHeight(col, row, out);
            wrote = true;
        }
    }

    if (!wrote)
        return GeoHeightField::INVALID;

    return GeoHeightField(hf.get(), ex);
}

//...................................................................
// RiverConstraintLayer

void
RiverConstraintLayer::init()
{
    TerrainConstraintLayer::init();
    if (getName().empty())
        setName("River banks");

    osg::observer_ptr<RiverConstraintLayer> weak(this);
    constraintCallback([weak](const TileKey& key, MeshConstraint& constraint, FilterContext*, ProgressCallback* progress)
        {
            osg::ref_ptr<RiverConstraintLayer> self;
            if (!weak.lock(self)) return;
            RiverLayer* rivers = self->getRiverLayer();
            if (rivers && rivers->isOpen())
            {
                constraint.hasElevation = false;
                constraint.removeInterior = false;
                constraint.removeExterior = false;
                rivers->getWaterOutlines(key, constraint.features, progress);
            }
        });
}

Status
RiverConstraintLayer::openImplementation()
{
    Status rs = options().riverLayer().open(getReadOptions());
    if (rs.isError())
        return rs;
    return TerrainConstraintLayer::openImplementation();
}

void
RiverConstraintLayer::addedToMap(const Map* map)
{
    options().riverLayer().addedToMap(map);
    TerrainConstraintLayer::addedToMap(map);
}

void
RiverConstraintLayer::removedFromMap(const Map* map)
{
    options().riverLayer().removedFromMap(map);
    TerrainConstraintLayer::removedFromMap(map);
}

Config
RiverConstraintLayer::getConfig() const
{
    return TerrainConstraintLayer::getConfig();
}

void
RiverConstraintLayer::setRiverLayer(RiverLayer* value)
{
    options().riverLayer().setLayer(value);
}

RiverLayer*
RiverConstraintLayer::getRiverLayer() const
{
    return options().riverLayer().getLayer();
}
