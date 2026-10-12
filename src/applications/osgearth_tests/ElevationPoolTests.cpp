/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#include <osgEarth/catch.hpp>
#include <osgEarth/ElevationPool>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Map>
#include <osgEarth/Progress>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <cmath>
#include <stdexcept>

using namespace osgEarth;

namespace
{
    class PoolTestLayer : public ElevationLayer
    {
    public:
        META_LayerNoOptions(osgEarth, PoolTestLayer, ElevationLayer, pool_test);

        //! Configures synthetic terrain without caches that could conceal pool reloads.
        void init() override
        {
            ElevationLayer::init();
            setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
            options().tileSize() = 9u;
            options().maxDataLevel() = 8u;
            options().l2CacheSize() = 0u;
            setCachePolicy(CachePolicy::NO_CACHE);
        }

        mutable std::atomic<unsigned> creates{0u};
        std::atomic<float> base{100.0f};
        bool flat = false;
        bool unavailable = false;
        unsigned delayMilliseconds = 0u;
        std::function<void(const TileKey&, ProgressCallback*)> onCreate;

    protected:
        //! Generates a planar field, optionally delaying or recursively sampling for concurrency tests.
        GeoHeightField createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const override
        {
            ++creates;
            if (onCreate)
                onCreate(key, progress);
            if (delayMilliseconds)
                std::this_thread::sleep_for(std::chrono::milliseconds(delayMilliseconds));
            if (unavailable || (progress && progress->isCanceled()))
                return GeoHeightField::INVALID;
            osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(
                key.getExtent(), getTileSize(), getTileSize(), 0u, false);
            for (unsigned r = 0; r < getTileSize(); ++r)
            {
                for (unsigned c = 0; c < getTileSize(); ++c)
                {
                    const double x = key.getExtent().xMin() + c * hf->getXInterval();
                    const double y = key.getExtent().yMin() + r * hf->getYInterval();
                    hf->setHeight(c, r, base.load() + (flat ? 0.0f : float(0.5*x + 0.25*y)));
                }
            }
            return GeoHeightField(hf.get(), key.getExtent());
        }
    };

    struct PoolFixture
    {
        osg::ref_ptr<Map> map;
        osg::ref_ptr<PoolTestLayer> layer;

        //! Creates a geodetic test map, optionally omitting advertised coverage metadata.
        explicit PoolFixture(bool extents = true)
        {
            map = new Map();
            map->setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
            map->setElevationInterpolation(INTERP_BILINEAR);
            layer = new PoolTestLayer();
            if (extents)
                layer->setDataExtents({DataExtent(map->getProfile()->getExtent(), 0u, 8u)});
            map->addLayer(layer.get());
        }

        //! Returns a fixed interior point away from tile boundaries.
        GeoPoint point() const { return GeoPoint(map->getSRS(), -73.1, 40.2); }

        //! Independently evaluates the planar terrain at the test point.
        double expected() const { return 100.0 + 0.5*point().x() + 0.25*point().y(); }
    };

    class StartGate
    {
        std::mutex mutex;
        std::condition_variable changed;
        unsigned arrived = 0u;
    public:
        //! Starts a fixed number of threads together, with a timeout to avoid hanging failed tests.
        void wait(unsigned participants)
        {
            std::unique_lock<std::mutex> lock(mutex);
            ++arrived;
            changed.notify_all();
            changed.wait_for(lock, std::chrono::seconds(5), [&] { return arrived >= participants; });
        }
    };

    class AtomicProgress : public ProgressCallback
    {
    public:
        std::atomic<bool> stop{false};

        //! Lets test threads cancel a request without racing ProgressCallback's mutable flag.
        bool canceled() const override { return stop.load(); }
    };

    //! Waits for a bounded concurrency-test condition, returning false instead of hanging on failure.
    template<typename Predicate>
    bool waitUntil(Predicate predicate)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!predicate())
        {
            if (std::chrono::steady_clock::now() >= deadline)
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return true;
    }

    //! Fills shared cache capacity with different tiles while leaving a working set untouched.
    void churn(ElevationPool* pool, const Profile* profile)
    {
        for (unsigned i = 0; i < 160u; ++i)
        {
            osg::ref_ptr<ElevationTile> tile;
            pool->getTile(TileKey(8u, i, 20u, profile), false, tile, nullptr, nullptr);
        }
    }
}

//! Checks analytic heights, reported resolution, and agreement across the sampling APIs.
TEST_CASE("ElevationPool sampling APIs agree", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    ElevationPool::WorkingSet ws;
    const Distance resolution(0.01, Units::DEGREES);
    auto sample = pool->getSample(f.point(), resolution, &ws);
    REQUIRE(sample.hasData());
    REQUIRE(std::abs(sample.elevation().getValue() - (f.expected())) <= 0.01);
    REQUIRE(sample.resolution().getValue() > 0.0);

    std::vector<osg::Vec3d> xyz(3u, osg::Vec3d(f.point().x(), f.point().y(), 0.0));
    REQUIRE(pool->sampleMapCoords(xyz.begin(), xyz.end(), resolution, &ws, nullptr) == 3);
    for (const auto& p : xyz)
        REQUIRE(std::abs(p.z() - (sample.elevation().getValue())) <= 1e-5);

    std::vector<osg::Vec4d> xyzw(3u, osg::Vec4d(f.point().x(), f.point().y(), 0.0, 0.01));
    REQUIRE(pool->sampleMapCoords(xyzw.begin(), xyzw.end(), &ws, nullptr) == 3);
    for (const auto& p : xyzw)
        REQUIRE(std::abs(p.z() - (sample.elevation().getValue())) <= 1e-5);

    ElevationPool::Envelope envelope;
    REQUIRE(pool->prepareEnvelope(envelope, f.point(), resolution, &ws));
    REQUIRE(envelope.sampleMapCoords(xyz.begin(), xyz.end(), nullptr) == 3);
    REQUIRE(std::abs(xyz.front().z() - (sample.elevation().getValue())) <= 1e-5);
}

//! Verifies projected input uses geographic latitude when converting meter resolutions.
TEST_CASE("ElevationPool projected and geographic queries select the same resolution", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    GeoPoint projected = f.point().transform(SpatialReference::get("spherical-mercator"));
    REQUIRE(projected.isValid());
    auto geographicSample = pool->getSample(f.point(), Distance(2500.0, Units::METERS), nullptr);
    auto projectedSample = pool->getSample(projected, Distance(2500.0, Units::METERS), nullptr);
    REQUIRE(geographicSample.hasData());
    REQUIRE(projectedSample.hasData());
    REQUIRE(projectedSample.resolution().getValue() == Approx(geographicSample.resolution().getValue()));
    REQUIRE(std::abs(projectedSample.elevation().getValue() - (geographicSample.elevation().getValue())) <= 1e-4);
}

//! Exercises flat-tile encoding, including the default 16-bit representation.
TEST_CASE("ElevationPool preserves constant elevation tiles", "[ElevationPool]")
{
    PoolFixture f;
    f.layer->flat = true;
    f.layer->base = 42.0f;
    auto sample = f.map->getElevationPool()->getSample(f.point(), nullptr);
    REQUIRE(sample.hasData());
    REQUIRE(std::isfinite(sample.elevation().getValue()));
    REQUIRE(sample.elevation().getValue() == Approx(42.0));
}

//! Unknown coverage remains queryable; an empty map produces failure values without invalid shifts.
TEST_CASE("ElevationPool handles unknown coverage and empty maps", "[ElevationPool]")
{
    PoolFixture f(false);
    auto* pool = f.map->getElevationPool();
    REQUIRE(pool->getSample(f.point(), nullptr).hasData());
    std::vector<osg::Vec3d> points(1u, osg::Vec3d(f.point().x(), f.point().y(), 0.0));
    REQUIRE(pool->sampleMapCoords(points.begin(), points.end(), Distance(0.01, Units::DEGREES), nullptr, nullptr) == 1);
    REQUIRE(std::abs(points.front().z() - (f.expected())) <= 0.01);

    f.map->removeLayer(f.layer.get());
    ElevationPool::Envelope envelope;
    REQUIRE(pool->prepareEnvelope(envelope, f.point(), Distance(0.01, Units::DEGREES)));
    REQUIRE(envelope.getLOD() == -1);
    REQUIRE(envelope.sampleMapCoords(points.begin(), points.end(), nullptr, -9999.0f) == 0);
    REQUIRE(points.front().z() == -9999.0);
}

//! Both batch overloads retain tiles in their caller's working set across shared-cache eviction.
TEST_CASE("ElevationPool batches populate working sets", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    pool->setCacheBudget(1024u*1024u);
    ElevationPool::WorkingSet ws;
    SECTION("fixed resolution")
    {
        std::vector<osg::Vec3d> points(1u, osg::Vec3d(f.point().x(), f.point().y(), 0.0));
        REQUIRE(pool->sampleMapCoords(points.begin(), points.end(), Distance(0.01, Units::DEGREES), &ws, nullptr) == 1);
    }
    SECTION("per-point resolution")
    {
        std::vector<osg::Vec4d> points(1u, osg::Vec4d(f.point().x(), f.point().y(), 0.0, 0.01));
        REQUIRE(pool->sampleMapCoords(points.begin(), points.end(), &ws, nullptr) == 1);
    }
    churn(pool, f.map->getProfile());
    unsigned before = f.layer->creates.load();
    REQUIRE(pool->getSample(f.point(), Distance(0.01, Units::DEGREES), &ws).hasData());
    REQUIRE(f.layer->creates.load() == before);
}

//! Content revisions, extent changes, and interpolation updates invalidate the appropriate cached results.
TEST_CASE("ElevationPool observes terrain changes", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    ElevationPool::WorkingSet ws;
    auto original = pool->getSample(f.point(), Distance(0.01, Units::DEGREES), &ws);
    REQUIRE(original.hasData());
    f.layer->base = 200.0f;
    f.layer->dirty();
    auto changed = pool->getSample(f.point(), Distance(0.01, Units::DEGREES), &ws);
    REQUIRE(std::abs(changed.elevation().getValue() - (original.elevation().getValue() + 100.0)) <= 0.01);

    f.layer->setDataExtents({DataExtent(GeoExtent(f.map->getSRS(), 0.0, 0.0, 10.0, 10.0), 0u, 8u)});
    REQUIRE_FALSE(pool->getSample(f.point(), nullptr).hasData());
    f.layer->setDataExtents({DataExtent(f.map->getProfile()->getExtent(), 0u, 8u)});
    REQUIRE(pool->getSample(f.point(), nullptr).hasData());

    osg::ref_ptr<ElevationTile> nearest, bilinear;
    TileKey key(0u, 0u, 0u, f.map->getProfile());
    f.map->setElevationInterpolation(INTERP_NEAREST);
    REQUIRE(pool->getTile(key, false, nearest, &ws, nullptr));
    f.map->setElevationInterpolation(INTERP_BILINEAR);
    REQUIRE(pool->getTile(key, false, bilinear, &ws, nullptr));
    REQUIRE(nearest.get() != bilinear.get());
    REQUIRE(std::abs(nearest->getRawElevationUV(0.02, 0.02) - bilinear->getRawElevationUV(0.02, 0.02)) > 0.1);
}

//! A custom layer selection can include open layers that are not members of the map.
TEST_CASE("ElevationPool supports independent working-set layers", "[ElevationPool]")
{
    PoolFixture f;
    osg::ref_ptr<PoolTestLayer> custom = new PoolTestLayer();
    custom->base = 300.0f;
    REQUIRE(custom->open().isOK());
    ElevationPool::WorkingSet ws;
    ElevationLayerVector selection;
    selection.push_back(custom.get());
    ws.setElevationLayers(selection);
    auto sample = f.map->getElevationPool()->getSample(f.point(), Distance(0.01, Units::DEGREES), &ws);
    REQUIRE(sample.hasData());
    REQUIRE(std::abs(sample.elevation().getValue() - (f.expected() + 200.0)) <= 0.01);
    ws.setElevationLayers({});
    REQUIRE(std::abs(f.map->getElevationPool()->getSample(f.point(), &ws).elevation().getValue() - (f.expected())) <= 0.01);
}

//! Concurrent cold requests share one tile construction and return the same object.
TEST_CASE("ElevationPool coalesces concurrent tile creation", "[ElevationPool]")
{
    PoolFixture f;
    f.layer->delayMilliseconds = 50u;
    StartGate gate;
    std::vector<osg::ref_ptr<ElevationTile>> results(8u);
    std::vector<std::thread> threads;
    for (unsigned i = 0; i < results.size(); ++i)
    {
        threads.emplace_back([&, i]
        {
            gate.wait(8u);
            f.map->getElevationPool()->getTile(TileKey(8u, 100u, 100u, f.map->getProfile()),
                false, results[i], nullptr, nullptr);
        });
    }
    for (auto& thread : threads)
        thread.join();
    REQUIRE(results.front().valid());
    for (const auto& result : results)
        REQUIRE(result.get() == results.front().get());
    REQUIRE(f.layer->creates.load() == 1u);
}

//! Fallback aliases reuse canonical ancestor tiles without satisfying exact-only requests.
TEST_CASE("ElevationPool reuses fallback ancestors", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    const TileKey parent(8u, 100u, 100u, f.map->getProfile());
    osg::ref_ptr<ElevationTile> native, fallback, strict;
    REQUIRE(pool->getTile(parent, false, native, nullptr, nullptr));
    unsigned before = f.layer->creates.load();
    for (unsigned i = 0; i < 4u; ++i)
    {
        TileKey child = parent.createChildKey(i);
        REQUIRE(pool->getTile(child, true, fallback, nullptr, nullptr));
        REQUIRE(fallback.get() == native.get());
        REQUIRE_FALSE(pool->getTile(child, false, strict, nullptr, nullptr));
    }
    REQUIRE(f.layer->creates.load() == before);
}

//! Bounds pool-owned terrain payload and weak metadata during traversal of new terrain.
TEST_CASE("ElevationPool bounds cache retention", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    pool->setCacheBudget(1024u*1024u);
    churn(pool, f.map->getProfile());
    auto stats = pool->getCacheStats();
    REQUIRE(stats.retainedBytes <= 1024u*1024u);
    REQUIRE(stats.weakEntries <= 64u);
    REQUIRE(stats.inFlight == 0u);
    REQUIRE(stats.builds >= 160u);
    pool->setCacheBudget(0u);
    REQUIRE(pool->getCacheStats().retainedTiles == 0u);
}

//! External consumers cannot make the pool's weak lookup metadata grow without bound.
TEST_CASE("ElevationPool bounds live weak metadata", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    pool->setCacheBudget(1024u*1024u);
    std::vector<osg::ref_ptr<ElevationTile>> external;
    for (unsigned i = 0; i < 96u; ++i)
    {
        osg::ref_ptr<ElevationTile> tile;
        REQUIRE(pool->getTile(TileKey(8u, i, 20u, f.map->getProfile()), false, tile, nullptr, nullptr));
        external.push_back(tile);
    }
    REQUIRE(pool->getCacheStats().weakEntries <= 64u);
    REQUIRE(pool->getCacheStats().retainedBytes <= 1024u*1024u);
}

//! Queries retain index ownership while repeated map refreshes replace the published snapshot.
TEST_CASE("ElevationPool supports concurrent map refresh", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    std::atomic<bool> good{true};
    std::thread reader([&]
    {
        ElevationPool::WorkingSet ws;
        for (unsigned i = 0; i < 200u; ++i)
        {
            auto sample = pool->getSample(f.point(), Distance(0.01, Units::DEGREES), &ws);
            if (!sample.hasData() || std::abs(sample.elevation().getValue() - f.expected()) > 0.01)
                good = false;
        }
    });
    for (unsigned i = 0; i < 100u; ++i)
        pool->setMap(f.map.get());
    reader.join();
    REQUIRE(good.load());
}

//! Cancellation is honored even on cache hits and leaves later requests usable.
TEST_CASE("ElevationPool honors cancellation on warm queries", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    ElevationPool::WorkingSet ws;
    REQUIRE(pool->getSample(f.point(), &ws).hasData());
    osg::ref_ptr<ProgressCallback> progress = new ProgressCallback();
    progress->cancel();
    REQUIRE_FALSE(pool->getSample(f.point(), &ws, progress).hasData());
    std::vector<osg::Vec3d> points(128u, osg::Vec3d(f.point().x(), f.point().y(), 0.0));
    REQUIRE(pool->sampleMapCoords(points.begin(), points.end(), Distance(0.01, Units::DEGREES),
        &ws, progress) == -1);
    progress->reset();
    REQUIRE(pool->getSample(f.point(), &ws, progress).hasData());
}

//! Queued work owns its sampling state after the submitting sampler is destroyed.
TEST_CASE("AsyncElevationSampler survives destruction with pending work", "[ElevationPool]")
{
    PoolFixture f;
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false, release = false;
    f.layer->onCreate = [&](const TileKey&, ProgressCallback*)
    {
        std::unique_lock<std::mutex> lock(mutex);
        entered = true;
        changed.notify_all();
        changed.wait_for(lock, std::chrono::seconds(5), [&] { return release; });
    };
    jobs::future<ElevationSample> first, second;
    {
        AsyncElevationSampler sampler(f.map.get(), 1u);
        first = sampler.getSample(f.point());
        {
            std::unique_lock<std::mutex> lock(mutex);
            changed.wait_for(lock, std::chrono::seconds(5), [&] { return entered; });
        }
        second = sampler.getSample(f.point());
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        release = true;
        changed.notify_all();
    }
    auto a = first.join();
    auto b = second.join();
    REQUIRE(a.hasData());
    REQUIRE(b.hasData());
    REQUIRE(std::abs(b.elevation().getValue() - (f.expected())) <= 0.01);
}

//! One caller's cancellation must not invalidate another caller's shared tile request.
TEST_CASE("ElevationPool isolates cancellation of concurrent callers", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    bool cancelOwner = false;
    SECTION("canceled creator is retried by its waiter") { cancelOwner = true; }
    SECTION("canceled waiter leaves the creator running") { cancelOwner = false; }
    std::atomic<bool> entered{false}, release{false}, waiterDone{false};
    f.layer->onCreate = [&](const TileKey&, ProgressCallback*)
    {
        entered = true;
        waitUntil([&] { return release.load(); });
    };
    osg::ref_ptr<AtomicProgress> cancellation = new AtomicProgress();
    osg::ref_ptr<ElevationTile> first, second;
    TileKey key(8u, 100u, 100u, f.map->getProfile());
    std::thread owner([&]
    {
        pool->getTile(key, false, first, nullptr, cancelOwner ? cancellation.get() : nullptr);
    });
    const bool started = waitUntil([&] { return entered.load(); });
    std::thread waiter([&]
    {
        pool->getTile(key, false, second, nullptr, cancelOwner ? nullptr : cancellation.get());
        waiterDone = true;
    });
    const bool waited = waitUntil([&] { return pool->getCacheStats().waits != 0u; });
    cancellation->stop = true;
    bool canceledWithoutOwner = true;
    if (!cancelOwner)
        canceledWithoutOwner = waitUntil([&] { return waiterDone.load(); });
    release = true;
    owner.join();
    waiter.join();
    REQUIRE(started);
    REQUIRE(waited);
    REQUIRE(canceledWithoutOwner);
    REQUIRE(first.valid() == !cancelOwner);
    REQUIRE(second.valid() == cancelOwner);
    REQUIRE(pool->getCacheStats().inFlight == 0u);
}

//! Procedural layers may synchronously query a lower-priority selection in the same pool.
TEST_CASE("ElevationPool allows recursive layer sampling", "[ElevationPool]")
{
    PoolFixture f;
    ElevationPool::WorkingSet underlying;
    ElevationLayerVector selection;
    selection.push_back(f.layer.get());
    underlying.setElevationLayers(selection);
    osg::ref_ptr<PoolTestLayer> procedural = new PoolTestLayer();
    std::atomic<bool> sampled{false};
    procedural->onCreate = [&](const TileKey& key, ProgressCallback* progress)
    {
        osg::ref_ptr<ElevationTile> tile;
        sampled = f.map->getElevationPool()->getTile(key, true, tile, &underlying, progress);
    };
    f.map->addLayer(procedural.get());
    osg::ref_ptr<ElevationTile> tile;
    REQUIRE(f.map->getElevationPool()->getTile(TileKey(8u, 100u, 100u, f.map->getProfile()),
        false, tile, nullptr, nullptr));
    REQUIRE(sampled.load());
}

//! A failing layer callback releases the in-progress entry so later requests can retry.
TEST_CASE("ElevationPool releases failed creation flights", "[ElevationPool]")
{
    PoolFixture f;
    f.layer->onCreate = [](const TileKey&, ProgressCallback*) { throw std::runtime_error("test failure"); };
    osg::ref_ptr<ElevationTile> tile;
    const TileKey key(8u, 100u, 100u, f.map->getProfile());
    REQUIRE_THROWS(f.map->getElevationPool()->getTile(key, false, tile, nullptr, nullptr));
    REQUIRE(f.map->getElevationPool()->getCacheStats().inFlight == 0u);
    f.layer->onCreate = {};
    REQUIRE(f.map->getElevationPool()->getTile(key, false, tile, nullptr, nullptr));
}

//! Mixed coverage and boundary points retain per-point LOD selection, sentinel and failure behavior.
TEST_CASE("ElevationPool batch selection handles coverage boundaries", "[ElevationPool]")
{
    PoolFixture f;
    f.layer->setDataExtents({
        DataExtent(GeoExtent(f.map->getSRS(), -180.0, -90.0, 0.0, 90.0), 0u, 2u),
        DataExtent(GeoExtent(f.map->getSRS(), 0.0, -90.0, 180.0, 90.0), 0u, 8u)});
    std::vector<osg::Vec4d> points{
        osg::Vec4d(-1.0, 1.0, 0.0, 0.001), osg::Vec4d(1.0, 1.0, 0.0, 0.001),
        osg::Vec4d(-180.0, -90.0, 0.0, 0.001), osg::Vec4d(180.0, 90.0, 0.0, 0.001),
        osg::Vec4d(181.0, 0.0, 0.0, 0.001), osg::Vec4d(0.0, 91.0, 0.0, 0.001),
        osg::Vec4d(0.0, 0.0, 123.0, FLT_MAX)};
    auto* pool = f.map->getElevationPool();
    REQUIRE(pool->sampleMapCoords(points.begin(), points.end(), nullptr, nullptr, -9999.0f) == 4);
    for (unsigned i = 0; i < 4u; ++i)
    {
        auto sample = pool->getSample(GeoPoint(f.map->getSRS(), points[i].x(), points[i].y()),
            Distance(0.001, Units::DEGREES), nullptr);
        REQUIRE(sample.hasData());
        REQUIRE(std::abs(points[i].z() - (sample.elevation().getValue())) <= 1e-4);
    }
    REQUIRE(points[4].z() == -9999.0);
    REQUIRE(points[5].z() == -9999.0);
    REQUIRE(points[6].z() == 123.0);
}

//! A reference point outside coverage does not prevent nearby envelope points from being sampled.
TEST_CASE("ElevationPool envelope can bridge a coverage gap", "[ElevationPool]")
{
    PoolFixture f;
    f.layer->setDataExtents({DataExtent(GeoExtent(f.map->getSRS(), -74.0, 39.0, -72.0, 41.0), 0u, 8u)});
    ElevationPool::Envelope envelope;
    auto* pool = f.map->getElevationPool();
    REQUIRE(pool->prepareEnvelope(envelope, GeoPoint(f.map->getSRS(), -71.0, 40.0),
        Distance(0.01, Units::DEGREES)));
    std::vector<osg::Vec3d> points(1u, osg::Vec3d(f.point().x(), f.point().y(), 0.0));
    REQUIRE(envelope.sampleMapCoords(points.begin(), points.end(), nullptr) == 1);
    REQUIRE(std::abs(points[0].z() - f.expected()) < 0.01);
}

//! Long-lived envelopes release early tiles as they traverse more than their bounded local capacity.
TEST_CASE("ElevationPool envelopes bound their local tile retention", "[ElevationPool]")
{
    PoolFixture f;
    auto* pool = f.map->getElevationPool();
    pool->setCacheBudget(0u);
    const TileKey key(8u, 0u, 100u, f.map->getProfile());
    const auto center = key.getExtent().getCentroid();
    ElevationPool::Envelope envelope;
    REQUIRE(pool->prepareEnvelope(envelope, center, Distance(0.003, Units::DEGREES)));
    std::vector<osg::Vec3d> points(1u, osg::Vec3d(center.x(), center.y(), 0.0));
    REQUIRE(envelope.sampleMapCoords(points.begin(), points.end(), nullptr) == 1);
    osg::observer_ptr<ElevationTile> observer;
    {
        osg::ref_ptr<ElevationTile> tile;
        REQUIRE(pool->getTile(key, false, tile, nullptr, nullptr));
        observer = tile.get();
    }
    points.clear();
    for (unsigned x = 1u; x < 128u; ++x)
    {
        auto point = TileKey(8u, x, 100u, f.map->getProfile()).getExtent().getCentroid();
        points.emplace_back(point.x(), point.y(), 0.0);
    }
    REQUIRE(envelope.sampleMapCoords(points.begin(), points.end(), nullptr) == int(points.size()));
    osg::ref_ptr<ElevationTile> expired;
    REQUIRE_FALSE(observer.lock(expired));
}

//! Wrapped coverage is indexed on both sides of the antimeridian without filling the intervening gap.
TEST_CASE("ElevationPool indexes antimeridian coverage", "[ElevationPool]")
{
    PoolFixture f;
    f.layer->setDataExtents({DataExtent(GeoExtent(f.map->getSRS(), 170.0, -20.0, -170.0, 20.0), 0u, 8u)});
    auto* pool = f.map->getElevationPool();
    REQUIRE(pool->getSample(GeoPoint(f.map->getSRS(), 179.0, 0.0), nullptr).hasData());
    REQUIRE(pool->getSample(GeoPoint(f.map->getSRS(), -179.0, 0.0), nullptr).hasData());
    REQUIRE_FALSE(pool->getSample(GeoPoint(f.map->getSRS(), 0.0, 0.0), nullptr).hasData());
}

//! Layer compositing retains offset semantics and explicit failure results.
TEST_CASE("ElevationPool preserves offset and unavailable-layer behavior", "[ElevationPool]")
{
    PoolFixture f;
    SECTION("offset above base terrain")
    {
        osg::ref_ptr<PoolTestLayer> offset = new PoolTestLayer();
        offset->flat = true;
        offset->base = 10.0f;
        offset->setInterpretValuesAsOffsets(true);
        f.map->addLayer(offset.get());
        auto sample = f.map->getElevationPool()->getSample(f.point(), nullptr);
        REQUIRE(sample.hasData());
        REQUIRE(std::abs(sample.elevation().getValue() - (f.expected() + 10.0)) < 0.01);
    }
    SECTION("unavailable terrain")
    {
        f.layer->unavailable = true;
        auto* pool = f.map->getElevationPool();
        REQUIRE_FALSE(pool->getSample(f.point(), nullptr).hasData());
        std::vector<osg::Vec3d> points(2u, osg::Vec3d(f.point().x(), f.point().y(), 0.0));
        REQUIRE(pool->sampleMapCoords(points.begin(), points.end(), Distance(0.01, Units::DEGREES),
            nullptr, nullptr, -12345.0f) == 0);
        REQUIRE(points.front().z() == -12345.0);
    }
}

//! Angular resolutions on a projected map use geographic latitude in scalar and batch queries.
TEST_CASE("ElevationPool samples projected maps consistently", "[ElevationPool]")
{
    PoolFixture f;
    const GeoPoint geographic = f.point();
    f.map->setProfile(Profile::create("spherical-mercator"));
    const GeoPoint projected = geographic.transform(f.map->getSRS());
    REQUIRE(projected.isValid());
    const Distance resolution(0.01, Units::DEGREES);
    auto* pool = f.map->getElevationPool();
    auto a = pool->getSample(geographic, resolution, nullptr);
    auto b = pool->getSample(projected, resolution, nullptr);
    REQUIRE(a.hasData());
    REQUIRE(b.hasData());
    REQUIRE(a.resolution().getValue() == Approx(b.resolution().getValue()));
    REQUIRE(std::abs(a.elevation().getValue() - b.elevation().getValue()) < 1e-4);
    std::vector<osg::Vec3d> points(1u, osg::Vec3d(projected.x(), projected.y(), 0.0));
    REQUIRE(pool->sampleMapCoords(points.begin(), points.end(), resolution, nullptr, nullptr) == 1);
    REQUIRE(std::abs(points[0].z() - b.elevation().getValue()) < 1e-4);
}

//! Opposing procedural dependencies do not wait on each other's in-progress tiles indefinitely.
TEST_CASE("ElevationPool breaks cross-thread recursive dependencies", "[ElevationPool]")
{
    PoolFixture f;
    osg::ref_ptr<PoolTestLayer> other = new PoolTestLayer();
    f.map->addLayer(other.get());
    ElevationPool::WorkingSet firstSelection, secondSelection;
    ElevationLayerVector firstLayers, secondLayers;
    firstLayers.push_back(f.layer.get());
    secondLayers.push_back(other.get());
    firstSelection.setElevationLayers(firstLayers);
    secondSelection.setElevationLayers(secondLayers);
    auto* pool = f.map->getElevationPool();
    StartGate gate;
    f.layer->onCreate = [&](const TileKey& key, ProgressCallback* progress)
    {
        if (f.layer->creates.load() == 1u)
            gate.wait(2u);
        osg::ref_ptr<ElevationTile> dependency;
        pool->getTile(key, false, dependency, &secondSelection, progress);
    };
    other->onCreate = [&](const TileKey& key, ProgressCallback* progress)
    {
        if (other->creates.load() == 1u)
            gate.wait(2u);
        osg::ref_ptr<ElevationTile> dependency;
        pool->getTile(key, false, dependency, &firstSelection, progress);
    };
    const TileKey key(8u, 100u, 100u, f.map->getProfile());
    osg::ref_ptr<ElevationTile> first, second;
    std::thread a([&] { pool->getTile(key, false, first, &firstSelection, nullptr); });
    std::thread b([&] { pool->getTile(key, false, second, &secondSelection, nullptr); });
    a.join();
    b.join();
    REQUIRE(first.valid());
    REQUIRE(second.valid());
    REQUIRE(pool->getCacheStats().inFlight == 0u);
}
