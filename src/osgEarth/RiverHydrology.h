/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#pragma once

// Internal header: river hydrology and water mesh generation shared by the river
// layers. Not part of the public API.

#include <osgEarth/RiverLayer>
#include <osgEarth/Feature>
#include <osgEarth/Geometry>
#include <osg/Node>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>

namespace osgEarth { namespace RiverDetail
{
    //! Snapshot of the settings a hydrology build depends on. Copied per build so
    //! background jobs never read layer options that the GUI is changing.
    struct BuildSettings
    {
        std::map<std::string, RiverLayer::WaterwayClass> classes;
        float meshResolution = 10.0f;  // longest water mesh edge (real meters)
        float levelSmoothing = 35.0f;  // gaussian sigma of the water level along channels (m)
        float depthScale = 1.0f;       // multiplier on all water depths
        bool lakes = true;             // build still water from polygons without channels
        bool waterfalls = true;        // find waterfalls and let the water fall there
    };

    //! Fills "heights" (row-major, cols x rows, row 0 at y0) with natural terrain
    //! heights at spherical-mercator points (x0 + i*dx, y0 + j*dy). Returns false on
    //! failure or cancelation. Must be callable from any thread.
    using HeightSampler = std::function<bool(
        double x0, double y0, double dx, double dy,
        unsigned cols, unsigned rows,
        std::vector<float>& heights,
        ProgressCallback* progress)>;

    //! Uniform bucket grid of item indices over a 2D box. Items are registered
    //! over their bounding boxes; queries may report an item more than once.
    class GridIndex
    {
    public:
        //! Prepares an empty index over [bmin, bmax] with square cells.
        void init(const osg::Vec2d& bmin, const osg::Vec2d& bmax, double cellSize)
        {
            _min = bmin;
            _cell = std::max(cellSize, 1e-6);
            _nx = std::max(1, (int)std::ceil((bmax.x() - bmin.x()) / _cell));
            _ny = std::max(1, (int)std::ceil((bmax.y() - bmin.y()) / _cell));
            _cells.assign((std::size_t)_nx * (std::size_t)_ny, std::vector<int>());
            _count = 0;
        }

        //! Registers an item over the box spanned by two corner points. Items outside the
        //! indexed box are clamped into the border cells.
        void insert(int item, const osg::Vec2d& a, const osg::Vec2d& b)
        {
            int x0, y0, x1, y1;
            range(std::min(a.x(), b.x()), std::min(a.y(), b.y()), std::max(a.x(), b.x()), std::max(a.y(), b.y()),
                x0, y0, x1, y1);
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x)
                    _cells[(std::size_t)y * _nx + x].push_back(item);
            ++_count;
        }

        //! Calls func(item) for every item registered in a cell overlapping the query box.
        template<typename F>
        void query(const osg::Vec2d& qmin, const osg::Vec2d& qmax, F&& func) const
        {
            if (_count == 0) return;
            int x0, y0, x1, y1;
            range(qmin.x(), qmin.y(), qmax.x(), qmax.y(), x0, y0, x1, y1);
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x)
                    for (int item : _cells[(std::size_t)y * _nx + x])
                        func(item);
        }

        //! Whether anything was inserted.
        bool empty() const { return _count == 0; }

    private:
        osg::Vec2d _min;
        double _cell = 1.0;
        int _nx = 0, _ny = 0;
        int _count = 0;
        std::vector<std::vector<int>> _cells;

        //! Cell index range covering a box, clamped to the grid.
        void range(double xmin, double ymin, double xmax, double ymax, int& x0, int& y0, int& x1, int& y1) const
        {
            x0 = clampCell((int)std::floor((xmin - _min.x()) / _cell), _nx);
            y0 = clampCell((int)std::floor((ymin - _min.y()) / _cell), _ny);
            x1 = clampCell((int)std::floor((xmax - _min.x()) / _cell), _nx);
            y1 = clampCell((int)std::floor((ymax - _min.y()) / _cell), _ny);
        }

        static int clampCell(int v, int n) { return v < 0 ? 0 : v >= n ? n - 1 : v; }
    };

    //! A waterway centerline resampled at regular spacing, carrying its smoothed
    //! water-surface profile. Points run in the direction of flow.
    struct Channel
    {
        std::vector<osg::Vec2d> p;      // resampled points (local mercator)
        std::vector<osg::Vec2f> dir;    // unit flow direction at each point (east, north)
        std::vector<float> s;           // arc length from the first point (real meters)
        std::vector<float> level;       // water surface height at each point (m)
        std::vector<float> slope;       // downhill surface gradient along the flow (m/m)
        std::vector<float> halfWidth;   // half the surface width (real meters)
        std::vector<float> depth;       // centerline depth (m)
        std::vector<float> speed;       // surface current including the slope effect (m/s)
        std::vector<float> rapids;      // whitewater propensity of the waterway class (0..1)
        std::vector<char> ownsArea;     // 1 where the channel runs far enough through a mapped water
                                        // area to set the water level there (see TileField::sample)
        std::vector<int> fall;          // index into TileField::falls for points from a fall's lip to
                                        // its foot, else -1
        std::vector<float> churn;       // extra whitewater around falls: the plunge pool below, the
                                        // rush toward the lip above (0..1)
        FeatureID fid = 0;
        bool user = false;
    };

    //! A waterfall or steep cascade: the stretch of a channel between the lip, where the
    //! water leaves its bed, and the foot, where it lands in the pool below.
    //! A river's falls (across a mapped water area) drop as a sheer curtain where the terrain
    //! crosses the middle of the drop: the terrain model smears the cliff into a slope, so the
    //! bed is built up to the upper pool above the curtain and carved down to the plunge pool
    //! below it. A stream mapped only as a line pours down over the rock instead.
    struct Fall
    {
        int channel = -1;       // index into TileField::channels
        int lip = 0;            // channel point index of the lip
        int foot = 0;           // channel point index of the foot (at least two past the lip)
        float top = 0.0f;       // water level at the lip (m)
        float bottom = 0.0f;    // water level at the foot (m)
        float width = 0.0f;     // width of the falling water at the foot (m)
        bool mapped = false;    // marked in the data (waterway=waterfall), not just found in the terrain
        bool curtain = false;   // drops as a curtain (across a mapped water area), else pours over the rock
    };

    //! One straight piece of a channel or shoreline (local mercator).
    struct Segment
    {
        osg::Vec2d a, b;
        int owner = -1;  // channel index (channel segments) or -1 (shore segments)
        int index = 0;   // index of the first point within the owner's point list
    };

    //! Hydrology of one data tile: everything needed to evaluate water level,
    //! depth, flow and shore distance anywhere in or near the tile. Immutable once
    //! built, so it can be shared by the renderer and the terrain layers.
    class TileField : public osg::Referenced
    {
    public:
        TileKey key;                    // data tile (spherical mercator)
        osg::Vec2d origin;              // mercator coordinates of the tile's south-west corner
        osg::Vec2d size;                // tile size (mercator meters)
        double k = 1.0;                 // real meters per mercator meter at the tile center
        double buffer = 0.0;            // mercator margin around the tile covered by the data

        std::vector<Channel> channels;
        std::vector<Segment> channelSegs;
        GridIndex channelGrid;
        std::vector<Fall> falls;

        std::vector<Segment> shoreSegs;
        GridIndex shoreGrid;
        double bandY0 = 0.0, bandHeight = 1.0;
        std::vector<std::vector<int>> bands;  // shore segment indices per horizontal band

        // the mapped water areas alone (lakes and river polygons, without the ribbons of waterways
        // mapped only as lines): ring edges in the same bands as above, and their real banks indexed
        std::vector<Segment> areaSegs;
        GridIndex areaGrid;
        std::vector<std::vector<int>> areaBands;

        // natural terrain heights over the tile + buffer (row-major, local mercator lattice)
        osg::Vec2d demOrigin;
        double demSpacing = 1.0;
        unsigned demCols = 0u, demRows = 0u;
        std::vector<float> dem;

        // water area clipped to the tile plus a small overlap (local mercator)
        osg::ref_ptr<Geometry> water;

        // ground within WET_BANK_WIDTH of the water, outside it, clipped like "water"
        osg::ref_ptr<Geometry> bank;

        BuildSettings settings;

        //! Whether the tile or its surroundings contain any water.
        bool hasWater() const { return !shoreSegs.empty() || !channels.empty(); }

        //! Whether the tile itself has water to draw.
        bool hasMesh() const { return water.valid() && water->getTotalPointCount() > 0; }

        //! Whether the tile has wet bank ground to draw.
        bool hasBank() const { return bank.valid() && bank->getTotalPointCount() > 0; }

        //! Evaluates the water at a local mercator point. insideHint: 1 = the caller knows
        //! the point is inside the water, -1 = outside, 0 = test it. Points farther than
        //! "margin" real meters outside the water are not evaluated. Returns out.valid.
        bool sample(const osg::Vec2d& p, int insideHint, float margin, RiverLayer::WaterSample& out) const;

        //! Point-in-water test (local mercator).
        bool inside(const osg::Vec2d& p) const;

        //! Whether a local mercator point lies in a mapped water area (a lake or river polygon, as
        //! opposed to the ribbon of a waterway mapped only as a line). Points within "tolerance" real
        //! meters of an area's bank count as inside, so that points on the shore classify reliably.
        bool inArea(const osg::Vec2d& p, double tolerance) const;

        //! Distance (real meters) from a local mercator point to the nearest shoreline,
        //! up to maxDistance; also returns the nearest shore point.
        double shoreDistance(const osg::Vec2d& p, double maxDistance, osg::Vec2d* nearest = nullptr) const;

        //! Natural terrain height (bilinear) at a local mercator point.
        float terrainHeight(const osg::Vec2d& p) const;

        //! Steepness of the natural terrain at a local mercator point: the magnitude of the
        //! height gradient (m/m, so 1 is a 45 degree slope).
        float terrainSteepness(const osg::Vec2d& p) const;

        //! Converts a local mercator point to absolute mercator coordinates.
        osg::Vec2d toMercator(const osg::Vec2d& local) const { return local + origin; }

        //! Converts absolute mercator coordinates to a local point.
        osg::Vec2d toLocal(double mx, double my) const { return osg::Vec2d(mx, my) - origin; }

        using Polyline = std::vector<osg::Vec2d>;
        using Polylines = std::vector<Polyline>;

        //! Bank lines and channel centerlines owned by this field's tile (local mercator),
        //! simplified to "tolerance" (local mercator units). Results are cached under
        //! "toleranceKey", which must identify the tolerance. Thread-safe.
        std::shared_ptr<const Polylines> constraintLines(int toleranceKey, double tolerance) const;

    private:
        float lakeLevel(const osg::Vec2d& p) const;

        mutable std::mutex _linesMutex;
        mutable std::map<int, std::shared_ptr<const Polylines>> _lines;
    };

    //! Builds the hydrology of one data tile from waterway features (already in
    //! spherical mercator) covering the tile and its surroundings. Returns null on
    //! cancelation or when the terrain heights could not be read (try again later);
    //! returns an empty field (hasWater() == false) when there is no water.
    osg::ref_ptr<TileField> buildField(
        const TileKey& key,
        const FeatureList& features,
        const BuildSettings& settings,
        const HeightSampler& heights,
        double bufferMeters,
        ProgressCallback* progress);

    //! Builds the water surface mesh of a field's tile in the map's coordinate system, plus
    //! the wet-bank band (a separate geometry under a range LOD). Returns null when the tile
    //! has neither. Thread-safe.
    osg::ref_ptr<osg::Node> buildMesh(
        const TileField& field,
        const SpatialReference* mapSRS,
        ProgressCallback* progress,
        unsigned* triangleCount);

    //! Builds the spray and mist thrown up by the falls and the strongest rapids of a field's tile
    //! (each belongs to the tile holding its source point) as camera-facing quads that the spray
    //! shader animates on its own (no per-frame CPU work). Returns null when nothing sprays.
    //! Reports the number of particles. Thread-safe.
    osg::ref_ptr<osg::Node> buildSpray(
        const TileField& field,
        const SpatialReference* mapSRS,
        unsigned* particleCount);

    //! Bank lines and channel centerlines of a field's tile that touch a box (absolute spherical
    //! mercator), simplified (see TileField::constraintLines), as line features for terrain mesh
    //! constraints. Each segment is reported by exactly one field, and cuts at the context
    //! border are not banks, so they are left out.
    void getOutlines(const TileField& field, const osg::Vec2d& clipMin, const osg::Vec2d& clipMax,
        int toleranceKey, double tolerance, FeatureList& out);

    //! Classifies a feature as a flowing waterway centerline and returns its physical
    //! parameters (width/depth/speed tags override the class defaults).
    bool getChannelParams(const Feature* f, const BuildSettings& settings, RiverLayer::WaterwayClass& out);

    //! Whether a feature is a water area (lake, pond, river area, reservoir).
    bool isWaterArea(const Feature* f);

    //! Parses a length tag such as "12", "12 m" or "40'" into meters (0 if unparsable).
    double parseLength(const std::string& value);

    //! Vertex attribute locations used by the water mesh.
    constexpr unsigned FLOW_ATTRIB = 6u;  // vec4: flow uv/s (xy), speed m/s, turbulence
    constexpr unsigned DATA_ATTRIB = 7u;  // vec4: shore distance m, depth m, across, meters per uv
                                          // (wet bank: -distance from the water m, lift m, distance/width)

    //! Width (real meters) of the band of wet ground drawn along the shore outside the water.
    constexpr float WET_BANK_WIDTH = 3.0f;

    //! How far (m) the wet band reaches over the water, covering the waterline at oblique views.
    constexpr float WET_BANK_INSET = 1.5f;

    //! Outline simplification (m) for the wet band; the inset absorbs the error.
    constexpr float WET_BANK_SIMPLIFY = 0.5f;

    //! Height (m) above the water up to which bank ground looks wet; rough water adds splash.
    constexpr float WET_BANK_HEIGHT = 0.3f;

    //! Distance (m) beyond a tile's bounds at which its wet band stops drawing.
    constexpr float WET_BANK_RANGE = 1500.0f;
} }
