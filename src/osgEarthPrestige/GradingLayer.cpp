/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#include <osgEarthPrestige/GradingLayer>
#include "Grading.h"
#include <osgEarth/ElevationPool>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Map>
#include <osgEarth/Session>
#include <algorithm>
#include <map>
#include <sstream>

using namespace osgEarth;

namespace osgEarthPrestige
{
REGISTER_OSGEARTH_LAYER_FACTORY("prestige:grading", GradingLayer);

namespace
{
    using namespace Grading;
    constexpr std::size_t maxSamples = 1000000u;

    //! Checks cancellation at preparation, sampling, and rasterization boundaries.
    bool canceled(ProgressCallback* progress) { return progress && progress->isCanceled(); }

    //! Tests finite coordinates before passing user geometry to transforms or indexes.
    bool finite(const osg::Vec3d& p)
    {
        return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
    }

    //! Compares exact geometry, including components and holes; absolute profiles also compare source Z.
    //! A shared feature ID alone cannot identify duplicates when a tiled source returns clipped fragments.
    bool sameGeometry(const Geometry& a, const Geometry& b, bool includeZ)
    {
        if (a.getType() != b.getType() || a.size() != b.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (a[i].x() != b[i].x() || a[i].y() != b[i].y() || (includeZ && a[i].z() != b[i].z())) return false;
        if (a.getType() == Geometry::TYPE_MULTI)
        {
            const auto& ac = static_cast<const MultiGeometry&>(a).getComponents();
            const auto& bc = static_cast<const MultiGeometry&>(b).getComponents();
            if (ac.size() != bc.size()) return false;
            for (std::size_t i = 0; i < ac.size(); ++i)
                if (!sameGeometry(*ac[i], *bc[i], includeZ)) return false;
        }
        else if (a.getType() == Geometry::TYPE_POLYGON)
        {
            const auto& ah = static_cast<const Polygon&>(a).getHoles();
            const auto& bh = static_cast<const Polygon&>(b).getHoles();
            if (ah.size() != bh.size()) return false;
            for (std::size_t i = 0; i < ah.size(); ++i)
                if (!sameGeometry(*ah[i], *bh[i], includeZ)) return false;
        }
        return true;
    }

    //! Tests conservative metric bounds against the tile's ECEF sample bounds.
    bool intersects(const osg::Vec3d& a, const osg::Vec3d& b, double radius,
        const osg::Vec3d& lo, const osg::Vec3d& hi)
    {
        for (unsigned axis = 0; axis < 3; ++axis)
            if (std::max(a[axis], b[axis]) + radius < lo[axis] ||
                std::min(a[axis], b[axis]) - radius > hi[axis]) return false;
        return true;
    }

    //! Builds a ground-meter halo in geographic space before selecting source tiles or bounds.
    //! This avoids projected scale distortion and GeoExtent::expand's total-width (not per-side) convention.
    bool featureQueries(const TileKey& key, const FeatureProfile* profile, double radius, std::vector<Query>& queries)
    {
        if (!profile || !profile->getSRS()) return false;
        const auto* geo = SpatialReference::get("wgs84");
        GeoExtent extent = key.getExtent().transform(geo);
        if (!extent.isValid()) return false;
        const double dy = osg::RadiansToDegrees(radius / 6300000.0);
        const double latitude = std::max(std::abs(extent.yMin()), std::abs(extent.yMax())) + dy;
        const double dx = latitude >= 89.999 ? 180.0 : dy/std::cos(osg::DegreesToRadians(latitude));
        extent.expand(2.0*dx, 2.0*dy);
        if (profile->isTiled())
        {
            const auto* tiling = profile->getTilingProfile();
            unsigned lod = tiling->getEquivalentLOD(key.getProfile(), key.getLOD());
            if (profile->getMaxLevel() >= 0) lod = std::min(lod, static_cast<unsigned>(profile->getMaxLevel()));
            if (lod < static_cast<unsigned>(profile->getFirstLevel())) return true;
            std::vector<TileKey> keys;
            tiling->getIntersectingTiles(extent, lod, keys);
            for (const auto& tile : keys) queries.emplace_back(tile);
        }
        else
        {
            GeoExtent west, east;
            std::vector<GeoExtent> extents;
            if (extent.splitAcrossAntimeridian(west, east)) { extents.push_back(west); extents.push_back(east); }
            else extents.push_back(extent);
            for (const auto& part : extents)
            {
                const auto transformed = part.transform(profile->getSRS());
                if (!transformed.isValid()) return false;
                Query query;
                query.bounds() = transformed.bounds();
                queries.push_back(query);
            }
        }
        return true;
    }

    //! Builds exact grid locations in WGS84 and ECEF. Geographic/Mercator axes are separable, so only
    //! one diagonal needs projection and one sine/cosine pair per column; other projections use a full transform.
    bool makeGrid(const GeoExtent& extent, unsigned size, std::vector<osg::Vec3d>& geo,
        std::vector<osg::Vec3d>& world, osg::Vec3d& lo, osg::Vec3d& hi)
    {
        const auto* geographic = SpatialReference::get("wgs84");
        const auto* srs = extent.getSRS();
        const auto& ellipsoid = geographic->getEllipsoid();
        const bool separable = srs->isHorizEquivalentTo(geographic) ||
            srs->isHorizEquivalentTo(SpatialReference::get("spherical-mercator")) ||
            (srs->isMercator() && srs->getGeodeticSRS()->isHorizEquivalentTo(geographic));
        geo.resize(size*size);
        world.resize(size*size);
        if (separable)
        {
            std::vector<osg::Vec3d> diagonal;
            std::vector<double> cosine(size), sine(size);
            for (unsigned i = 0; i < size; ++i)
                diagonal.emplace_back(extent.xMin()+extent.width()*i/(size-1),
                    extent.yMin()+extent.height()*i/(size-1), 0.0);
            if (!srs->transform(diagonal, geographic)) return false;
            for (unsigned i = 0; i < size; ++i)
            {
                cosine[i] = std::cos(osg::DegreesToRadians(diagonal[i].x()));
                sine[i] = std::sin(osg::DegreesToRadians(diagonal[i].x()));
            }
            for (unsigned row = 0; row < size; ++row)
            {
                const auto meridian = ellipsoid.geodeticToGeocentric(osg::Vec3d(0.0, diagonal[row].y(), 0.0));
                for (unsigned col = 0; col < size; ++col)
                {
                    const unsigned index = row*size+col;
                    geo[index].set(diagonal[col].x(), diagonal[row].y(), 0.0);
                    world[index].set(meridian.x()*cosine[col], meridian.x()*sine[col], meridian.z());
                }
            }
        }
        else
        {
            for (unsigned row = 0; row < size; ++row)
            for (unsigned col = 0; col < size; ++col)
                geo[row*size+col].set(extent.xMin()+extent.width()*col/(size-1),
                    extent.yMin()+extent.height()*row/(size-1), 0.0);
            if (!srs->transform(geo, geographic)) return false;
            for (std::size_t i = 0; i < geo.size(); ++i)
            {
                geo[i].z() = 0.0;
                world[i] = ellipsoid.geodeticToGeocentric(geo[i]);
            }
        }
        for (const auto& point : world)
        {
            if (!finite(point)) return false;
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                lo[axis] = std::min(lo[axis], point[axis]);
                hi[axis] = std::max(hi[axis], point[axis]);
            }
        }
        return true;
    }

    struct Path
    {
        std::vector<osg::Vec3d> world;
        std::vector<double> station, heights;
    };

    //! Owns one paging job's inputs and scratch; no mutable state is shared between workers.
    struct Compiler
    {
        const GradingLayer::Options& options;
        const SpatialReference* geographic = SpatialReference::get("wgs84");
        ElevationPool* pool;
        ElevationPool::WorkingSet workingSet;
        ProgressCallback* progress;
        osg::Vec3d lo, hi;
        std::size_t sampleCount = 0;
        Surface surface;
        std::shared_ptr<Junctions> junctions = std::make_shared<Junctions>();
        std::map<std::string, unsigned> profileGroups;

        //! Retains the caller-owned pool for this stack lifetime and selects only preceding elevation layers.
        Compiler(const GradingLayer::Options& o, ElevationPool* p, const ElevationLayerVector& layers,
            ProgressCallback* callback, const osg::Vec3d& minimum, const osg::Vec3d& maximum) :
            options(o), pool(p), progress(callback), lo(minimum), hi(maximum)
        {
            workingSet.setElevationLayers(layers);
        }

        //! Samples a compact batch in the map SRS at a fixed physical resolution, independent of output LOD.
        bool sample(std::vector<osg::Vec3d>& positions)
        {
            if (positions.empty()) return !canceled(progress);
            if (canceled(progress) || positions.size() > maxSamples - sampleCount) return false;
            sampleCount += positions.size();
            if (!geographic->transform(positions, pool->getMapSRS())) return false;
            const int count = pool->sampleMapCoords(positions.begin(), positions.end(),
                Distance(options.elevationResolution().get(), Units::METERS), &workingSet, progress);
            return count >= 0 && !canceled(progress);
        }

        //! Copies source XY onto WGS84's zero-height ellipsoid; never changes cursor-owned features or uses source Z.
        bool position(const osg::Vec3d& input, const SpatialReference* srs, osg::Vec3d& world) const
        {
            osg::Vec3d xy(input.x(), input.y(), 0.0), geo;
            if (!finite(xy) || !srs->transform(xy, geographic, geo) || !finite(geo) || std::abs(geo.y()) > 90.0)
                return false;
            geo.z() = 0.0;
            return geographic->transformToWorld(geo, world) && finite(world);
        }

        //! Returns lon/lat on the reference ellipsoid from a chord position, discarding chord sag.
        osg::Vec3d onSurface(const osg::Vec3d& world) const
        {
            auto geo = geographic->getEllipsoid().geocentricToGeodetic(world);
            geo.z() = 0.0;
            return geo;
        }

        //! Preserves a solved piecewise-linear profile in feature-datum heights, converted to ellipsoid meters.
        //! Rejects invalid/vertical segments; keeps complete source stations and never resamples or smooths Z.
        bool absoluteRoad(const Geometry& geometry, const SpatialReference* srs, double radius, double buffer,
            unsigned group, Surface* destination, double offset)
        {
            if (!destination || geometry.size() < 2 || geometry.size() > maxSamples-sampleCount) return false;
            sampleCount += geometry.size();
            std::vector<osg::Vec3d> world, geographicPoints;
            std::vector<double> stations;
            world.reserve(geometry.size()); geographicPoints.reserve(geometry.size());
            for (const auto& input : geometry)
            {
                if (canceled(progress) || !finite(input) || !validHeight(input.z())) return false;
                osg::Vec3d p, geo;
                if (!position(input, srs, p) || !srs->transform(input, geographic, geo) || !finite(geo) ||
                    !validHeight(geo.z())) return false;
                stations.push_back(world.empty() ? 0.0 : stations.back()+(p-world.back()).length());
                geo.z() += offset;
                world.push_back(p); geographicPoints.push_back(geo);
            }
            for (std::size_t i = 1; i < world.size(); ++i)
            {
                const double length = (world[i]-world[i-1]).length();
                if (length > 100000.0) return false;
                if (length < 1e-6)
                {
                    if (std::abs(geographicPoints[i].z()-geographicPoints[i-1].z()) > 1e-6) return false;
                    continue;
                }
                const double sag = length*length / (4.0*geographic->getEllipsoid().getRadiusPolar());
                if (!intersects(world[i-1], world[i], radius+buffer+sag, lo, hi)) continue;
                Segment segment;
                segment.a = world[i-1]; segment.b = world[i];
                segment.ha = geographicPoints[i-1].z(); segment.hb = geographicPoints[i].z();
                segment.radius = radius; segment.buffer = buffer; segment.group = group;
                segment.stationA = stations[i-1]; segment.stationB = stations[i];
                segment.profileLength = stations.back(); segment.transition = options.profileTransition().get();
                destination->add(segment);
            }
            return true;
        }

        //! Compiles only road spans near this tile, keeping a physical smoothing halo and original station locations.
        //! Work is bounded per request. Invalid geometry aborts the tile; missing elevation leaves gaps untouched.
        bool road(const Geometry& geometry, const SpatialReference* srs, double radius, double buffer, unsigned group,
            Surface* destination = nullptr)
        {
            if (geometry.size() < 2 || radius <= 0.0) return true;
            Path path;
            const double spacing = options.sampleSpacing().get();
            const double halo = radius + buffer + options.smoothingDistance().get() + 2.0*spacing;
            double station = 0.0;
            osg::Vec3d a;
            if (!position(geometry.front(), srs, a)) return false;
            // Station gaps separate runs when an intervening part of the road is outside this tile's halo.
            for (std::size_t i = 1; i < geometry.size(); ++i)
            {
                if (canceled(progress)) return false;
                osg::Vec3d b;
                if (!position(geometry[i], srs, b)) return false;
                const double length = (b-a).length();
                if (length < 1e-6) { a = b; continue; }
                // Chord sag is bounded conservatively for regional data; long arcs are explicitly rejected.
                if (length > 100000.0) return false;
                const double sag = length*length / (4.0*geographic->getEllipsoid().getRadiusPolar());
                if (intersects(a, b, halo+sag, lo, hi))
                {
                    const unsigned count = static_cast<unsigned>(std::ceil(length/spacing));
                    for (unsigned j = 0; j <= count; ++j)
                    {
                        const double t = double(j)/count;
                        const auto geo = onSurface(a + (b-a)*t);
                        const auto world = geographic->getEllipsoid().geodeticToGeocentric(geo);
                        const double s = station + length*t;
                        if (!path.station.empty() && path.station.back() == s) continue;
                        if (!intersects(world, world, halo, lo, hi)) continue;
                        if (path.world.size() >= maxSamples) return false;
                        path.world.push_back(world);
                        path.station.push_back(s);
                    }
                }
                station += length;
                a = b;
            }
            if (path.world.size() < 2) return true;
            std::vector<osg::Vec3d> samples;
            samples.reserve(path.world.size());
            for (const auto& world : path.world) samples.push_back(onSurface(world));
            if (!sample(samples)) return false;
            path.heights.resize(samples.size(), NO_DATA_VALUE);
            const double smoothing = options.smoothingDistance().get();
            // A symmetric triangular filter preserves affine grades away from endpoints. Invalid samples
            // split the profile: never bridge an elevation-data gap or a cropped-out road span.
            std::size_t first = 0;
            while (first < samples.size())
            {
                if (!validHeight(samples[first].z())) { ++first; continue; }
                std::size_t end = first+1;
                while (end < samples.size() && validHeight(samples[end].z()) &&
                    path.station[end]-path.station[end-1] <= spacing*1.01) ++end;
                for (std::size_t i = first; i < end; ++i)
                {
                    if (canceled(progress)) return false;
                    double sum = 0.0, weight = 0.0;
                    auto beginIt = std::lower_bound(path.station.begin()+first, path.station.begin()+end,
                        path.station[i]-smoothing);
                    for (std::size_t j = beginIt-path.station.begin(); j < end; ++j)
                    {
                        const double distance = std::abs(path.station[j]-path.station[i]);
                        if (path.station[j] > path.station[i]+smoothing) break;
                        const double w = smoothing > 0.0 ? std::max(0.0, 1.0-distance/smoothing) : 1.0;
                        // Integration weights prevent densely digitized vertices from dominating the filter.
                        const double cell = end-first == 1 ? 1.0 : 0.5 *
                            (path.station[std::min(j+1, end-1)]-path.station[j > first ? j-1 : first]);
                        sum += samples[j].z()*w*cell;
                        weight += w*cell;
                    }
                    path.heights[i] = weight > 0.0 ? sum/weight : samples[i].z();
                }
                first = end;
            }
            for (std::size_t i = 1; i < path.world.size(); ++i)
            {
                if (path.station[i]-path.station[i-1] > spacing*1.01 ||
                    !intersects(path.world[i-1], path.world[i], radius+buffer, lo, hi)) continue;
                Segment s;
                s.a = path.world[i-1]; s.b = path.world[i];
                s.ha = path.heights[i-1]; s.hb = path.heights[i];
                s.radius = radius; s.buffer = buffer; s.group = group;
                s.extendStart = i > 1 && validHeight(path.heights[i-2]) &&
                    path.station[i-1]-path.station[i-2] <= spacing*1.01;
                s.extendEnd = i+1 < path.world.size() && validHeight(path.heights[i+1]) &&
                    path.station[i+1]-path.station[i] <= spacing*1.01;
                (destination ? *destination : surface).add(s);
            }
            return true;
        }

        //! Fits one common junction plane from its own centerline terrain samples, independently of output tiles.
        //! Only explicit member profiles acquire the transition; disconnected nearby roads remain unaffected.
        bool junction(const std::vector<const Geometry*>& lines, const std::vector<const Polygon*>& polygons,
            const SpatialReference* srs, const std::vector<unsigned>& members)
        {
            if (polygons.empty() || polygons.front()->empty()) return false;
            osg::Vec3d origin;
            if (!position(polygons.front()->front(), srs, origin)) return false;
            Junction j;
            j.worldToLocal = osg::Matrixd::inverse(geographic->getEllipsoid().geocentricToLocalToWorld(origin));
            j.members = members;
            std::sort(j.members.begin(), j.members.end());
            j.members.erase(std::unique(j.members.begin(), j.members.end()), j.members.end());
            j.transition = options.junctionTransition().get();
            osg::Vec3d minimum(DBL_MAX, DBL_MAX, DBL_MAX), maximum(-DBL_MAX, -DBL_MAX, -DBL_MAX);
            for (const auto* input : polygons)
            {
                osg::ref_ptr<Polygon> polygon = static_cast<Polygon*>(input->clone());
                GeometryIterator rings(polygon.get(), true);
                while (rings.hasMore()) for (auto& p : *rings.next())
                {
                    osg::Vec3d world;
                    if (!position(p, srs, world) || (world-origin).length() > 100000.0) return false;
                    for (unsigned axis = 0; axis < 3; ++axis)
                    {
                        minimum[axis] = std::min(minimum[axis], world[axis]);
                        maximum[axis] = std::max(maximum[axis], world[axis]);
                    }
                    p = world*j.worldToLocal;
                    p.z() = 0.0;
                }
                j.footprints.push_back(polygon);
            }
            const double span = (maximum-minimum).length();
            const double pad = j.transition + span*span/geographic->getEllipsoid().getRadiusPolar();
            minimum -= osg::Vec3d(pad, pad, pad);
            maximum += osg::Vec3d(pad, pad, pad);
            if (!intersects(minimum, maximum, 0.0, lo, hi)) return true;

            std::vector<osg::Vec3d> samples, local;
            std::vector<double> weights;
            // Sampling is anchored to complete source segments, never to a tile's clipped portion.
            const double spacing = std::min(5.0, options.sampleSpacing().get());
            for (const auto* line : lines)
            {
                for (std::size_t i = 1; i < line->size(); ++i)
                {
                    if (canceled(progress)) return false;
                    osg::Vec3d a, b;
                    if (!position((*line)[i-1], srs, a) || !position((*line)[i], srs, b)) return false;
                    const double length = (b-a).length();
                    if (length > 100000.0) return false;
                    if (length < 1e-6 || !intersects(a, b, 2.0, minimum, maximum)) continue;
                    const unsigned count = static_cast<unsigned>(std::ceil(length/spacing));
                    for (unsigned k = 0; k <= count; ++k)
                    {
                        const auto geo = onSurface(a+(b-a)*(double(k)/count));
                        const auto p = geographic->getEllipsoid().geodeticToGeocentric(geo)*j.worldToLocal;
                        if (j.distance2(p) > 4.0) continue;
                        if (samples.size() >= maxSamples) return false;
                        samples.push_back(geo);
                        local.push_back(p);
                        weights.push_back(length/count*(k == 0 || k == count ? 0.5 : 1.0));
                    }
                }
            }
            if (!sample(samples)) return false;
            double weight = 0.0, x = 0.0, y = 0.0, h = 0.0;
            unsigned valid = 0;
            for (std::size_t i = 0; i < samples.size(); ++i) if (validHeight(samples[i].z()))
            {
                weight += weights[i];
                x += weights[i]*local[i].x(); y += weights[i]*local[i].y(); h += weights[i]*samples[i].z();
                ++valid;
            }
            // Never fill a missing elevation region from an absent or single terrain sample.
            if (valid < 3 || weight <= 0.0) return true;
            x /= weight; y /= weight; h /= weight;
            double xx = 0.0, xy = 0.0, yy = 0.0, xh = 0.0, yh = 0.0;
            for (std::size_t i = 0; i < samples.size(); ++i) if (validHeight(samples[i].z()))
            {
                const double dx = local[i].x()-x, dy = local[i].y()-y, dh = samples[i].z()-h;
                xx += weights[i]*dx*dx; xy += weights[i]*dx*dy; yy += weights[i]*dy*dy;
                xh += weights[i]*dx*dh; yh += weights[i]*dy*dh;
            }
            // A tiny ridge term stabilizes collinear approaches without inventing an unconstrained cross-slope.
            const double regularization = std::max(1e-8, (xx+yy)*1e-8);
            xx += regularization; yy += regularization;
            const double determinant = xx*yy-xy*xy;
            if (!(determinant > 0.0)) return true;
            j.plane.x() = (xh*yy-yh*xy)/determinant;
            j.plane.y() = (yh*xx-xh*xy)/determinant;
            j.plane.z() = h-j.plane.x()*x-j.plane.y()*y;
            if (!finite(j.plane)) return false;
            junctions->add(j, minimum, maximum);
            return true;
        }

        //! Creates a round level pad for each point, sampling its center once.
        bool points(const Geometry& geometry, const SpatialReference* srs, double radius, double buffer, unsigned& group)
        {
            for (const auto& point : geometry)
            {
                osg::Vec3d world;
                if (!position(point, srs, world)) return false;
                if (!intersects(world, world, radius+buffer, lo, hi)) continue;
                std::vector<osg::Vec3d> samples(1, onSurface(world));
                if (!sample(samples)) return false;
                Segment segment;
                segment.a = segment.b = world;
                segment.ha = segment.hb = samples.front().z();
                segment.radius = radius; segment.buffer = buffer; segment.group = group++;
                surface.add(segment);
            }
            return true;
        }

        //! Masks a supplied centerline profile, or flattens to one interior sample when no profile is supplied.
        //! Edges are local tangent-plane segments; polygons spanning more than 100 km are unsupported.
        bool polygon(const Polygon& input, const SpatialReference* srs, double buffer, unsigned group,
            std::shared_ptr<const Surface> profile = nullptr)
        {
            if (input.size() < 3) return true;
            osg::Vec3d origin;
            if (!position(input.front(), srs, origin)) return false;
            const auto localToWorld = geographic->getEllipsoid().geocentricToLocalToWorld(origin);
            const auto worldToLocal = osg::Matrixd::inverse(localToWorld);
            osg::ref_ptr<Polygon> polygon = static_cast<Polygon*>(input.clone());
            GeometryIterator rings(polygon.get(), true);
            osg::Vec3d minimum(DBL_MAX, DBL_MAX, DBL_MAX), maximum(-DBL_MAX, -DBL_MAX, -DBL_MAX);
            while (rings.hasMore())
            {
                auto* ring = rings.next();
                for (auto& p : *ring)
                {
                    osg::Vec3d world;
                    if (!position(p, srs, world) || (world-origin).length() > 100000.0) return false;
                    for (unsigned axis = 0; axis < 3; ++axis)
                    {
                        minimum[axis] = std::min(minimum[axis], world[axis]);
                        maximum[axis] = std::max(maximum[axis], world[axis]);
                    }
                    p = world * worldToLocal;
                    p.z() = 0.0;
                }
            }
            const double span = (maximum-minimum).length();
            const double padding = buffer + span*span/geographic->getEllipsoid().getRadiusPolar();
            minimum -= osg::Vec3d(padding, padding, padding);
            maximum += osg::Vec3d(padding, padding, padding);
            if (!intersects(minimum, maximum, 0.0, lo, hi)) return true;

            if (profile)
            {
                Pad pad;
                pad.polygon = polygon; pad.worldToLocal = worldToLocal;
                pad.buffer = buffer; pad.group = group; pad.profile = profile;
                pad.useProfileAlpha = options.profileElevation().get() == "absolute";
                surface.add(pad, minimum, maximum);
                return true;
            }

            // Try the bounds center first; scan between vertex ordinates to avoid vertex/ray degeneracy.
            osg::Vec3d anchor = polygon->getBounds().center();
            if (!polygon->contains2D(anchor.x(), anchor.y()))
            {
                std::vector<double> ys;
                ConstGeometryIterator allRings(polygon.get(), true);
                while (allRings.hasMore()) for (const auto& p : *allRings.next()) ys.push_back(p.y());
                std::sort(ys.begin(), ys.end());
                ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
                bool found = false;
                for (std::size_t i = 1; i < ys.size() && !found; ++i)
                {
                    if (canceled(progress)) return false;
                    const double y = 0.5*(ys[i-1]+ys[i]);
                    std::vector<double> xs;
                    ConstGeometryIterator edgesByRing(polygon.get(), true);
                    while (edgesByRing.hasMore())
                    {
                        ConstSegmentIterator edges(edgesByRing.next(), true);
                        while (edges.hasMore())
                        {
                            const auto& edge = edges.next();
                            const auto& a = edge.first; const auto& b = edge.second;
                            if ((a.y() > y) != (b.y() > y))
                                xs.push_back(a.x()+(b.x()-a.x())*(y-a.y())/(b.y()-a.y()));
                        }
                    }
                    std::sort(xs.begin(), xs.end());
                    for (std::size_t j = 1; j < xs.size(); ++j)
                    {
                        anchor.set(0.5*(xs[j-1]+xs[j]), y, 0.0);
                        if (polygon->contains2D(anchor.x(), anchor.y())) { found = true; break; }
                    }
                }
                if (!found) return true;
            }
            anchor.z() = 0.0;
            std::vector<osg::Vec3d> samples(1, onSurface(anchor*localToWorld));
            if (!sample(samples)) return false;
            Pad pad;
            pad.polygon = polygon; pad.worldToLocal = worldToLocal;
            pad.height = samples.front().z(); pad.buffer = buffer; pad.group = group;
            surface.add(pad, minimum, maximum);
            return true;
        }
    };
}

Config GradingLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    featureSource().set(conf, "features");
    conf.set("line_width", lineWidth());
    conf.set("buffer_width", bufferWidth());
    conf.set("max_line_width", maxLineWidth());
    conf.set("max_buffer_width", maxBufferWidth());
    conf.set("sample_spacing", sampleSpacing());
    conf.set("smoothing_distance", smoothingDistance());
    conf.set("elevation_resolution", elevationResolution());
    conf.set("road_footprints", roadFootprints());
    conf.set("junction", junction());
    conf.set("profile_id_attribute", profileIdAttribute());
    conf.set("junction_transition", junctionTransition());
    conf.set("profile_elevation", profileElevation());
    conf.set("profile_transition", profileTransition());
    conf.set("profile_offset", profileOffset());
    return conf;
}

void GradingLayer::Options::fromConfig(const Config& conf)
{
    featureSource().get(conf, "features");
    conf.get("line_width", lineWidth());
    conf.get("buffer_width", bufferWidth());
    conf.get("max_line_width", maxLineWidth());
    conf.get("max_buffer_width", maxBufferWidth());
    conf.get("sample_spacing", sampleSpacing());
    conf.get("smoothing_distance", smoothingDistance());
    conf.get("elevation_resolution", elevationResolution());
    conf.get("road_footprints", roadFootprints());
    conf.get("junction", junction());
    conf.get("profile_id_attribute", profileIdAttribute());
    conf.get("junction_transition", junctionTransition());
    conf.get("profile_elevation", profileElevation());
    conf.get("profile_transition", profileTransition());
    conf.get("profile_offset", profileOffset());
}

void GradingLayer::setFeatureSource(FeatureSource* source) { options().featureSource().setLayer(source); }
FeatureSource* GradingLayer::getFeatureSource() const { return options().featureSource().getLayer(); }

void GradingLayer::init()
{
    ElevationLayer::init();
    // Output depends on preceding elevation, whose revision is not part of this layer's persistent cache key.
    options().cachePolicy() = CachePolicy::NO_CACHE;
    options().l2CacheSize() = 0u;
}

Status GradingLayer::openImplementation()
{
    const auto& o = options();
    if ((o.profileElevation().get() != "terrain" && o.profileElevation().get() != "absolute") ||
        (o.profileElevation().get() == "absolute" && !o.roadFootprints().get()))
        return Status(Status::ConfigurationError, "Absolute profile elevation requires paired road footprints");
    if (!std::isfinite(o.maxLineWidth().get()) || o.maxLineWidth().get() <= 0.0 ||
        o.maxLineWidth().get() > 10000.0 || !std::isfinite(o.maxBufferWidth().get()) ||
        o.maxBufferWidth().get() < 0.0 || o.maxBufferWidth().get() > 10000.0 ||
        !std::isfinite(o.sampleSpacing().get()) || o.sampleSpacing().get() < 1.0 || o.sampleSpacing().get() > 100.0 ||
        !std::isfinite(o.smoothingDistance().get()) || o.smoothingDistance().get() < 0.0 ||
        o.smoothingDistance().get() > 1000.0 || !std::isfinite(o.elevationResolution().get()) ||
        o.elevationResolution().get() <= 0.0 || !std::isfinite(o.junctionTransition().get()) ||
        o.junctionTransition().get() <= 0.0 || o.junctionTransition().get() > 1000.0 ||
        !std::isfinite(o.profileTransition().get()) || o.profileTransition().get() < 0.0 ||
        o.profileTransition().get() > 1000.0 ||
        getTileSize() < 2u || getTileSize() > 1024u)
        return Status(Status::ConfigurationError, "Invalid grading widths, spacing, smoothing, resolution, or tile size");
    Status status = ElevationLayer::openImplementation();
    if (status.isError()) return status;
    if (!options().featureSource().isSet())
        return Status(Status::ConfigurationError, "Grading requires a feature source");
    status = options().featureSource().open(getReadOptions());
    if (status.isError()) return status;
    if (!getProfile())
    {
        setProfile(Profile::create(Profile::GLOBAL_GEODETIC));
        // Avoid enumerating continent-scale road datasets for grids too coarse to represent road corridors.
        // Applications with a regional profile keep their own LOD convention.
        if (!options().minLevel().isSet()) options().minLevel().init(12u);
    }
    if (getProfile()->getSRS()->getVerticalDatum())
        return Status(Status::ConfigurationError, "Grading output uses ellipsoid heights; omit the output vdatum");
    addDataExtent(DataExtent(getProfile()->getExtent(), getMinLevel(), getMaxDataLevel()));
    return Status::NoError;
}

void GradingLayer::addedToMap(const Map* map)
{
    ElevationLayer::addedToMap(map);
    options().featureSource().addedToMap(map);
    if (!getFeatureSource() || !getFeatureSource()->isOpen())
        setStatus(Status(Status::ConfigurationError, "Grading feature source could not be opened or resolved"));
    _map = map;
}

void GradingLayer::removedFromMap(const Map* map)
{
    _map = nullptr;
    options().featureSource().removedFromMap(map);
    ElevationLayer::removedFromMap(map);
}

GeoHeightField GradingLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    osg::ref_ptr<const Map> map;
    auto* source = getFeatureSource();
    if (!_map.lock(map) || !source || !source->isOpen() || canceled(progress))
        return GeoHeightField::INVALID;
    ElevationLayerVector layers, preceding;
    map->getLayers(layers);
    bool found = false;
    for (const auto& layer : layers)
    {
        if (layer.get() == this) { found = true; break; }
        if (layer->isOpen()) preceding.push_back(layer);
    }
    // An empty WorkingSet selects ALL map layers in ElevationPool, which would recursively call us.
    if (!found || preceding.empty()) return GeoHeightField::INVALID;

    const auto& extent = key.getExtent();
    const unsigned size = getTileSize();
    std::vector<osg::Vec3d> grid, world;
    osg::Vec3d lo(DBL_MAX, DBL_MAX, DBL_MAX), hi(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    std::unique_ptr<Compiler> compiler;
    std::vector<Query> queries;
    if (!featureQueries(key, source->getFeatureProfile(),
        0.5*options().maxLineWidth().get()+options().maxBufferWidth().get()+
        (options().roadFootprints().get() ? options().junctionTransition().get() : 0.0), queries))
        return GeoHeightField(Status(Status::ConfigurationError, "Cannot transform flattening feature query"));
    osg::ref_ptr<Session> session = new Session(map);
    FilterContext context(session.get());
    // Retain read-only features for this job so their geometry survives each source cursor.
    std::map<FeatureID, std::vector<osg::ref_ptr<const Feature>>> seen;
    const bool absolute = options().profileElevation().get() == "absolute";
    unsigned group = 0;
    for (const auto& query : queries)
    {
        auto cursor = source->createFeatureCursor(query, {}, &context, progress);
        if (!cursor) return GeoHeightField::INVALID;
        while (cursor->hasMore())
        {
            if (canceled(progress)) return GeoHeightField::INVALID;
            auto* feature = cursor->nextFeature();
            if (!feature || !feature->getGeometry() || !feature->getSRS()) continue;
            // Keep distinct fragments with a shared ID; discard only matching geometry in the same SRS.
            if (feature->getFID() != 0)
            {
                auto& previous = seen[feature->getFID()];
                const bool duplicate = std::any_of(previous.begin(), previous.end(), [&](const auto& other)
                {
                    return (absolute ? feature->getSRS()->isEquivalentTo(other->getSRS()) :
                        feature->getSRS()->isHorizEquivalentTo(other->getSRS())) &&
                        sameGeometry(*feature->getGeometry(), *other->getGeometry(), absolute);
                });
                if (duplicate) continue;
                previous.emplace_back(feature);
            }
            const double width = options().lineWidth()->eval(feature, context);
            const double buffer = options().bufferWidth()->eval(feature, context);
            if (!std::isfinite(width) || !std::isfinite(buffer) || width < 0.0 || buffer < 0.0) continue;
            if (!compiler)
            {
                if (!makeGrid(extent, size, grid, world, lo, hi)) return GeoHeightField::INVALID;
                compiler.reset(new Compiler(options(), map->getElevationPool(), preceding, progress, lo, hi));
            }
            const double radius = 0.5*std::min(width, options().maxLineWidth().get());
            const double shoulder = std::min(buffer, options().maxBufferWidth().get());
            if (options().roadFootprints().get())
            {
                // Each feature explicitly pairs its complete centerlines with its pavement footprint.
                // Search width bounds profile support only; it never expands the pavement mask.
                auto profile = std::make_shared<Surface>(absolute ? nullptr : compiler->junctions, absolute);
                std::vector<const Polygon*> footprints;
                std::vector<const Geometry*> lines;
                ConstGeometryIterator components(feature->getGeometry(), false);
                while (components.hasMore())
                {
                    const auto* component = components.next();
                    if (component->getType() == Geometry::TYPE_POLYGON)
                        footprints.push_back(static_cast<const Polygon*>(component));
                    else if (component->getType() == Geometry::TYPE_LINESTRING)
                        lines.push_back(component);
                    else return GeoHeightField(Status(Status::ConfigurationError,
                        "Road footprints require only centerlines and polygons in each feature"));
                }
                if (lines.empty() || footprints.empty() || radius <= 0.0)
                    return GeoHeightField(Status(Status::ConfigurationError,
                        "Road footprints require centerlines, pavement polygons, and positive profile search width"));
                std::vector<std::string> ids;
                if (!options().profileIdAttribute()->empty())
                {
                    std::istringstream input(feature->getString(options().profileIdAttribute().get()));
                    std::string id;
                    while (std::getline(input, id, ',')) ids.push_back(id);
                    if (ids.size() != lines.size() || std::any_of(ids.begin(), ids.end(),
                        [](const std::string& value) { return value.empty(); }))
                        return GeoHeightField(Status(Status::ConfigurationError,
                            "Profile IDs must pair one nonempty ID with each complete centerline"));
                }
                const bool commonJunction = options().junction()->eval(feature, context);
                if (absolute && (commonJunction || lines.size() != 1))
                    return GeoHeightField(Status(Status::ConfigurationError,
                        "Absolute road footprints require one continuous solved centerline and no junction refit"));
                if (commonJunction && ids.empty())
                    return GeoHeightField(Status(Status::ConfigurationError, "Junction grading requires profile IDs"));
                std::vector<unsigned> members;
                const double offset = absolute ? options().profileOffset()->eval(feature, context) : 0.0;
                if (!std::isfinite(offset) || std::abs(offset) > 10000.0)
                    return GeoHeightField(Status(Status::ConfigurationError, "Invalid absolute profile offset"));
                for (std::size_t i = 0; i < lines.size(); ++i)
                {
                    unsigned member = group++;
                    if (!ids.empty())
                        member = compiler->profileGroups.emplace(ids[i], member).first->second;
                    members.push_back(member);
                    const bool ok = absolute ?
                        compiler->absoluteRoad(*lines[i], feature->getSRS(), radius, shoulder, member, profile.get(), offset) :
                        compiler->road(*lines[i], feature->getSRS(), radius, shoulder, member, profile.get());
                    if (!ok)
                        return GeoHeightField::INVALID;
                }
                if (commonJunction && !compiler->junction(lines, footprints, feature->getSRS(), members))
                    return GeoHeightField::INVALID;
                const unsigned owner = group++;
                for (const auto* footprint : footprints)
                    if (!compiler->polygon(*footprint, feature->getSRS(), shoulder, owner, profile))
                        return GeoHeightField::INVALID;
                continue;
            }
            ConstGeometryIterator parts(feature->getGeometry(), false);
            while (parts.hasMore())
            {
                const auto* part = parts.next();
                bool ok = true;
                if (part->getType() == Geometry::TYPE_LINESTRING)
                    ok = compiler->road(*part, feature->getSRS(), radius, shoulder, group++);
                else if (part->getType() == Geometry::TYPE_POINTSET || part->getType() == Geometry::TYPE_POINT)
                    ok = compiler->points(*part, feature->getSRS(), radius, shoulder, group);
                else if (part->getType() == Geometry::TYPE_POLYGON)
                    ok = compiler->polygon(*static_cast<const Polygon*>(part), feature->getSRS(), shoulder, group++);
                else if (part->getType() == Geometry::TYPE_RING)
                {
                    osg::ref_ptr<Geometry> polygon = part->cloneAs(Geometry::TYPE_POLYGON);
                    ok = compiler->polygon(*static_cast<const Polygon*>(polygon.get()), feature->getSRS(), shoulder, group++);
                }
                if (!ok) return canceled(progress) ? GeoHeightField::INVALID :
                    GeoHeightField(Status(Status::ResourceUnavailable,
                        "Grading input transform/sample failed, geometry exceeds 100 km, or sample budget exceeded"));
            }
        }
    }
    if (!compiler || compiler->surface.empty() || canceled(progress)) return GeoHeightField::INVALID;
    std::vector<Grading::Constraint> constraints;
    std::vector<Grading::Candidate> scratch;
    scratch.reserve(8);
    std::vector<osg::Vec3d> natural;
    std::vector<std::size_t> pixels;
    auto hf = HeightFieldUtils::createReferenceHeightField(extent, size, size, 0u, false, NO_DATA_VALUE);
    bool wrote = false;
    for (unsigned row = 0; row < size; row += 8)
    for (unsigned col = 0; col < size; col += 8)
    {
        if (canceled(progress)) return GeoHeightField::INVALID;
        osg::Vec3d minimum(DBL_MAX, DBL_MAX, DBL_MAX), maximum(-DBL_MAX, -DBL_MAX, -DBL_MAX);
        const unsigned endRow = std::min(row+8, size), endCol = std::min(col+8, size);
        // Bound actual grid samples, rather than projecting just the corners of a nonlinear map projection.
        for (unsigned r = row; r < endRow; ++r)
        for (unsigned c = col; c < endCol; ++c)
        {
            const auto& p = world[r*size+c];
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                minimum[axis] = std::min(minimum[axis], p[axis]);
                maximum[axis] = std::max(maximum[axis], p[axis]);
            }
        }
        if (!compiler->surface.intersects(minimum, maximum)) continue;
        for (unsigned r = row; r < endRow; ++r)
        for (unsigned c = col; c < endCol; ++c)
        {
            const std::size_t i = r*size+c;
            const auto constraint = compiler->surface.query(world[i], scratch);
            if (constraint.alpha == 1.0 && Grading::validHeight(constraint.height))
            {
                hf->getHeightList()[i] = static_cast<float>(constraint.height);
                wrote = true;
            }
            else if (constraint.alpha > 0.0)
            {
                pixels.push_back(i);
                natural.push_back(grid[i]);
                constraints.push_back(constraint);
            }
        }
    }
    if (!compiler->sample(natural)) return GeoHeightField::INVALID;
    for (std::size_t j = 0; j < pixels.size(); ++j)
    {
        const auto& c = constraints[j];
        if (Grading::validHeight(natural[j].z()) && Grading::validHeight(c.height))
        {
            hf->getHeightList()[pixels[j]] = static_cast<float>(natural[j].z() + c.alpha*(c.height-natural[j].z()));
            wrote = true;
        }
    }
    return wrote && !canceled(progress) ? GeoHeightField(hf, extent) : GeoHeightField::INVALID;
}
}
