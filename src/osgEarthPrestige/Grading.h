/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#pragma once

#include <osgEarth/Geometry>
#include <osgEarth/rtree.h>
#include <osg/Matrixd>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace osgEarthPrestige { namespace Grading
{
    //! Accepts finite real elevations only; never interpolates the no-data sentinel.
    inline bool validHeight(double h)
    {
        return std::isfinite(h) && h != NO_DATA_VALUE;
    }

    //! Clamps a fraction without extrapolating.
    inline double unit(double t) { return std::max(0.0, std::min(1.0, t)); }

    //! Returns compact quintic influence with zero first/second derivative at the shoulder edges.
    inline double influence(double distance, double radius, double buffer)
    {
        if (distance <= radius) return 1.0;
        if (buffer <= 0.0 || distance >= radius + buffer) return 0.0;
        const double t = (distance - radius) / buffer;
        return 1.0 - t*t*t*(t*(t*6.0 - 15.0) + 10.0);
    }

    struct Segment
    {
        osg::Vec3d a, b;
        double ha = NO_DATA_VALUE, hb = NO_DATA_VALUE;
        double radius = 0.0, buffer = 0.0;
        unsigned group = 0;
        bool extendStart = false, extendEnd = false;
        double stationA = 0.0, stationB = 0.0, profileLength = 0.0, transition = 0.0;
    };

    class Surface;

    //! Returns squared distance outside a local XY footprint, including its holes; interiors return zero.
    inline double footprintDistance2(const osgEarth::Polygon* polygon, const osg::Vec3d& p)
    {
        if (polygon->contains2D(p.x(), p.y())) return 0.0;
        double d2 = std::numeric_limits<double>::max();
        osgEarth::ConstGeometryIterator rings(polygon, true);
        while (rings.hasMore())
        {
            osgEarth::ConstSegmentIterator edges(rings.next(), true);
            while (edges.hasMore())
            {
                const auto& edge = edges.next();
                osg::Vec3d a(edge.first.x(), edge.first.y(), 0.0);
                osg::Vec3d ab(edge.second.x()-a.x(), edge.second.y()-a.y(), 0.0);
                osg::Vec3d ap(p.x()-a.x(), p.y()-a.y(), 0.0);
                const double t = ab.length2() > 0.0 ? unit((ap*ab)/ab.length2()) : 0.0;
                d2 = std::min(d2, (ap-ab*t).length2());
            }
        }
        return d2;
    }

    struct Junction
    {
        std::vector<osg::ref_ptr<const osgEarth::Polygon>> footprints;
        osg::Matrixd worldToLocal;
        osg::Vec3d plane; // Ellipsoid height = plane.z + plane.x*local.x + plane.y*local.y.
        std::vector<unsigned> members;
        double transition = 30.0;

        //! Returns squared distance to the union of local footprints, respecting their holes.
        double distance2(const osg::Vec3d& local) const
        {
            double result = std::numeric_limits<double>::max();
            for (const auto& polygon : footprints)
                result = std::min(result, footprintDistance2(polygon.get(), local));
            return result;
        }
    };

    //! Shared per-job junction planes, built before querying and used only by explicitly identified profiles.
    class Junctions
    {
    public:
        //! Retains a fully fitted plane and conservative ECEF bounds, including its transition region.
        void add(const Junction& junction, const osg::Vec3d& lo, const osg::Vec3d& hi)
        {
            const unsigned index = static_cast<unsigned>(_junctions.size());
            _junctions.push_back(junction);
            _index.Insert(lo.ptr(), hi.ptr(), index);
        }

        //! Blends a member profile into a common plane with zero derivative at both ends of the transition.
        //! This changes height only; pavement masks and terrain shoulders retain exclusive coverage control.
        double apply(unsigned member, const osg::Vec3d& world, double original) const
        {
            double sum = 0.0, weight = 0.0, alpha = 0.0;
            _index.Search(world.ptr(), world.ptr(), [&](unsigned index)
            {
                const auto& j = _junctions[index];
                if (!std::binary_search(j.members.begin(), j.members.end(), member)) return RTREE_KEEP_SEARCHING;
                const auto p = world*j.worldToLocal;
                const double a = influence(std::sqrt(j.distance2(p)), 0.0, j.transition);
                if (a > 0.0)
                {
                    // An interior plane dominates neighboring transition bands without a discrete priority switch.
                    const double w = a / (1.0-a+1e-8);
                    sum += w*(j.plane.z()+j.plane.x()*p.x()+j.plane.y()*p.y());
                    weight += w;
                    alpha = std::max(alpha, a);
                }
                return RTREE_KEEP_SEARCHING;
            });
            return weight > 0.0 ? original + alpha*(sum/weight-original) : original;
        }

    private:
        std::vector<Junction> _junctions;
        RTree<unsigned, double, 3> _index;
    };

    struct Pad
    {
        osg::ref_ptr<const osgEarth::Polygon> polygon;
        osg::Matrixd worldToLocal;
        double height = NO_DATA_VALUE, buffer = 0.0;
        unsigned group = 0;
        // Optional worker-owned centerline profiles; the polygon supplies coverage, not a constant height.
        std::shared_ptr<const Surface> profile;
        bool useProfileAlpha = false;
    };

    struct Candidate
    {
        unsigned group;
        double distance2, height, alpha;
        double weight = 0.0;
    };

    struct Constraint
    {
        double height = NO_DATA_VALUE, alpha = 0.0;
    };

    //! Worker-owned immutable-after-build geometry. Query scratch belongs to the caller.
    class Surface
    {
    public:
        //! Retains optional shared junction controls; callers finish all mutations before querying this surface.
        explicit Surface(std::shared_ptr<const Junctions> junctions = nullptr, bool nearestSegments = false) :
            _nearestSegments(nearestSegments), _junctions(junctions) { }

        //! Adds a metric ECEF segment, including a zero-length segment for a point pad.
        void add(const Segment& segment)
        {
            if (!validHeight(segment.ha) || !validHeight(segment.hb)) return;
            const unsigned index = static_cast<unsigned>(_segments.size());
            IndexedSegment indexed;
            indexed.segment = segment;
            indexed.ab = segment.b - segment.a;
            indexed.inverseLength2 = indexed.ab.length2() > 0.0 ? 1.0/indexed.ab.length2() : 0.0;
            indexed.outer2 = (segment.radius+segment.buffer)*(segment.radius+segment.buffer);
            _segments.push_back(indexed);
            double lo[3], hi[3];
            const double r = segment.radius + segment.buffer;
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                lo[axis] = std::min(segment.a[axis], segment.b[axis]) - r;
                hi[axis] = std::max(segment.a[axis], segment.b[axis]) + r;
            }
            _index.Insert(lo, hi, index);
        }

        //! Adds an owned local polygon and conservative ECEF bounds; holes remain unflattened interiors.
        void add(const Pad& pad, const osg::Vec3d& lo, const osg::Vec3d& hi)
        {
            if (pad.profile ? pad.profile->empty() : !validHeight(pad.height)) return;
            const unsigned index = static_cast<unsigned>(_pads.size());
            _pads.push_back(pad);
            _padIndex.Insert(lo.ptr(), hi.ptr(), index);
        }

        //! Reports whether any valid terrain constraints were compiled.
        bool empty() const { return _segments.empty() && _pads.empty(); }

        //! Conservatively rejects whole raster blocks before doing individual point queries.
        bool intersects(const osg::Vec3d& lo, const osg::Vec3d& hi) const
        {
            bool hit = false;
            _index.Search(lo.ptr(), hi.ptr(), [&hit](unsigned) { hit = true; return RTREE_STOP_SEARCHING; });
            if (!hit && !_pads.empty())
                _padIndex.Search(lo.ptr(), hi.ptr(), [&hit](unsigned) { hit = true; return RTREE_STOP_SEARCHING; });
            return hit;
        }

        //! Queries intersecting bounds and continuously blends nearby segments within each geometry part.
        //! Shared endpoints are never deleted. Missing endpoint pairs are omitted when building.
        Constraint query(const osg::Vec3d& point, std::vector<Candidate>& scratch) const
        {
            scratch.clear();
            _index.Search(point.ptr(), point.ptr(), [&](unsigned index)
            {
                const auto& indexed = _segments[index];
                const auto& s = indexed.segment;
                const double projected = ((point-s.a)*indexed.ab)*indexed.inverseLength2;
                const double t = unit(projected);
                // Continue internal segment grades through joins. Collinear segments then agree exactly,
                // while weighted corner contributions avoid jumps across the nearest-segment bisector.
                const double heightT = (projected < 0.0 && s.extendStart) || (projected > 1.0 && s.extendEnd) ?
                    projected : t;
                const double d2 = (point - (s.a + indexed.ab*t)).length2();
                if (d2 <= indexed.outer2)
                {
                    double alpha = influence(std::sqrt(d2), s.radius, s.buffer);
                    if (s.transition > 0.0)
                    {
                        const double station = s.stationA + (s.stationB-s.stationA)*t;
                        alpha *= 1.0-influence(std::min(station, s.profileLength-station), 0.0, s.transition);
                    }
                    Candidate value{s.group, d2, s.ha + (s.hb-s.ha)*heightT, alpha};
                    if (_nearestSegments) accumulateNearest(scratch, value);
                    else accumulate(scratch, value);
                }
                return RTREE_KEEP_SEARCHING;
            });
            if (!_pads.empty()) _padIndex.Search(point.ptr(), point.ptr(), [&](unsigned index)
            {
                const auto& pad = _pads[index];
                const auto p = point * pad.worldToLocal;
                const double d2 = footprintDistance2(pad.polygon.get(), p);
                double alpha = influence(std::sqrt(d2), 0.0, pad.buffer);
                double height = pad.height;
                if (alpha > 0.0 && pad.profile)
                {
                    std::vector<Candidate> profileScratch;
                    const auto profile = pad.profile->query(point, profileScratch);
                    height = profile.height;
                    if (pad.useProfileAlpha) alpha *= profile.alpha;
                }
                if (validHeight(height)) accumulate(scratch, {pad.group, d2, height, alpha});
                return RTREE_KEEP_SEARCHING;
            });
            Constraint result;
            double sum = 0.0, weight = 0.0;
            // Normalize per part so digitization density does not give one road priority at a junction.
            for (const auto& candidate : scratch)
            {
                const double w = candidate.alpha / (1.0 + candidate.distance2);
                const double h = _junctions ? _junctions->apply(candidate.group, point, candidate.height) : candidate.height;
                sum += w * h;
                weight += w;
                result.alpha = std::max(result.alpha, candidate.alpha);
            }
            if (weight > 0.0) result.height = sum / weight;
            return result;
        }

    private:
        //! Solved profiles use the nearest station instead of smoothing across unrelated station heights.
        //! Ties retain the first segment; valid continuous paths agree at their shared endpoint.
        static void accumulateNearest(std::vector<Candidate>& candidates, const Candidate& value)
        {
            // Retain zero-weight endpoints so a more distant positive segment cannot defeat the fade.
            for (auto& candidate : candidates)
            {
                if (candidate.group != value.group) continue;
                if (value.distance2 < candidate.distance2) candidate = value;
                return;
            }
            candidates.push_back(value);
        }

        //! Blends compact segment contributions within a part and retains its maximum flattening influence.
        static void accumulate(std::vector<Candidate>& candidates, const Candidate& value)
        {
            if (value.alpha <= 0.0) return;
            const double weight = value.alpha / (1.0 + value.distance2);
            for (auto& candidate : candidates)
            {
                if (candidate.group != value.group) continue;
                candidate.height += (value.height-candidate.height)*weight/(candidate.weight+weight);
                candidate.weight += weight;
                candidate.distance2 = std::min(candidate.distance2, value.distance2);
                candidate.alpha = std::max(candidate.alpha, value.alpha);
                return;
            }
            candidates.push_back(value);
            candidates.back().weight = weight;
        }

        struct IndexedSegment
        {
            Segment segment;
            osg::Vec3d ab;
            double inverseLength2, outer2;
        };
        std::vector<IndexedSegment> _segments;
        bool _nearestSegments = false;
        std::shared_ptr<const Junctions> _junctions;
        std::vector<Pad> _pads;
        RTree<unsigned, double, 3> _index, _padIndex;
    };
} }
