/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#include "RiverHydrology.h"
#include <osgEarth/Common>
#include <osgEarth/GeoData>
#include <osgEarth/SpatialReference>
#include <osgEarth/StringUtils>
#include <osgEarth/Progress>
#include <osgEarth/Notify>
#include <osg/Geometry>
#include <osg/LOD>
#include <osg/MatrixTransform>
#include <unordered_map>
#include <array>
#include <cstdlib>
#include <cstring>

#ifdef OSGEARTH_HAVE_GEOS
#define GEOS_USE_ONLY_R_API
#include <geos_c.h>
#endif

#define LC "[RiverHydrology] "

using namespace osgEarth;
using namespace osgEarth::RiverDetail;

namespace
{
    const double EARTH_RADIUS = 6378137.0;

    //! Longitude (degrees) of a spherical-mercator x coordinate.
    inline double mercToLon(double x) { return osg::RadiansToDegrees(x / EARTH_RADIUS); }

    //! Latitude (degrees) of a spherical-mercator y coordinate.
    inline double mercToLat(double y) { return osg::RadiansToDegrees(2.0 * std::atan(std::exp(y / EARTH_RADIUS)) - osg::PI_2); }

    //! Real meters per mercator meter at a mercator y coordinate (cosine of the latitude).
    inline double mercScale(double y) { return 1.0 / std::cosh(y / EARTH_RADIUS); }

    //! Hermite smoothstep, clamped.
    inline double hermite(double e0, double e1, double x)
    {
        double t = std::min(std::max((x - e0) / (e1 - e0), 0.0), 1.0);
        return t * t * (3.0 - 2.0 * t);
    }

    //! Squared distance from p to segment ab, with the segment parameter of the closest point.
    inline double distance2ToSegment(const osg::Vec2d& p, const osg::Vec2d& a, const osg::Vec2d& b, double& t)
    {
        osg::Vec2d ab = b - a;
        double len2 = ab.length2();
        t = len2 > 0.0 ? std::min(std::max(((p - a) * ab) / len2, 0.0), 1.0) : 0.0;
        osg::Vec2d c = a + ab * t;
        return (p - c).length2();
    }

    //! Shortest run (m) through a mapped water area that lets a channel set the water level there.
    //! A channel's level is fitted to the terrain along it and smoothed over a few level-smoothing
    //! sigmas, so a stream that reaches only a short way into a lake still carries the level of its
    //! steep approach, which must not tilt the lake.
    const double AREA_RUN = 150.0;

    //! Even-odd point-in-polygon test against ring edges binned into horizontal bands
    //! ("bandHeight" tall, the first one starting at "bandY0").
    bool insideRings(const osg::Vec2d& p, const std::vector<RiverDetail::Segment>& segs,
        const std::vector<std::vector<int>>& bands, double bandY0, double bandHeight)
    {
        if (bands.empty())
            return false;
        int band = (int)std::floor((p.y() - bandY0) / bandHeight);
        if (band < 0 || band >= (int)bands.size())
            return false;
        bool in = false;
        for (int i : bands[band])
        {
            const osg::Vec2d& a = segs[i].a;
            const osg::Vec2d& b = segs[i].b;
            if ((a.y() > p.y()) != (b.y() > p.y()))
            {
                double x = a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
                if (p.x() < x)
                    in = !in;
            }
        }
        return in;
    }

    //! Whether a waterway or water area lies below ground or above it on a structure.
    bool isOffTerrain(const Feature* f)
    {
        std::string tunnel = f->getString("tunnel");
        if (!tunnel.empty() && tunnel != "no")
            return true;
        if (f->getString("covered") == "yes")
            return true;
        std::string location = f->getString("location");
        if (location == "underground" || location == "underwater")
            return true;
        if (f->getString("bridge") == "aqueduct")
            return true;
        return false;
    }

    //! A centerline piece gathered from the source features, in local mercator coordinates.
    struct Piece
    {
        std::vector<osg::Vec2d> pts;
        std::vector<RiverLayer::WaterwayClass> params; // per point (constant per source feature)
        FeatureID fid = 0;
        bool user = false;
        std::string name;
        bool removed = false;
    };

    //! Length of a polyline (mercator units).
    double polylineLength(const std::vector<osg::Vec2d>& pts)
    {
        double len = 0.0;
        for (std::size_t i = 1; i < pts.size(); ++i)
            len += (pts[i] - pts[i - 1]).length();
        return len;
    }

    //! Closest point on a polyline to p: returns the distance and fills the segment index and parameter.
    double closestOnPolyline(const std::vector<osg::Vec2d>& pts, const osg::Vec2d& p, int& seg, double& t)
    {
        double best = DBL_MAX;
        seg = 0; t = 0.0;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i)
        {
            double tt;
            double d2 = distance2ToSegment(p, pts[i], pts[i + 1], tt);
            if (d2 < best)
            {
                best = d2, seg = (int)i, t = tt;
            }
        }
        return std::sqrt(best);
    }

    //! Merges pieces of the same source feature that came from different data tiles.
    //! Tiled sources clip a waterway at every tile edge with a small overlap, so the
    //! pieces overlap rather than share endpoints; this splices them back into one line
    //! and drops pieces wholly contained in another.
    void spliceByFeature(std::vector<Piece>& pieces, double tolerance)
    {
        std::map<FeatureID, std::vector<int>> groups;
        for (int i = 0; i < (int)pieces.size(); ++i)
            if (pieces[i].fid != 0)
                groups[pieces[i].fid].push_back(i);

        for (auto& group : groups)
        {
            std::vector<int>& members = group.second;
            bool merged = true;
            while (merged && members.size() > 1)
            {
                merged = false;
                for (std::size_t ii = 0; ii < members.size() && !merged; ++ii)
                {
                    for (std::size_t jj = 0; jj < members.size() && !merged; ++jj)
                    {
                        if (ii == jj) continue;
                        Piece& a = pieces[members[ii]];
                        Piece& b = pieces[members[jj]];
                        int segFront, segBack;
                        double tFront, tBack;
                        double dFront = closestOnPolyline(a.pts, b.pts.front(), segFront, tFront);
                        if (dFront > tolerance)
                            continue;

                        double dBack = closestOnPolyline(a.pts, b.pts.back(), segBack, tBack);
                        if (dBack <= tolerance)
                        {
                            // b lies along a: it's a duplicate.
                            b.removed = true;
                        }
                        else
                        {
                            // b continues a: keep a up to the splice point, then all of b.
                            std::vector<osg::Vec2d> pts(a.pts.begin(), a.pts.begin() + segFront + 1);
                            std::vector<RiverLayer::WaterwayClass> params(a.params.begin(), a.params.begin() + segFront + 1);
                            if ((pts.back() - b.pts.front()).length2() < 1e-12)
                            {
                                pts.pop_back();
                                params.pop_back();
                            }
                            pts.insert(pts.end(), b.pts.begin(), b.pts.end());
                            params.insert(params.end(), b.params.begin(), b.params.end());
                            a.pts.swap(pts);
                            a.params.swap(params);
                            b.removed = true;
                        }
                        members.erase(members.begin() + jj);
                        merged = true;
                    }
                }
            }
        }

        pieces.erase(
            std::remove_if(pieces.begin(), pieces.end(), [](const Piece& p) { return p.removed; }),
            pieces.end());
    }

    //! Joins pieces that continue one another at a shared node (end of one = start of
    //! another). At confluences the downstream piece continues the upstream piece with
    //! the same name, or else the widest one, so main stems become long smooth chains.
    void chainContinuations(std::vector<Piece>& pieces, double tolerance)
    {
        auto nodeKey = [tolerance](const osg::Vec2d& p) {
            return std::make_pair((long long)std::floor(p.x() / tolerance), (long long)std::floor(p.y() / tolerance));
        };

        for (int pass = 0; pass < 64; ++pass)
        {
            std::map<std::pair<long long, long long>, std::vector<int>> starts, ends;
            for (int i = 0; i < (int)pieces.size(); ++i)
            {
                if (pieces[i].removed || pieces[i].pts.size() < 2) continue;
                starts[nodeKey(pieces[i].pts.front())].push_back(i);
                ends[nodeKey(pieces[i].pts.back())].push_back(i);
            }

            bool merged = false;
            for (auto& entry : starts)
            {
                if (entry.second.size() != 1)
                    continue;
                int down = entry.second.front();
                auto e = ends.find(entry.first);
                if (e == ends.end())
                    continue;

                int best = -1;
                float bestWidth = -1.0f;
                bool bestNamed = false;
                for (int up : e->second)
                {
                    if (up == down || pieces[up].removed || pieces[down].removed) continue;
                    bool named = !pieces[up].name.empty() && pieces[up].name == pieces[down].name;
                    float width = pieces[up].params.back().width;
                    if (best < 0 || (named && !bestNamed) || (named == bestNamed && width > bestWidth))
                    {
                        best = up, bestWidth = width, bestNamed = named;
                    }
                }
                if (best < 0)
                    continue;

                Piece& a = pieces[best];
                Piece& b = pieces[down];
                a.pts.insert(a.pts.end(), b.pts.begin() + 1, b.pts.end());
                a.params.insert(a.params.end(), b.params.begin() + 1, b.params.end());
                b.removed = true;
                merged = true;
            }

            if (!merged)
                break;
        }

        pieces.erase(
            std::remove_if(pieces.begin(), pieces.end(), [](const Piece& p) { return p.removed; }),
            pieces.end());
    }

    //! Gaussian smoothing of a profile sampled at arc lengths s (sigma in the same units).
    std::vector<float> smoothProfile(const std::vector<float>& s, const std::vector<float>& v, float sigma)
    {
        std::vector<float> out(v.size());
        if (sigma <= 0.0f)
            return v;
        const float window = 3.0f * sigma;
        const float inv2s2 = 1.0f / (2.0f * sigma * sigma);
        std::size_t lo = 0;
        for (std::size_t i = 0; i < v.size(); ++i)
        {
            while (lo < i && s[i] - s[lo] > window) ++lo;
            double sum = 0.0, wsum = 0.0;
            for (std::size_t j = lo; j < v.size() && s[j] - s[i] <= window; ++j)
            {
                float ds = s[j] - s[i];
                double w = std::exp(-ds * ds * inv2s2);
                sum += w * v[j];
                wsum += w;
            }
            out[i] = wsum > 0.0 ? (float)(sum / wsum) : v[i];
        }
        return out;
    }

    //! Terrain under a channel steeper than this (m/m, about 30 degrees over 20 m) is a fall:
    //! water cannot stay in a bed there, it tumbles down the rock.
    const float FALL_GRADIENT = 0.58f;

    //! Around a mapped waterfall the terrain model smears the drop; it counts from this slope.
    const float MAPPED_FALL_GRADIENT = 0.12f;

    //! A fall found only in the terrain must drop at least this far (m).
    const float FALL_MIN_DROP = 4.0f;

    //! Gap (m, perpendicular to the rock) between falling water and the terrain under it.
    const float FALL_OFFSET = 1.1f;

    //! Half the thickness (m) of the band drawn as falling water around a river's curtain falls.
    const float CURTAIN_HALF_WIDTH = 2.5f;

    //! Longest water mesh edge (m) across a river's curtain falls.
    const double CURTAIN_EDGE = 2.0;

    //! How far (m) a river's curtain falls arc out past the rock edge before they drop.
    const float CURTAIN_THROW = 3.0f;

    //! A stretch of channel points where the water falls: [lip, foot], lip and foot being the
    //! last point of the reach above and the first point of the reach below.
    struct FallSpan
    {
        std::size_t lip = 0, foot = 0;
        bool mapped = false;
    };

    //! Fits a water surface to the terrain profile h of one reach (channel points i0..i1, between
    //! falls): smooths it, holds it to the lowest level found within a window upstream (water does
    //! not climb), then softens the kinks the minimum introduces. Writes level[i0..i1]. Fitting
    //! each reach on its own keeps a fall a sharp drop rather than a long ramp, and lets the pool
    //! below keep its own level.
    void fitReach(const std::vector<float>& s, const std::vector<float>& h, std::size_t i0, std::size_t i1,
        float sigma, std::vector<float>& level)
    {
        std::vector<float> rs(s.begin() + i0, s.begin() + i1 + 1);
        std::vector<float> rh(h.begin() + i0, h.begin() + i1 + 1);
        std::vector<float> smooth = smoothProfile(rs, rh, sigma);
        std::vector<float> held(rs.size());
        const float window = 3.0f * sigma;
        for (std::size_t i = 0; i < rs.size(); ++i)
        {
            float lowest = smooth[i];
            for (std::size_t j = i; j-- > 0 && rs[i] - rs[j] <= window; )
                lowest = std::min(lowest, smooth[j]);
            held[i] = lowest;
        }
        std::vector<float> out = smoothProfile(rs, held, sigma * 0.35f);
        for (std::size_t i = 0; i < out.size(); ++i)
            level[i0 + i] = out[i] - 0.15f;
    }

    //! Falls along a channel: stretches where the terrain profile h under it drops steeply, and
    //! the drop around each mapped waterfall node (local mercator) within reach of it. k is real
    //! meters per mercator meter. Returns the spans in order, each at least two points long and
    //! none overlapping.
    std::vector<FallSpan> findFalls(const RiverDetail::Channel& ch, const std::vector<float>& h,
        const std::vector<osg::Vec2d>& nodes, double stepMeters, double k)
    {
        std::vector<FallSpan> spans;
        const std::size_t n = ch.p.size();
        if (n < 3)
            return spans;

        // Downhill gradient of the terrain over about 20 m.
        const std::size_t m = std::max<std::size_t>(1u, (std::size_t)std::lround(10.0 / stepMeters));
        std::vector<float> grad(n, 0.0f);
        for (std::size_t i = 0; i < n; ++i)
        {
            std::size_t i0 = i >= m ? i - m : 0, i1 = std::min(i + m, n - 1);
            float ds = ch.s[i1] - ch.s[i0];
            grad[i] = ds > 0.0f ? (h[i0] - h[i1]) / ds : 0.0f;
        }

        // 1 = steep in the terrain, 2 = around a mapped waterfall
        std::vector<char> mark(n, 0);
        for (std::size_t i = 0; i < n; ++i)
            if (grad[i] >= FALL_GRADIENT)
                mark[i] = 1;

        for (const osg::Vec2d& node : nodes)
        {
            std::size_t best = n;
            double bestD = DBL_MAX;
            for (std::size_t i = 0; i < n; ++i)
            {
                double d = (ch.p[i] - node).length() * k;
                if (d < bestD)
                    bestD = d, best = i;
            }
            if (best == n || bestD > std::max((double)ch.halfWidth[best], 5.0) + 25.0)
                continue;
            // The terrain model spreads the drop over a stretch around the node; a drop it does
            // not resolve at all still gets a short cascade at the node.
            std::size_t a = best, b = best;
            while (a > 0 && ch.s[best] - ch.s[a - 1] <= 150.0f && grad[a - 1] >= MAPPED_FALL_GRADIENT)
                --a;
            while (b + 1 < n && ch.s[b + 1] - ch.s[best] <= 300.0f && grad[b + 1] >= MAPPED_FALL_GRADIENT)
                ++b;
            for (std::size_t i = a; i <= b; ++i)
                mark[i] = 2;
        }

        // Runs of marked points (joined across gaps under 15 m) become falls. The lip and the
        // foot are the unmarked points on either side, which belong to the pools.
        std::size_t i = 0;
        while (i < n)
        {
            if (!mark[i])
            {
                ++i;
                continue;
            }
            std::size_t first = i, last = i;
            bool mapped = mark[i] == 2;
            for (std::size_t j = i + 1; j < n; ++j)
            {
                if (mark[j])
                {
                    last = j;
                    mapped |= mark[j] == 2;
                }
                else if (ch.s[j] - ch.s[last] > 15.0f)
                    break;
            }
            i = last + 1;

            FallSpan span;
            span.lip = first > 0 ? first - 1 : 0;
            span.foot = std::min(last + 1, n - 1);
            span.mapped = mapped;
            if (span.foot < span.lip + 2)
                continue;
            if (!mapped && h[span.lip] - h[span.foot] < FALL_MIN_DROP)
                continue;
            if (!spans.empty() && span.lip <= spans.back().foot)
            {
                spans.back().foot = std::max(spans.back().foot, span.foot);
                spans.back().mapped |= span.mapped;
            }
            else
                spans.push_back(span);
        }
        return spans;
    }

#ifdef OSGEARTH_HAVE_GEOS

    //! Owns one reentrant GEOS context for the duration of a build, with messages muted.
    struct GeosContext
    {
        GEOSContextHandle_t h;
        GeosContext() : h(GEOS_init_r())
        {
            GEOSContext_setNoticeHandler_r(h, nullptr);
            GEOSContext_setErrorHandler_r(h, nullptr);
        }
        ~GeosContext() { GEOS_finish_r(h); }
    };

    //! Move-only owner of a GEOS geometry.
    class GGeom
    {
    public:
        GGeom() = default;
        GGeom(GEOSContextHandle_t h, GEOSGeometry* g) : _h(h), _g(g) { }
        GGeom(GGeom&& rhs) : _h(rhs._h), _g(rhs._g) { rhs._g = nullptr; }
        GGeom& operator=(GGeom&& rhs) { reset(); _h = rhs._h; _g = rhs._g; rhs._g = nullptr; return *this; }
        GGeom(const GGeom&) = delete;
        GGeom& operator=(const GGeom&) = delete;
        ~GGeom() { reset(); }
        void reset() { if (_g) GEOSGeom_destroy_r(_h, _g); _g = nullptr; }
        GEOSGeometry* get() const { return _g; }
        GEOSGeometry* release() { GEOSGeometry* g = _g; _g = nullptr; return g; }
        bool valid() const { return _g != nullptr && !GEOSisEmpty_r(_h, _g); }
    private:
        GEOSContextHandle_t _h = nullptr;
        GEOSGeometry* _g = nullptr;
    };

    //! GEOS coordinate sequence from points, optionally closing the loop.
    GEOSCoordSequence* makeSequence(GEOSContextHandle_t h, const std::vector<osg::Vec2d>& pts, bool close)
    {
        std::size_t n = pts.size() + (close ? 1u : 0u);
        GEOSCoordSequence* seq = GEOSCoordSeq_create_r(h, (unsigned)n, 2);
        for (std::size_t i = 0; i < pts.size(); ++i)
            GEOSCoordSeq_setXY_r(h, seq, (unsigned)i, pts[i].x(), pts[i].y());
        if (close)
            GEOSCoordSeq_setXY_r(h, seq, (unsigned)pts.size(), pts.front().x(), pts.front().y());
        return seq;
    }

    //! Points of an osgEarth geometry part shifted to local coordinates, without repeated points.
    std::vector<osg::Vec2d> localPoints(const Geometry& part, const osg::Vec2d& origin)
    {
        std::vector<osg::Vec2d> pts;
        pts.reserve(part.size());
        for (auto& v : part)
        {
            osg::Vec2d p(v.x() - origin.x(), v.y() - origin.y());
            if (pts.empty() || (pts.back() - p).length2() > 1e-12)
                pts.push_back(p);
        }
        return pts;
    }

    //! GEOS linear ring from an osgEarth ring, or null if degenerate.
    GEOSGeometry* makeRing(GEOSContextHandle_t h, const Geometry& ring, const osg::Vec2d& origin)
    {
        std::vector<osg::Vec2d> pts = localPoints(ring, origin);
        if (pts.size() > 1 && (pts.front() - pts.back()).length2() < 1e-12)
            pts.pop_back();
        if (pts.size() < 3)
            return nullptr;
        return GEOSGeom_createLinearRing_r(h, makeSequence(h, pts, true));
    }

    //! Appends GEOS polygons for every polygon part of an osgEarth geometry.
    void collectPolygons(GEOSContextHandle_t h, const Geometry* geom, const osg::Vec2d& origin, std::vector<GEOSGeometry*>& out)
    {
        if (!geom) return;
        if (geom->getType() == Geometry::TYPE_MULTI)
        {
            for (auto& part : static_cast<const MultiGeometry*>(geom)->getComponents())
                collectPolygons(h, part.get(), origin, out);
            return;
        }
        if (geom->getType() != Geometry::TYPE_POLYGON)
            return;

        const Polygon* poly = static_cast<const Polygon*>(geom);
        GEOSGeometry* shell = makeRing(h, *poly, origin);
        if (!shell) return;
        std::vector<GEOSGeometry*> holes;
        for (auto& hole : poly->getHoles())
        {
            GEOSGeometry* ring = hole.valid() ? makeRing(h, *hole, origin) : nullptr;
            if (ring) holes.push_back(ring);
        }
        GEOSGeometry* g = GEOSGeom_createPolygon_r(h, shell, holes.empty() ? nullptr : holes.data(), (unsigned)holes.size());
        if (g) out.push_back(g);
    }

    //! Appends GEOS line strings for every linear part of an osgEarth geometry.
    void collectLines(GEOSContextHandle_t h, const Geometry* geom, const osg::Vec2d& origin, std::vector<GEOSGeometry*>& out)
    {
        if (!geom) return;
        if (geom->getType() == Geometry::TYPE_MULTI)
        {
            for (auto& part : static_cast<const MultiGeometry*>(geom)->getComponents())
                collectLines(h, part.get(), origin, out);
            return;
        }
        if (geom->getType() != Geometry::TYPE_LINESTRING && geom->getType() != Geometry::TYPE_RING)
            return;
        std::vector<osg::Vec2d> pts = localPoints(*geom, origin);
        if (geom->getType() == Geometry::TYPE_RING && pts.size() > 2)
            pts.push_back(pts.front());
        if (pts.size() < 2)
            return;
        GEOSGeometry* g = GEOSGeom_createLineString_r(h, makeSequence(h, pts, false));
        if (g) out.push_back(g);
    }

    //! Geometry collection taking ownership of the parts (null if there are none).
    GEOSGeometry* makeCollection(GEOSContextHandle_t h, std::vector<GEOSGeometry*>& parts)
    {
        if (parts.empty()) return nullptr;
        GEOSGeometry* c = GEOSGeom_createCollection_r(h, GEOS_GEOMETRYCOLLECTION, parts.data(), (unsigned)parts.size());
        parts.clear();
        return c;
    }

    //! Rectangle polygon with extra vertices at every multiple of "step" (absolute
    //! coordinates), so neighboring tiles clip their water at identical points.
    GEOSGeometry* makeRect(GEOSContextHandle_t h, double x0, double y0, double x1, double y1, double step,
        const osg::Vec2d& origin)
    {
        std::vector<osg::Vec2d> pts;
        auto edge = [&](const osg::Vec2d& a, const osg::Vec2d& b, bool alongX)
        {
            pts.push_back(a);
            if (step <= 0.0) return;
            double a0 = alongX ? a.x() + origin.x() : a.y() + origin.y();
            double a1 = alongX ? b.x() + origin.x() : b.y() + origin.y();
            double lo = std::min(a0, a1), hi = std::max(a0, a1);
            std::vector<double> marks;
            for (double m = std::ceil(lo / step) * step; m < hi; m += step)
                if (m > lo) marks.push_back(m);
            if (a0 > a1) std::reverse(marks.begin(), marks.end());
            for (double m : marks)
                pts.push_back(alongX ? osg::Vec2d(m - origin.x(), a.y()) : osg::Vec2d(a.x(), m - origin.y()));
        };
        edge(osg::Vec2d(x0, y0), osg::Vec2d(x1, y0), true);
        edge(osg::Vec2d(x1, y0), osg::Vec2d(x1, y1), false);
        edge(osg::Vec2d(x1, y1), osg::Vec2d(x0, y1), true);
        edge(osg::Vec2d(x0, y1), osg::Vec2d(x0, y0), false);
        GEOSGeometry* ring = GEOSGeom_createLinearRing_r(h, makeSequence(h, pts, true));
        return GEOSGeom_createPolygon_r(h, ring, nullptr, 0);
    }

    //! Points of a GEOS line string or ring.
    void readSequence(GEOSContextHandle_t h, const GEOSGeometry* g, std::vector<osg::Vec2d>& out)
    {
        out.clear();
        const GEOSCoordSequence* seq = GEOSGeom_getCoordSeq_r(h, g);
        if (!seq) return;
        unsigned n = 0;
        GEOSCoordSeq_getSize_r(h, seq, &n);
        out.reserve(n);
        for (unsigned i = 0; i < n; ++i)
        {
            double x, y;
            GEOSCoordSeq_getXY_r(h, seq, i, &x, &y);
            out.emplace_back(x, y);
        }
    }

    //! Calls func(points) for every ring (shells and holes) of a polygonal GEOS geometry.
    template<typename F>
    void forEachRing(GEOSContextHandle_t h, const GEOSGeometry* g, F&& func)
    {
        if (!g) return;
        int type = GEOSGeomTypeId_r(h, g);
        if (type == GEOS_POLYGON)
        {
            std::vector<osg::Vec2d> pts;
            readSequence(h, GEOSGetExteriorRing_r(h, g), pts);
            func(pts, false);
            int holes = GEOSGetNumInteriorRings_r(h, g);
            for (int i = 0; i < holes; ++i)
            {
                readSequence(h, GEOSGetInteriorRingN_r(h, g, i), pts);
                func(pts, true);
            }
        }
        else if (type == GEOS_MULTIPOLYGON || type == GEOS_GEOMETRYCOLLECTION)
        {
            int n = GEOSGetNumGeometries_r(h, g);
            for (int i = 0; i < n; ++i)
                forEachRing(h, GEOSGetGeometryN_r(h, g, i), func);
        }
    }

    //! Converts the polygonal parts of a GEOS geometry into an osgEarth MultiGeometry.
    osg::ref_ptr<Geometry> exportPolygons(GEOSContextHandle_t h, const GEOSGeometry* g)
    {
        osg::ref_ptr<MultiGeometry> out = new MultiGeometry();
        std::function<void(const GEOSGeometry*)> visit = [&](const GEOSGeometry* part)
        {
            int type = GEOSGeomTypeId_r(h, part);
            if (type == GEOS_POLYGON)
            {
                std::vector<osg::Vec2d> pts;
                readSequence(h, GEOSGetExteriorRing_r(h, part), pts);
                if (pts.size() < 4) return;
                osg::ref_ptr<Polygon> poly = new Polygon();
                for (std::size_t i = 0; i + 1 < pts.size(); ++i)
                    poly->push_back(pts[i].x(), pts[i].y(), 0.0);
                int holes = GEOSGetNumInteriorRings_r(h, part);
                for (int i = 0; i < holes; ++i)
                {
                    readSequence(h, GEOSGetInteriorRingN_r(h, part, i), pts);
                    if (pts.size() < 4) continue;
                    osg::ref_ptr<Ring> ring = new Ring();
                    for (std::size_t j = 0; j + 1 < pts.size(); ++j)
                        ring->push_back(pts[j].x(), pts[j].y(), 0.0);
                    poly->getHoles().push_back(ring);
                }
                out->add(poly.get());
            }
            else if (type == GEOS_MULTIPOLYGON || type == GEOS_GEOMETRYCOLLECTION)
            {
                int n = GEOSGetNumGeometries_r(h, part);
                for (int i = 0; i < n; ++i)
                    visit(GEOSGetGeometryN_r(h, part, i));
            }
        };
        visit(g);
        return out;
    }

    //! Builds a GEOS geometry from an osgEarth polygonal geometry already in local coordinates.
    GEOSGeometry* importLocal(GEOSContextHandle_t h, const Geometry* geom)
    {
        std::vector<GEOSGeometry*> parts;
        collectPolygons(h, geom, osg::Vec2d(0, 0), parts);
        if (parts.empty()) return nullptr;
        if (parts.size() == 1) return parts.front();
        return GEOSGeom_createCollection_r(h, GEOS_MULTIPOLYGON, parts.data(), (unsigned)parts.size());
    }

    //! Constrained Delaunay triangulation of local polygons, appended to verts/tris with
    //! shared vertices and counter-clockwise winding. Leaves both untouched on failure.
    void triangulate(GEOSContextHandle_t h, const Geometry* polygons,
        std::vector<osg::Vec2d>& verts, std::vector<std::array<unsigned, 3>>& tris)
    {
        GGeom input(h, importLocal(h, polygons));
        if (!input.valid())
            return;
        GGeom valid(h, GEOSMakeValid_r(h, input.get()));
        if (!valid.valid())
            return;
        GGeom triangles(h, GEOSConstrainedDelaunayTriangulation_r(h, valid.get()));
        if (!triangles.valid())
            return;

        struct PairHash {
            std::size_t operator()(const std::pair<double, double>& p) const {
                return std::hash<double>()(p.first) ^ (std::hash<double>()(p.second) * 31u);
            }
        };
        std::unordered_map<std::pair<double, double>, unsigned, PairHash> lookup;
        auto vertex = [&](const osg::Vec2d& p) -> unsigned
        {
            auto key = std::make_pair(p.x(), p.y());
            auto i = lookup.find(key);
            if (i != lookup.end()) return i->second;
            unsigned index = (unsigned)verts.size();
            verts.push_back(p);
            lookup[key] = index;
            return index;
        };

        int n = GEOSGetNumGeometries_r(h, triangles.get());
        std::vector<osg::Vec2d> pts;
        for (int i = 0; i < n; ++i)
        {
            const GEOSGeometry* tri = GEOSGetGeometryN_r(h, triangles.get(), i);
            if (GEOSGeomTypeId_r(h, tri) != GEOS_POLYGON) continue;
            readSequence(h, GEOSGetExteriorRing_r(h, tri), pts);
            if (pts.size() < 4) continue;
            osg::Vec2d a = pts[0], b = pts[1], c = pts[2];
            double area = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
            if (std::abs(area) < 1e-10) continue;
            if (area < 0.0) std::swap(b, c);
            tris.push_back({ vertex(a), vertex(b), vertex(c) });
        }
    }

#endif // OSGEARTH_HAVE_GEOS

    //! Mesh refinement: splits every edge longer than maxEdge at its midpoint (shared between the
    //! two triangles using it, so the mesh stays watertight) until no edge is too long. Edges with
    //! an end where "fine" holds are split down to fineEdge instead. The decision depends only on
    //! the edge itself, so both triangles sharing an edge always agree.
    void refineMesh(std::vector<osg::Vec2d>& verts, std::vector<std::array<unsigned, 3>>& tris, double maxEdge,
        const std::function<bool(const osg::Vec2d&)>& fine = nullptr, double fineEdge = 0.0)
    {
        const double max2 = maxEdge * maxEdge;
        const double fine2 = fineEdge * fineEdge;
        std::vector<signed char> fineAt; // per vertex: -1 unknown, 0 no, 1 yes
        auto isFine = [&](unsigned v) -> bool
        {
            if (!fine) return false;
            if (fineAt.size() <= v) fineAt.resize(verts.size(), (signed char)-1);
            if (fineAt[v] < 0) fineAt[v] = fine(verts[v]) ? 1 : 0;
            return fineAt[v] == 1;
        };
        for (int pass = 0; pass < 12; ++pass)
        {
            std::unordered_map<std::uint64_t, unsigned> mids;
            auto mid = [&](unsigned a, unsigned b) -> unsigned
            {
                std::uint64_t key = a < b ? ((std::uint64_t)a << 32) | b : ((std::uint64_t)b << 32) | a;
                auto i = mids.find(key);
                if (i != mids.end()) return i->second;
                unsigned index = (unsigned)verts.size();
                verts.push_back((verts[a] + verts[b]) * 0.5);
                mids[key] = index;
                return index;
            };

            bool any = false;
            std::vector<std::array<unsigned, 3>> out;
            out.reserve(tris.size() * 2);
            for (const auto& t : tris)
            {
                bool split[3];
                int count = 0;
                for (int e = 0; e < 3; ++e)
                {
                    unsigned a = t[e], b = t[(e + 1) % 3];
                    double len2 = (verts[a] - verts[b]).length2();
                    split[e] = len2 > max2 || (len2 > fine2 && fine && (isFine(a) || isFine(b)));
                    if (split[e]) ++count;
                }

                if (count == 0)
                {
                    out.push_back(t);
                    continue;
                }
                any = true;

                if (count == 3)
                {
                    unsigned m0 = mid(t[0], t[1]), m1 = mid(t[1], t[2]), m2 = mid(t[2], t[0]);
                    out.push_back({ t[0], m0, m2 });
                    out.push_back({ m0, t[1], m1 });
                    out.push_back({ m2, m1, t[2] });
                    out.push_back({ m0, m1, m2 });
                }
                else if (count == 1)
                {
                    int e = split[0] ? 0 : split[1] ? 1 : 2;
                    unsigned a = t[e], b = t[(e + 1) % 3], c = t[(e + 2) % 3];
                    unsigned m = mid(a, b);
                    out.push_back({ a, m, c });
                    out.push_back({ m, b, c });
                }
                else // two edges split; u is the one that is not
                {
                    int u = !split[0] ? 0 : !split[1] ? 1 : 2;
                    unsigned P = t[u], Q = t[(u + 1) % 3], R = t[(u + 2) % 3];
                    unsigned m1 = mid(Q, R), m2 = mid(R, P);
                    out.push_back({ m1, R, m2 });
                    // split quad P,Q,m1,m2 along its shorter diagonal
                    if ((verts[P] - verts[m1]).length2() < (verts[Q] - verts[m2]).length2())
                    {
                        out.push_back({ P, Q, m1 });
                        out.push_back({ P, m1, m2 });
                    }
                    else
                    {
                        out.push_back({ P, Q, m2 });
                        out.push_back({ Q, m1, m2 });
                    }
                }
            }
            tris.swap(out);
            if (!any) break;
        }
    }
}

//...................................................................

double
RiverDetail::parseLength(const std::string& value)
{
    std::string s = osgEarth::Util::trim(value);
    if (s.empty())
        return 0.0;
    const char* begin = s.c_str();
    char* end = nullptr;
    double v = std::strtod(begin, &end);
    if (end == begin)
        return 0.0;
    std::string unit = osgEarth::Util::toLower(osgEarth::Util::trim(std::string(end)));
    if (unit == "'" || unit == "ft" || unit == "feet")
        v *= 0.3048;
    else if (unit == "km")
        v *= 1000.0;
    return v > 0.0 && v < 100000.0 ? v : 0.0;
}

bool
RiverDetail::isWaterArea(const Feature* f)
{
    if (!f || !f->getGeometry() || !f->getGeometry()->isPolygon())
        return false;
    if (isOffTerrain(f))
        return false;
    std::string natural = f->getString("natural");
    if (natural == "water")
        return true;
    std::string waterway = f->getString("waterway");
    if (waterway == "riverbank" || waterway == "dock")
        return true;
    if (f->getString("landuse") == "reservoir")
        return true;
    if (natural.empty() && !f->getString("water").empty())
        return true;
    return false;
}

bool
RiverDetail::getChannelParams(const Feature* f, const BuildSettings& settings, RiverLayer::WaterwayClass& out)
{
    if (!f || !f->getGeometry())
        return false;
    if (f->getGeometry()->getComponentType() != Geometry::TYPE_LINESTRING)
        return false;
    if (isOffTerrain(f))
        return false;
    auto i = settings.classes.find(f->getString("waterway"));
    if (i == settings.classes.end())
        return false;

    out = i->second;
    double width = parseLength(f->getString("width"));
    if (width > 0.25)
        out.width = (float)width;
    double depth = parseLength(f->getString("depth"));
    if (depth > 0.05)
        out.depth = (float)depth;
    double speed = std::atof(f->getString("speed").c_str());
    if (speed > 0.0)
        out.speed = (float)speed;
    return true;
}

//...................................................................

double
TileField::shoreDistance(const osg::Vec2d& p, double maxDistance, osg::Vec2d* nearest) const
{
    double best2 = DBL_MAX;
    osg::Vec2d bestPoint = p;
    // Search a small box first; most queries near water resolve there.
    const double radii[2] = { std::min(32.0, maxDistance), maxDistance };
    for (double radiusMeters : radii)
    {
        double r = radiusMeters / k;
        shoreGrid.query(p - osg::Vec2d(r, r), p + osg::Vec2d(r, r), [&](int i)
            {
                const Segment& s = shoreSegs[i];
                double t;
                double d2 = distance2ToSegment(p, s.a, s.b, t);
                if (d2 < best2)
                {
                    best2 = d2;
                    bestPoint = s.a + (s.b - s.a) * t;
                }
            });
        if (best2 <= r * r)
            break;
    }
    if (nearest) *nearest = bestPoint;
    double d = best2 < DBL_MAX ? std::sqrt(best2) * k : maxDistance;
    return std::min(d, maxDistance);
}

bool
TileField::inside(const osg::Vec2d& p) const
{
    return insideRings(p, shoreSegs, bands, bandY0, bandHeight);
}

bool
TileField::inArea(const osg::Vec2d& p, double tolerance) const
{
    if (areaSegs.empty())
        return false;
    if (tolerance > 0.0)
    {
        // Shore vertices of the water mesh sit exactly on an area's edge, where the parity
        // test could go either way.
        double r = tolerance / k;
        bool onEdge = false;
        areaGrid.query(p - osg::Vec2d(r, r), p + osg::Vec2d(r, r), [&](int i)
            {
                double t;
                if (!onEdge && distance2ToSegment(p, areaSegs[i].a, areaSegs[i].b, t) <= r * r)
                    onEdge = true;
            });
        if (onEdge)
            return true;
    }
    return insideRings(p, areaSegs, areaBands, bandY0, bandHeight);
}

float
TileField::terrainHeight(const osg::Vec2d& p) const
{
    if (dem.empty())
        return 0.0f;
    double fx = (p.x() - demOrigin.x()) / demSpacing;
    double fy = (p.y() - demOrigin.y()) / demSpacing;
    fx = std::min(std::max(fx, 0.0), (double)demCols - 1.001);
    fy = std::min(std::max(fy, 0.0), (double)demRows - 1.001);
    unsigned x0 = (unsigned)fx, y0 = (unsigned)fy;
    double tx = fx - x0, ty = fy - y0;
    const float* row0 = &dem[(std::size_t)y0 * demCols];
    const float* row1 = row0 + demCols;
    double a = row0[x0] * (1.0 - tx) + row0[x0 + 1] * tx;
    double b = row1[x0] * (1.0 - tx) + row1[x0 + 1] * tx;
    return (float)(a * (1.0 - ty) + b * ty);
}

float
TileField::terrainSteepness(const osg::Vec2d& p) const
{
    if (dem.empty())
        return 0.0f;
    const double d = demSpacing;
    double dx = terrainHeight(p + osg::Vec2d(d, 0.0)) - terrainHeight(p - osg::Vec2d(d, 0.0));
    double dy = terrainHeight(p + osg::Vec2d(0.0, d)) - terrainHeight(p - osg::Vec2d(0.0, d));
    return (float)(std::sqrt(dx * dx + dy * dy) / (2.0 * d * k));
}

float
TileField::lakeLevel(const osg::Vec2d& p) const
{
    // Still water is flat: take the lowest terrain in a disk around the point
    // (robust mean of the three lowest samples), which also floods small ponds the
    // terrain model cannot resolve.
    static const double ring[] = { 0.0, 18.0, 36.0, 60.0 };
    float samples[25];
    int n = 0;
    samples[n++] = terrainHeight(p);
    for (int r = 1; r < 4; ++r)
    {
        double rad = ring[r] / k;
        for (int a = 0; a < 8; ++a)
        {
            double ang = (a + 0.5 * (r & 1)) * osg::PI / 4.0;
            samples[n++] = terrainHeight(p + osg::Vec2d(std::cos(ang), std::sin(ang)) * rad);
        }
    }
    std::partial_sort(samples, samples + 3, samples + n);
    return (samples[0] + samples[1] + samples[2]) / 3.0f - 0.1f;
}

bool
TileField::sample(const osg::Vec2d& p, int insideHint, float margin, RiverLayer::WaterSample& out) const
{
    out = RiverLayer::WaterSample();

    const double maxShore = 250.0;
    osg::Vec2d shorePoint;
    double dShore = shoreDistance(p, maxShore, &shorePoint);
    bool in = insideHint > 0 ? true : insideHint < 0 ? false : inside(p);
    if (!in && dShore > margin)
        return false;

    out.water = in;
    out.shore = in ? (float)dShore : -(float)dShore;

    // In a mapped water area (or, for points outside the water, at the shore of one), only the
    // channels that run through the area set its level; a stream that just reaches into a lake
    // would tilt it toward the level of its steep approach. Ribbon water hears every channel.
    const bool area = inArea(in ? p : shorePoint, 0.5);

    // Nearest point on each nearby channel. Wide rivers can put the centerline far from the
    // banks, so widen the search until something turns up.
    struct Hit { int channel; double d; int seg; double t; };
    std::vector<Hit> hits;
    static const double radii[3] = { 60.0, 220.0, 650.0 };
    for (double radiusMeters : radii)
    {
        double r = radiusMeters / k;
        hits.clear();
        channelGrid.query(p - osg::Vec2d(r, r), p + osg::Vec2d(r, r), [&](int i)
            {
                const Segment& s = channelSegs[i];
                if (area)
                {
                    const std::vector<char>& owns = channels[s.owner].ownsArea;
                    if (!owns[s.index] && !owns[s.index + 1])
                        return;
                }
                double t;
                double d = std::sqrt(distance2ToSegment(p, s.a, s.b, t)) * k;
                for (auto& h : hits)
                {
                    if (h.channel == s.owner)
                    {
                        if (d < h.d) h.d = d, h.seg = s.index, h.t = t;
                        return;
                    }
                }
                hits.push_back(Hit{ s.owner, d, s.index, t });
            });
        double dmin = DBL_MAX;
        for (auto& h : hits) dmin = std::min(dmin, h.d);
        if (dmin <= radiusMeters)
            break;
        hits.clear();
    }

    if (!hits.empty())
    {
        std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.d < b.d; });
        const Channel& c0 = channels[hits.front().channel];
        auto lerp = [](float a, float b, double t) { return (float)(a + (b - a) * t); };
        const Hit& h0 = hits.front();
        float hw0 = lerp(c0.halfWidth[h0.seg], c0.halfWidth[h0.seg + 1], h0.t);
        double blend = std::max(8.0, 0.35 * hw0);
        double dmin = h0.d;

        double wsum = 0.0, level = 0.0, slope = 0.0, depth = 0.0, speed = 0.0, halfWidth = 0.0, rapids = 0.0;
        osg::Vec2d dir(0, 0);
        for (auto& h : hits)
        {
            if (h.d > dmin + blend) break;
            const Channel& c = channels[h.channel];
            double w = 1.0 / ((h.d + 1.5) * (h.d + 1.5));
            int i = h.seg;
            level += w * lerp(c.level[i], c.level[i + 1], h.t);
            slope += w * lerp(c.slope[i], c.slope[i + 1], h.t);
            depth += w * lerp(c.depth[i], c.depth[i + 1], h.t);
            rapids += w * lerp(c.rapids[i], c.rapids[i + 1], h.t);
            float sp = lerp(c.speed[i], c.speed[i + 1], h.t);
            speed += w * sp;
            halfWidth += w * lerp(c.halfWidth[i], c.halfWidth[i + 1], h.t);
            osg::Vec2f d = c.dir[i] * (float)(1.0 - h.t) + c.dir[i + 1] * (float)h.t;
            dir += osg::Vec2d(d.x(), d.y()) * (w * sp);
            wsum += w;
        }
        level /= wsum, slope /= wsum, depth /= wsum, speed /= wsum, halfWidth /= wsum, rapids /= wsum;

        // Half of the local water width: the channel's own width for narrow streams drawn
        // as lines, or centerline-to-bank distance inside mapped river areas.
        double localHalf = in ? std::max(dmin + dShore, 0.5) : halfWidth;
        double across = in ? std::min(std::max(dmin / localHalf, 0.0), 1.0) : 1.0;
        double maxDepth = std::max(depth, std::min(0.07 * localHalf, 12.0)) * settings.depthScale;

        out.valid = true;
        out.level = (float)level;
        out.slope = (float)slope;
        out.turbulence = (float)(hermite(0.012, 0.06, slope) * rapids);
        out.across = (float)across;
        out.depth = in ? (float)(maxDepth * (1.0 - across * across)) : 0.0f;
        double profile = 0.25 + 0.75 * (1.0 - across * across);
        out.speed = in ? (float)(speed * profile) : 0.0f;
        double len = dir.length();
        out.flow = len > 1e-9 ? osg::Vec2f(dir.x() / len, dir.y() / len) : osg::Vec2f(0, 0);

        // Falls (see Fall); around them the pools churn.
        const int fi = c0.fall[h0.t < 0.5 ? h0.seg : h0.seg + 1];
        bool falling = false;
        if (fi >= 0)
        {
            const Fall& fall = falls[fi];
            const float sHit = lerp(c0.s[h0.seg], c0.s[h0.seg + 1], h0.t);
            if (fall.curtain)
            {
                // Above the middle of the drop the bed is built up into a shelf; below it the
                // plunge pool is carved. The water leaves the shelf at the lip's level, arcs out a
                // few meters past the rock edge and drops in a sheer curtain to the pool's level.
                const float mid = 0.5f * (fall.top + fall.bottom);
                const float ground = terrainHeight(p);
                const float steep = std::max(terrainSteepness(p), 0.05f);
                const float edge = mid - CURTAIN_THROW * steep; // where the water drops
                const bool above = ground > edge;
                out.level = above ? fall.top : fall.bottom;
                out.shelf = ground > mid;
                if (std::abs(ground - edge) / steep < CURTAIN_HALF_WIDTH)
                {
                    falling = true;
                    out.fall = 1.0f;
                    out.fallen = above ? 0.0f : std::max(fall.top - fall.bottom, 0.0f);
                }
            }
            else if (sHit > c0.s[fall.lip] && sHit < c0.s[fall.foot])
            {
                // Pouring over the rock: draped over the terrain, held off it by a gap measured
                // across the slope so that steep faces keep it clear.
                const float steep = terrainSteepness(p);
                const float lift = FALL_OFFSET * std::min(std::sqrt(1.0f + steep * steep), 8.0f);
                out.level = std::min(terrainHeight(p) + lift, fall.top);
                out.draped = true;
                out.slope = std::max(out.slope, steep);
                falling = true;
                out.fall = 1.0f;
                out.fallen = std::max(fall.top - out.level, 0.0f);
            }
        }
        if (falling)
        {
            // whitewater racing down at the speed the fall has given it so far
            out.turbulence = 1.0f;
            out.depth = in ? 0.3f : 0.0f;
            out.speed = in ? std::min(std::max(std::sqrt(2.0f * 9.81f * std::max(out.fallen, 0.3f)), 2.5f), 25.0f) : 0.0f;
        }
        else
        {
            out.turbulence = std::max(out.turbulence, lerp(c0.churn[h0.seg], c0.churn[h0.seg + 1], h0.t));
        }
        return true;
    }

    // Still water: evaluate outside points at the nearest bank so a lake's surroundings
    // see the lake's level rather than their own terrain.
    osg::Vec2d q = in ? p : shorePoint;
    out.valid = true;
    out.level = lakeLevel(q);
    out.slope = 0.0f;
    out.speed = 0.0f;
    out.flow.set(0, 0);
    out.across = in ? (float)(1.0 - hermite(0.0, 40.0, dShore)) : 1.0f;
    out.depth = in ? (float)((0.25 + 3.5 * hermite(0.0, 70.0, dShore)) * settings.depthScale) : 0.0f;
    return true;
}

//...................................................................

osg::ref_ptr<TileField>
RiverDetail::buildField(
    const TileKey& key,
    const FeatureList& features,
    const BuildSettings& settings,
    const HeightSampler& heights,
    double bufferMeters,
    ProgressCallback* progress)
{
    osg::ref_ptr<TileField> field = new TileField();
    field->key = key;
    field->settings = settings;

    const GeoExtent& ex = key.getExtent();
    field->origin.set(ex.xMin(), ex.yMin());
    field->size.set(ex.width(), ex.height());
    field->k = mercScale(ex.yMin() + 0.5 * ex.height());
    field->buffer = bufferMeters / field->k;

    const double k = field->k;
    const double buffer = field->buffer;
    const osg::Vec2d& origin = field->origin;

    // Gather centerline pieces in local coordinates, the water areas, and the mapped waterfalls.
    std::vector<Piece> pieces;
    std::vector<const Feature*> areas;
    std::vector<osg::Vec2d> fallNodes;
    for (auto& f : features)
    {
        if (!f.valid() || !f->getGeometry())
            continue;

        RiverLayer::WaterwayClass params;
        if (getChannelParams(f.get(), settings, params))
        {
            ConstGeometryIterator iter(f->getGeometry(), false);
            while (iter.hasMore())
            {
                const Geometry* part = iter.next();
                Piece piece;
                for (auto& v : *part)
                {
                    osg::Vec2d p(v.x() - origin.x(), v.y() - origin.y());
                    if (piece.pts.empty() || (piece.pts.back() - p).length2() > 1e-10)
                        piece.pts.push_back(p);
                }
                if (piece.pts.size() < 2 || polylineLength(piece.pts) * k < 1.0)
                    continue;
                piece.params.assign(piece.pts.size(), params);
                piece.fid = f->getFID();
                piece.user = f->hasAttr("river:user");
                piece.name = f->getString("name");
                pieces.push_back(std::move(piece));
            }
        }
        else if (isWaterArea(f.get()))
        {
            areas.push_back(f.get());
        }
        else if (settings.waterfalls && f->getString("waterway") == "waterfall")
        {
            ConstGeometryIterator iter(f->getGeometry(), false);
            while (iter.hasMore())
                for (auto& v : *iter.next())
                    fallNodes.emplace_back(v.x() - origin.x(), v.y() - origin.y());
        }
    }

    if (pieces.empty() && areas.empty())
        return field;

    if (progress && progress->isCanceled())
        return nullptr;

    // Sample the natural terrain on a lattice aligned to the data tiling so that
    // neighboring tiles read identical heights where they overlap.
    {
        const double spacing = field->size.x() / 256.0;
        const double pad = buffer + 64.0 / k;
        double x0 = std::floor((origin.x() - pad) / spacing) * spacing;
        double y0 = std::floor((origin.y() - pad) / spacing) * spacing;
        double x1 = std::ceil((origin.x() + field->size.x() + pad) / spacing) * spacing;
        double y1 = std::ceil((origin.y() + field->size.y() + pad) / spacing) * spacing;
        field->demCols = (unsigned)std::lround((x1 - x0) / spacing) + 1u;
        field->demRows = (unsigned)std::lround((y1 - y0) / spacing) + 1u;
        field->demSpacing = spacing;
        field->demOrigin.set(x0 - origin.x(), y0 - origin.y());
        if (!heights || !heights(x0, y0, spacing, spacing, field->demCols, field->demRows, field->dem, progress))
        {
            // Never build water on terrain that failed to load: a blank DEM reads as sea level,
            // and the carved beds would follow it down. No field means the tile is retried.
            return nullptr;
        }
        for (auto& h : field->dem)
            if (!(h > -11000.0f && h < 9000.0f)) h = 0.0f;
    }

    if (progress && progress->isCanceled())
        return nullptr;

    // Channels: splice tile pieces, chain continuations, resample, and fit a smooth
    // water surface profile to the terrain along each one.
    spliceByFeature(pieces, 2.0 / k);
    chainContinuations(pieces, 0.05);

    const float sigma = std::max(settings.levelSmoothing, 1.0f);
    const double stepMeters = std::min(std::max(sigma / 4.0, 4.0), 10.0);
    const double step = stepMeters / k;

    for (auto& piece : pieces)
    {
        Channel ch;
        ch.fid = piece.fid;
        ch.user = piece.user;

        // Resample at fixed spacing.
        std::vector<RiverLayer::WaterwayClass> params;
        double carry = 0.0;
        ch.p.push_back(piece.pts.front());
        params.push_back(piece.params.front());
        for (std::size_t i = 0; i + 1 < piece.pts.size(); ++i)
        {
            osg::Vec2d a = piece.pts[i], b = piece.pts[i + 1];
            double len = (b - a).length();
            double pos = step - carry;
            while (pos < len)
            {
                ch.p.push_back(a + (b - a) * (pos / len));
                params.push_back(piece.params[i]);
                pos += step;
            }
            carry = len - (pos - step);
        }
        if ((ch.p.back() - piece.pts.back()).length() > 0.1 * step)
        {
            ch.p.push_back(piece.pts.back());
            params.push_back(piece.params.back());
        }
        const std::size_t n = ch.p.size();
        if (n < 2)
            continue;

        ch.dir.resize(n);
        ch.s.resize(n);
        ch.halfWidth.resize(n);
        ch.depth.resize(n);
        ch.rapids.resize(n);
        ch.ownsArea.assign(n, 0); // set once the water areas are known
        for (std::size_t i = 0; i < n; ++i)
        {
            osg::Vec2d d = ch.p[std::min(i + 1, n - 1)] - ch.p[i > 0 ? i - 1 : 0];
            d.normalize();
            ch.dir[i].set((float)d.x(), (float)d.y());
            ch.s[i] = i == 0 ? 0.0f : ch.s[i - 1] + (float)((ch.p[i] - ch.p[i - 1]).length() * k);
            ch.halfWidth[i] = 0.5f * params[i].width;
            ch.depth[i] = params[i].depth;
            ch.rapids[i] = params[i].rapids;
        }

        // Terrain under the channel: the lowest of the centerline and two points across
        // it, since mapped centerlines rarely sit exactly in the terrain model's valley.
        std::vector<float> h(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            osg::Vec2d nrm(-ch.dir[i].y(), ch.dir[i].x());
            double r = std::min(std::max((double)ch.halfWidth[i], 4.0), 30.0) / k;
            h[i] = std::min(field->terrainHeight(ch.p[i]),
                std::min(field->terrainHeight(ch.p[i] + nrm * r), field->terrainHeight(ch.p[i] - nrm * r)));
        }

        // Falls split the channel into reaches, each fitted to the terrain on its own.
        std::vector<FallSpan> spans;
        if (settings.waterfalls)
            spans = findFalls(ch, h, fallNodes, stepMeters, k);
        ch.level.assign(n, 0.0f);
        std::size_t reachStart = 0;
        for (std::size_t r = 0; r <= spans.size(); ++r)
        {
            std::size_t reachEnd = r < spans.size() ? spans[r].lip : n - 1;
            fitReach(ch.s, h, reachStart, reachEnd, sigma, ch.level);
            if (r < spans.size())
                reachStart = spans[r].foot;
        }

        // Over a fall the level runs straight from the lip to the foot for now (its slope drives
        // the current and the whitewater); how the water falls is settled once the water areas are
        // known. The plunge pool below churns, and the water rushes toward the lip above.
        ch.fall.assign(n, -1);
        ch.churn.assign(n, 0.0f);
        for (std::size_t f = 0; f < spans.size(); ++f)
        {
            const FallSpan& span = spans[f];
            const float top = ch.level[span.lip];
            const float run = std::max(ch.s[span.foot] - ch.s[span.lip], 1e-3f);
            for (std::size_t i = span.lip; i <= span.foot; ++i)
            {
                if (i > span.lip && i < span.foot)
                    ch.level[i] = top + (ch.level[span.foot] - top) * (ch.s[i] - ch.s[span.lip]) / run;
                ch.fall[i] = (int)(field->falls.size() + f);
            }
            const float drop = std::max(top - ch.level[span.foot], 0.0f);
            const float pool = std::min(8.0f + 0.5f * drop, 60.0f);
            for (std::size_t i = span.foot; i < n && ch.s[i] - ch.s[span.foot] <= pool; ++i)
            {
                float w = 1.0f - (ch.s[i] - ch.s[span.foot]) / pool;
                ch.churn[i] = std::max(ch.churn[i], w * w);
            }
            for (std::size_t i = span.lip + 1; i-- > 0 && ch.s[span.lip] - ch.s[i] <= 10.0f; )
                ch.churn[i] = std::max(ch.churn[i], 0.5f * (1.0f - (ch.s[span.lip] - ch.s[i]) / 10.0f));
        }

        // Surface slope along the flow drives the current and the whitewater.
        ch.slope.resize(n);
        ch.speed.resize(n);
        const std::size_t m = std::max<std::size_t>(1u, (std::size_t)std::lround(20.0 / stepMeters));
        for (std::size_t i = 0; i < n; ++i)
        {
            std::size_t i0 = i >= m ? i - m : 0, i1 = std::min(i + m, n - 1);
            float ds = ch.s[i1] - ch.s[i0];
            float slope = ds > 0.0f ? (ch.level[i0] - ch.level[i1]) / ds : 0.0f;
            ch.slope[i] = std::max(slope, 0.0f);
            double factor = std::sqrt(std::max((double)ch.slope[i], 1e-5) / 0.003);
            ch.speed[i] = (float)(params[i].speed * std::min(std::max(factor, 0.55), 2.6));
        }

        for (const FallSpan& span : spans)
        {
            Fall fall;
            fall.channel = (int)field->channels.size();
            fall.lip = (int)span.lip;
            fall.foot = (int)span.foot;
            fall.top = ch.level[span.lip];
            fall.bottom = ch.level[span.foot];
            fall.mapped = span.mapped;
            field->falls.push_back(fall);
        }
        field->channels.push_back(std::move(ch));
    }

    if (progress && progress->isCanceled())
        return nullptr;

    // Channel segment index.
    {
        osg::Vec2d bmin(-buffer, -buffer), bmax(field->size.x() + buffer, field->size.y() + buffer);
        field->channelGrid.init(bmin, bmax, 40.0 / k);
        for (int c = 0; c < (int)field->channels.size(); ++c)
        {
            const Channel& ch = field->channels[c];
            for (std::size_t i = 0; i + 1 < ch.p.size(); ++i)
            {
                Segment s;
                s.a = ch.p[i], s.b = ch.p[i + 1], s.owner = c, s.index = (int)i;
                field->channelGrid.insert((int)field->channelSegs.size(), s.a, s.b);
                field->channelSegs.push_back(s);
            }
        }
    }

#ifdef OSGEARTH_HAVE_GEOS
    GeosContext ctx;
    GEOSContextHandle_t h = ctx.h;

    const double cx0 = -buffer, cy0 = -buffer;
    const double cx1 = field->size.x() + buffer, cy1 = field->size.y() + buffer;
    GGeom contextRect(h, makeRect(h, cx0, cy0, cx1, cy1, 0.0, origin));

    // Edges along the context border are cuts, not banks: they take part in inside/outside
    // tests but not in distance-to-shore queries.
    const double eps = 1e-6;
    auto onBorder = [&](const osg::Vec2d& a, const osg::Vec2d& b)
    {
        return (std::abs(a.x() - cx0) < eps && std::abs(b.x() - cx0) < eps) ||
            (std::abs(a.x() - cx1) < eps && std::abs(b.x() - cx1) < eps) ||
            (std::abs(a.y() - cy0) < eps && std::abs(b.y() - cy0) < eps) ||
            (std::abs(a.y() - cy1) < eps && std::abs(b.y() - cy1) < eps);
    };

    // Ring edges are binned into horizontal bands for the parity (inside) tests.
    field->bandHeight = 16.0 / k;
    field->bandY0 = cy0;
    const int nb = (int)std::ceil((cy1 - cy0) / field->bandHeight) + 1;
    auto addToBands = [&](std::vector<std::vector<int>>& bands, int i, const Segment& s)
    {
        int b0 = (int)std::floor((std::min(s.a.y(), s.b.y()) - field->bandY0) / field->bandHeight);
        int b1 = (int)std::floor((std::max(s.a.y(), s.b.y()) - field->bandY0) / field->bandHeight);
        for (int b = std::max(b0, 0); b <= std::min(b1, nb - 1); ++b)
            bands[b].push_back(i);
    };

    // Water areas, clipped to the context region and unioned.
    GGeom areaUnion;
    GGeom areaReach; // the same areas as far as the gathered data reaches, for judging channel runs
    {
        std::vector<GEOSGeometry*> parts, fullParts;
        for (const Feature* f : areas)
        {
            std::vector<GEOSGeometry*> polys;
            collectPolygons(h, f->getGeometry(), origin, polys);
            for (GEOSGeometry* poly : polys)
            {
                GGeom raw(h, poly);
                GGeom valid(h, GEOSMakeValid_r(h, raw.get()));
                if (!valid.valid()) continue;
                GGeom clipped(h, GEOSIntersection_r(h, valid.get(), contextRect.get()));
                if (clipped.valid()) parts.push_back(clipped.release());
                fullParts.push_back(valid.release());
            }
        }

        if (!fullParts.empty())
        {
            GGeom collection(h, makeCollection(h, fullParts));
            areaReach = GGeom(h, GEOSUnaryUnion_r(h, collection.get()));
        }

        if (!settings.lakes && !parts.empty())
        {
            // Keep only areas that a channel runs through.
            std::vector<GEOSGeometry*> lines;
            for (auto& ch : field->channels)
                lines.push_back(GEOSGeom_createLineString_r(h, makeSequence(h, ch.p, false)));
            GGeom lineSet(h, makeCollection(h, lines));
            std::vector<GEOSGeometry*> kept;
            for (GEOSGeometry* g : parts)
            {
                if (lineSet.get() && GEOSIntersects_r(h, g, lineSet.get()) == 1)
                    kept.push_back(g);
                else
                    GEOSGeom_destroy_r(h, g);
            }
            parts.swap(kept);
        }

        if (!parts.empty())
        {
            GGeom collection(h, makeCollection(h, parts));
            areaUnion = GGeom(h, GEOSUnaryUnion_r(h, collection.get()));
        }
    }

    if (progress && progress->isCanceled())
        return nullptr;

    // The areas on their own (for TileField::inArea), and the stretches where each channel runs
    // through one far enough to set its level. The tiles carry no tag telling a lake from a river
    // area, but a river runs through its area while the streams around a lake only reach into it.
    if (areaUnion.valid())
    {
        field->areaBands.assign(nb, std::vector<int>());
        field->areaGrid.init(osg::Vec2d(cx0, cy0), osg::Vec2d(cx1, cy1), 24.0 / k);
        forEachRing(h, areaUnion.get(), [&](const std::vector<osg::Vec2d>& pts, bool)
            {
                for (std::size_t i = 0; i + 1 < pts.size(); ++i)
                {
                    Segment s;
                    s.a = pts[i], s.b = pts[i + 1];
                    s.index = onBorder(s.a, s.b) ? 1 : 0;
                    int id = (int)field->areaSegs.size();
                    field->areaSegs.push_back(s);
                    addToBands(field->areaBands, id, s);
                    if (s.index == 0)
                        field->areaGrid.insert(id, s.a, s.b);
                }
            });

        // The profile settles onto the water within a few smoothing sigmas. Runs are measured
        // through the areas as far as the gathered data reaches (well past the context region),
        // so that neighboring fields agree about every run near their shared edge.
        const GEOSPreparedGeometry* reach = areaReach.valid() ? GEOSPrepare_r(h, areaReach.get()) : nullptr;
        const double minRun = std::max(AREA_RUN, 4.0 * (double)sigma);
        for (auto& ch : field->channels)
        {
            if (!reach)
                break;
            const std::size_t n = ch.p.size();
            std::vector<char> through(n, 0);
            for (std::size_t i = 0; i < n; ++i)
            {
                GEOSGeometry* point = GEOSGeom_createPointFromXY_r(h, ch.p[i].x(), ch.p[i].y());
                through[i] = point && GEOSPreparedIntersects_r(h, reach, point) == 1;
                if (point)
                    GEOSGeom_destroy_r(h, point);
            }
            for (std::size_t i = 0; i < n; )
            {
                if (!through[i])
                {
                    ++i;
                    continue;
                }
                std::size_t j = i;
                while (j < n && through[j])
                    ++j;
                if (ch.s[j - 1] - ch.s[i] >= minRun)
                    std::fill(ch.ownsArea.begin() + i, ch.ownsArea.begin() + j, (char)1);
                i = j;
            }
        }
        if (reach)
            GEOSPreparedGeom_destroy_r(h, reach);
    }

    // How each fall falls. Across a mapped water area (a river's falls) the water drops as a
    // curtain (see Fall and TileField::sample). A stream mapped only as a line pours down over
    // the rock: its level follows the terrain along the centerline.
    for (Fall& fall : field->falls)
    {
        Channel& ch = field->channels[fall.channel];
        int owned = 0;
        for (int i = fall.lip; i <= fall.foot; ++i)
            owned += ch.ownsArea[i] ? 1 : 0;
        fall.curtain = 2 * owned > fall.foot - fall.lip + 1;
        if (!fall.curtain)
            for (int i = fall.lip + 1; i < fall.foot; ++i)
                ch.level[i] = std::min(field->terrainHeight(ch.p[i]) + FALL_OFFSET, fall.top);
    }

    // Ribbons for waterways mapped only as lines: buffer the parts that run outside
    // the mapped water areas by half their width.
    std::vector<GEOSGeometry*> waterParts;
    for (const auto& f : features)
    {
        RiverLayer::WaterwayClass params;
        if (!f.valid() || !getChannelParams(f.get(), settings, params))
            continue;
        std::vector<GEOSGeometry*> lines;
        collectLines(h, f->getGeometry(), origin, lines);
        for (GEOSGeometry* line : lines)
        {
            GGeom raw(h, line);
            GGeom clipped(h, GEOSIntersection_r(h, raw.get(), contextRect.get()));
            if (!clipped.valid()) continue;
            GGeom outside;
            if (areaUnion.valid())
                outside = GGeom(h, GEOSDifference_r(h, clipped.get(), areaUnion.get()));
            else
                outside = std::move(clipped);
            if (!outside.valid()) continue;
            double radius = 0.5 * params.width / k;
            GGeom ribbon(h, GEOSBufferWithStyle_r(h, outside.get(), radius, 4, GEOSBUF_CAP_ROUND, GEOSBUF_JOIN_ROUND, 5.0));
            if (ribbon.valid()) waterParts.push_back(ribbon.release());
        }
    }

    // A mapped waterfall on a small stream fans out into a sheet wider than the stream's bed, and
    // has to stay visible from across a valley: it gets a broader ribbon of its own. (A steep run
    // found only in the terrain is a mountain stream tumbling down its own bed.)
    for (const Fall& fall : field->falls)
    {
        if (fall.curtain || !fall.mapped || fall.top - fall.bottom < 8.0f)
            continue;
        const Channel& ch = field->channels[fall.channel];
        std::vector<osg::Vec2d> pts(ch.p.begin() + fall.lip, ch.p.begin() + fall.foot + 1);
        GGeom line(h, GEOSGeom_createLineString_r(h, makeSequence(h, pts, false)));
        GGeom clipped(h, GEOSIntersection_r(h, line.get(), contextRect.get()));
        if (!clipped.valid())
            continue;
        GGeom outside;
        if (areaUnion.valid())
            outside = GGeom(h, GEOSDifference_r(h, clipped.get(), areaUnion.get()));
        else
            outside = std::move(clipped);
        if (!outside.valid())
            continue;
        double radius = std::max(1.8 * (double)ch.halfWidth[fall.lip], 4.0 + 0.01 * (fall.top - fall.bottom)) / k;
        GGeom ribbon(h, GEOSBufferWithStyle_r(h, outside.get(), radius, 4, GEOSBUF_CAP_ROUND, GEOSBUF_JOIN_ROUND, 5.0));
        if (ribbon.valid())
            waterParts.push_back(ribbon.release());
    }

    if (areaUnion.valid())
        waterParts.push_back(areaUnion.release());

    if (waterParts.empty())
        return field;

    GGeom water;
    {
        GGeom collection(h, makeCollection(h, waterParts));
        water = GGeom(h, GEOSUnaryUnion_r(h, collection.get()));
    }
    if (!water.valid())
        return field;

    if (progress && progress->isCanceled())
        return nullptr;

    // Shorelines.
    {
        std::vector<Segment> all;
        forEachRing(h, water.get(), [&](const std::vector<osg::Vec2d>& pts, bool)
            {
                for (std::size_t i = 0; i + 1 < pts.size(); ++i)
                {
                    Segment s;
                    s.a = pts[i], s.b = pts[i + 1];
                    s.index = onBorder(s.a, s.b) ? 1 : 0;
                    all.push_back(s);
                }
            });

        // parity bands use every edge; the distance grid only real banks
        field->bands.assign(nb, std::vector<int>());
        field->shoreGrid.init(osg::Vec2d(cx0, cy0), osg::Vec2d(cx1, cy1), 24.0 / k);
        field->shoreSegs = all;
        for (int i = 0; i < (int)all.size(); ++i)
        {
            const Segment& s = all[i];
            addToBands(field->bands, i, s);
            if (s.index == 0)
                field->shoreGrid.insert(i, s.a, s.b);
        }
    }

    // How wide each fall is where the water lands (for its spray and its curtain): across the
    // mapped water under a river's curtain, or the stream's own width where it is only a line.
    for (Fall& fall : field->falls)
    {
        const Channel& ch = field->channels[fall.channel];
        const int at = fall.curtain ? (fall.lip + fall.foot) / 2 : fall.foot;
        double half = field->shoreDistance(ch.p[at], 2000.0);
        fall.width = (float)std::max(2.0 * half, 2.0 * (double)ch.halfWidth[at]);
    }

    // The tile's own water: clipped to the tile plus a sliver of overlap so neighbors
    // never leave a crack, with clip edges vertexed at shared absolute positions.
    {
        const double overlap = 0.25 / k;
        GGeom tileRect(h, makeRect(h, -overlap, -overlap, field->size.x() + overlap, field->size.y() + overlap,
            4.0 / k, origin));
        GGeom home(h, GEOSIntersection_r(h, water.get(), tileRect.get()));
        if (home.valid())
        {
            field->water = exportPolygons(h, home.get());
        }

        // The wet bank: a band along the shore, from a little way over the water to a few
        // meters onto land. It floats above the waterline, so seen at an angle a band that
        // stopped at the shore would miss the wettest ground; over the water its fragments
        // find river bed below the surface and draw nothing. It only has to cover the wet
        // ground, not trace the shore, so it is built from a simplified outline with mitred
        // corners to keep it light. It comes from the water around the tile (not the clipped
        // piece), and the water is cut a little beyond the tile first to keep buffering cheap.
        const double width = WET_BANK_WIDTH / k;
        const double reach = width + 2.0 / k;
        GGeom nearRect(h, makeRect(h, -reach, -reach, field->size.x() + reach, field->size.y() + reach, 0.0, origin));
        GGeom nearWater(h, GEOSIntersection_r(h, water.get(), nearRect.get()));
        if (nearWater.valid())
        {
            GGeom outline(h, GEOSTopologyPreserveSimplify_r(h, nearWater.get(), WET_BANK_SIMPLIFY / k));
            const GEOSGeometry* base = outline.valid() ? outline.get() : nearWater.get();
            GGeom outer(h, GEOSBufferWithStyle_r(h, base, width, 2, GEOSBUF_CAP_FLAT, GEOSBUF_JOIN_MITRE, 2.0));
            GGeom inner(h, GEOSBufferWithStyle_r(h, base, -WET_BANK_INSET / k, 2, GEOSBUF_CAP_FLAT,
                GEOSBUF_JOIN_MITRE, 2.0));
            GGeom ring;
            if (outer.valid())
                ring = GGeom(h, inner.valid() ? GEOSDifference_r(h, outer.get(), inner.get()) :
                    GEOSGeom_clone_r(h, outer.get()));
            GGeom band(h, ring.valid() ? GEOSIntersection_r(h, ring.get(), tileRect.get()) : nullptr);
            if (band.valid())
                field->bank = exportPolygons(h, band.get());
        }
    }
#else
    OE_WARN << LC << "osgEarth was built without GEOS; rivers are unavailable" << std::endl;
#endif

    return field;
}

//...................................................................

osg::ref_ptr<osg::Node>
RiverDetail::buildMesh(
    const TileField& field,
    const SpatialReference* mapSRS,
    ProgressCallback* progress,
    unsigned* triangleCount)
{
    if (triangleCount) *triangleCount = 0u;

#ifdef OSGEARTH_HAVE_GEOS
    if ((!field.hasMesh() && !field.hasBank()) || !mapSRS)
        return nullptr;

    // Water first, then the wet bank. They meet along the shore but get separate vertices,
    // since the bank floats a little above the water and carries different attributes.
    std::vector<osg::Vec2d> verts, bankVerts;
    std::vector<std::array<unsigned, 3>> tris, bankTris;
    {
        GeosContext ctx;
        if (field.hasMesh())
            triangulate(ctx.h, field.water.get(), verts, tris);
        if (field.hasBank())
            triangulate(ctx.h, field.bank.get(), bankVerts, bankTris);
    }

    if (tris.empty() && bankTris.empty())
        return nullptr;

    if (progress && progress->isCanceled())
        return nullptr;

    const double maxEdge = std::max(field.settings.meshResolution, 1.0f) / field.k;

    // A river's curtain falls need fine triangles where the terrain crosses the middle of the
    // drop, so that the curtain stands sheer instead of leaning across one coarse triangle.
    struct CurtainZone { osg::Vec2d center, dir; double halfLength, halfWidth; float mid, tolerance; };
    std::vector<CurtainZone> zones;
    for (const Fall& fall : field.falls)
    {
        if (!fall.curtain)
            continue;
        const Channel& ch = field.channels[fall.channel];
        const osg::Vec2d a = ch.p[fall.lip], b = ch.p[fall.foot];
        const double len = (b - a).length();
        CurtainZone z;
        z.center = (a + b) * 0.5;
        z.dir = len > 0.0 ? (b - a) / len : osg::Vec2d(1.0, 0.0);
        z.halfLength = 0.5 * len + 15.0 / field.k;
        z.halfWidth = (std::max(0.5 * fall.width, 300.0) + 60.0) / field.k;
        z.mid = 0.5f * (fall.top + fall.bottom);
        z.tolerance = std::max(2.0f, 0.15f * (fall.top - fall.bottom));
        zones.push_back(z);
    }
    std::function<bool(const osg::Vec2d&)> nearCurtain;
    if (!zones.empty())
    {
        nearCurtain = [&](const osg::Vec2d& v)
        {
            for (const CurtainZone& z : zones)
            {
                osg::Vec2d d = v - z.center;
                double along = d * z.dir, across = d.x() * z.dir.y() - d.y() * z.dir.x();
                if (std::abs(along) > z.halfLength || std::abs(across) > z.halfWidth)
                    continue;
                // around where the water drops (see TileField::sample)
                float edge = z.mid - CURTAIN_THROW * std::max(field.terrainSteepness(v), 0.05f);
                if (std::abs(field.terrainHeight(v) - edge) < z.tolerance)
                    return true;
            }
            return false;
        };
    }
    refineMesh(verts, tris, maxEdge, nearCurtain, CURTAIN_EDGE / field.k);
    // The bank only has to follow the smoothed water level, which bends gently; its long,
    // thin triangles can be much longer than the water's.
    refineMesh(bankVerts, bankTris, maxEdge * 4.0);

    // Evaluate the water at every vertex; bank vertices take the level of the nearest water.
    // (The bank reaches a little over the water, so its vertices need the inside test.)
    const std::size_t numWater = verts.size();
    verts.insert(verts.end(), bankVerts.begin(), bankVerts.end());

    const std::size_t nv = verts.size();
    std::vector<RiverLayer::WaterSample> samples(nv);
    for (std::size_t i = 0; i < nv; ++i)
    {
        bool bank = i >= numWater;
        if (!field.sample(verts[i], bank ? 0 : 1, bank ? WET_BANK_WIDTH + 1.0f : 0.0f, samples[i]))
        {
            samples[i].level = field.terrainHeight(verts[i]);
            samples[i].shore = bank ? -WET_BANK_WIDTH : 0.0f;
        }
        if ((i & 1023) == 0 && progress && progress->isCanceled())
            return nullptr;
    }

    // Local frame at the tile center.
    const SpatialReference* geoSRS = mapSRS->getGeographicSRS();
    osg::Vec2d center = field.origin + field.size * 0.5;
    double centerLevel = 0.0;
    for (auto& s : samples) centerLevel += s.level;
    centerLevel /= (double)nv;
    GeoPoint centerPoint(geoSRS, mercToLon(center.x()), mercToLat(center.y()), centerLevel, ALTMODE_ABSOLUTE);
    osg::Matrixd local2world;
    centerPoint.createLocalToWorld(local2world);
    osg::Matrixd world2local;
    world2local.invert(local2world);

    // Texture coordinates are absolute mercator meters relative to an origin snapped to
    // a 1024 m lattice, so wave patterns continue seamlessly across tiles.
    const double uvPeriod = 1024.0;
    osg::Vec2d uvOrigin(std::floor(field.origin.x() / uvPeriod) * uvPeriod, std::floor(field.origin.y() / uvPeriod) * uvPeriod);
    osg::Vec2d uvOffset = field.origin - uvOrigin;

    osg::ref_ptr<osg::Vec3Array> positions = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec2Array> uvs = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec4Array> flow = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec4Array> data = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec2Array> falls = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX); // falling, m fallen
    positions->reserve(nv), normals->reserve(nv), uvs->reserve(nv), flow->reserve(nv), data->reserve(nv);
    falls->reserve(nv);

    const float metersPerUV = (float)field.k;
    for (std::size_t i = 0; i < nv; ++i)
    {
        const osg::Vec2d& v = verts[i];
        const RiverLayer::WaterSample& s = samples[i];
        const bool bank = i >= numWater;
        // the bank floats above the water up to the height that splashes and waves reach
        const float lift = bank ? WET_BANK_HEIGHT + 0.5f * s.turbulence : 0.0f;
        osg::Vec2d m = field.toMercator(v);
        GeoPoint point(geoSRS, mercToLon(m.x()), mercToLat(m.y()), s.level + lift, ALTMODE_ABSOLUTE);
        osg::Vec3d world;
        point.toWorld(world);
        positions->push_back(world * world2local);

        osg::Vec3d up;
        point.createWorldUpVector(up);
        osg::Vec3d upLocal = osg::Matrixd::transform3x3(up, world2local);
        upLocal.normalize();
        normals->push_back(upLocal);

        osg::Vec2d uv = v + uvOffset;
        uvs->push_back(osg::Vec2((float)uv.x(), (float)uv.y()));
        falls->push_back(bank ? osg::Vec2(0.0f, 0.0f) : osg::Vec2(s.fall, s.fallen));

        if (bank)
        {
            // distance from the water (0 over it); kept negative so the shader knows the bank
            float outside = std::max(-s.shore, 0.0f);
            flow->push_back(osg::Vec4(0.0f, 0.0f, 0.0f, s.turbulence));
            data->push_back(osg::Vec4(-std::max(outside, 0.001f), lift, std::min(outside / WET_BANK_WIDTH, 1.0f),
                metersPerUV));
            continue;
        }

        // flow velocity in texture units (mercator meters) per second
        double uvSpeed = s.speed / field.k;
        float turbulence = s.turbulence;
        flow->push_back(osg::Vec4(
            (float)(s.flow.x() * uvSpeed), (float)(s.flow.y() * uvSpeed), s.speed, turbulence));
        data->push_back(osg::Vec4(std::max(s.shore, 0.0f), s.depth, s.across, metersPerUV));
    }

    // One geometry for the water and one for the bank, sharing the vertex arrays.
    auto makeGeometry = [&](const char* name, const std::vector<std::array<unsigned, 3>>& triangles, unsigned base)
    {
        osg::ref_ptr<osg::DrawElementsUInt> elements = new osg::DrawElementsUInt(GL_TRIANGLES);
        elements->reserveElements(triangles.size() * 3);
        for (auto& t : triangles)
        {
            elements->addElement(t[0] + base);
            elements->addElement(t[1] + base);
            elements->addElement(t[2] + base);
        }
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
        geom->setName(name);
        geom->setUseVertexBufferObjects(true);
        geom->setUseDisplayList(false);
        geom->setVertexArray(positions.get());
        geom->setNormalArray(normals.get());
        geom->setTexCoordArray(0, uvs.get());
        geom->setTexCoordArray(1, falls.get());
        geom->setVertexAttribArray(FLOW_ATTRIB, flow.get());
        geom->setVertexAttribArray(DATA_ATTRIB, data.get());
        geom->addPrimitiveSet(elements.get());
        return geom;
    };

    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform(local2world);
    if (!tris.empty())
        xform->addChild(makeGeometry("River water", tris, 0u));
    if (!bankTris.empty())
    {
        // A few meters of wet ground vanish within a couple of pixels beyond this range.
        osg::ref_ptr<osg::Geometry> bank = makeGeometry("River banks", bankTris, (unsigned)numWater);
        osg::ref_ptr<osg::LOD> lod = new osg::LOD();
        lod->addChild(bank.get(), 0.0f, (float)bank->getBound().radius() + WET_BANK_RANGE);
        xform->addChild(lod.get());
    }

    if (triangleCount) *triangleCount = (unsigned)(tris.size() + bankTris.size());
    return xform;
#else
    return nullptr;
#endif
}

osg::ref_ptr<osg::Node>
RiverDetail::buildSpray(const TileField& field, const SpatialReference* mapSRS, unsigned* particleCount)
{
    if (particleCount) *particleCount = 0u;
    if (!mapSRS || field.channels.empty())
        return nullptr;

    // A source of spray: its particles scatter along a short line across the flow.
    struct Emitter
    {
        osg::Vec2d p;           // local mercator
        float level = 0.0f;     // height the particles start from (m)
        osg::Vec2f dir;         // downstream direction (east, north)
        float spread = 1.0f;    // length of the line the particles scatter along (m)
        float radius = 1.0f;    // particle radius (m)
        float rise = 0.0f;      // climb rate (m/s; negative sinks)
        float drift = 0.0f;     // downstream drift (m/s)
        float life = 1.0f;      // particle lifetime (s)
        float density = 1.0f;   // opacity
        int count = 1;          // particles
    };
    std::vector<Emitter> emitters;
    const double k = field.k;
    auto owned = [&](const osg::Vec2d& p)
    {
        return p.x() >= 0.0 && p.y() >= 0.0 && p.x() < field.size.x() && p.y() < field.size.y();
    };

    // Falls: mist boils up out of the plunge pool, the more and the bigger the higher the drop,
    // and a tall fall sheds veils of spray on the way down.
    for (const Fall& fall : field.falls)
    {
        const Channel& ch = field.channels[fall.channel];
        // The mist rises where the water lands: under a river's curtain, or at the foot of a
        // stream pouring down the rock.
        const int at = fall.curtain ? (fall.lip + fall.foot) / 2 : fall.foot;
        const osg::Vec2d& base = ch.p[at];
        if (!owned(base))
            continue;
        const float drop = std::max(fall.top - fall.bottom, 0.0f);
        const float width = std::max(fall.width, 3.0f);
        // A small stream throws up little mist, unless it falls a long way and shatters. A steep
        // run of a mountain stream (found in the terrain, not mapped as a waterfall) only foams.
        const float flow = std::min(std::max(width / 15.0f, 0.25f), 1.0f);
        if (!fall.mapped && !fall.curtain && flow < 0.6f)
            continue;
        const float strength = std::min(0.3f + drop / 30.0f, 1.0f) * std::max(flow, 0.7f * std::min(drop / 200.0f, 1.0f));
        const osg::Vec2f dir = ch.dir[at];
        const osg::Vec2d down(dir.x(), dir.y()), across(-dir.y(), dir.x());
        const int lines = std::min(std::max((int)std::ceil(width / 12.0f), 1), 48);
        for (int i = 0; i < lines; ++i)
        {
            const double offset = ((i + 0.5) / lines - 0.5) * width;
            Emitter e;
            e.p = base + (across * offset + down * (fall.curtain ? 8.0 : 3.0)) / k;
            e.level = fall.bottom + 0.5f;
            e.dir = dir;
            e.spread = width / lines;
            // big falls raise a towering plume that drifts off downstream; a stream's mist is a
            // looser cloud of smaller puffs
            e.radius = std::min(3.0f + 0.4f * drop, 36.0f) * (0.35f + 0.65f * flow);
            e.rise = std::min(0.5f + 0.07f * drop, 5.0f);
            e.drift = std::min(0.6f + 0.03f * drop, 3.0f);
            e.life = std::min(3.0f + 0.08f * drop, 10.0f);
            e.density = strength * (0.55f + 0.45f * flow);
            e.count = flow < 0.6f ? 6 : drop > 25.0f ? 4 : 3;
            emitters.push_back(e);
        }

        if (!fall.curtain && drop > 20.0f)
        {
            const float every = std::max(12.0f, drop / 10.0f);
            float next = every;
            for (int i = fall.lip + 1; i < fall.foot; ++i)
            {
                if (fall.top - ch.level[i] < next)
                    continue;
                next += every;
                Emitter e;
                e.p = ch.p[i];
                e.level = ch.level[i] + 1.0f;
                e.dir = ch.dir[i];
                e.spread = std::max(2.0f * ch.halfWidth[i], 4.0f);
                e.radius = std::min(std::max(0.08f * drop, 3.0f), 14.0f);
                e.rise = -0.4f; // veils settle as they drift off the sheet
                e.drift = 0.9f;
                e.life = 4.0f;
                e.density = 0.55f * strength;
                e.count = 2;
                emitters.push_back(e);
            }
        }
    }

    // Rapids: a fine spray hangs over the fastest whitewater.
    for (const Channel& ch : field.channels)
    {
        float next = -1.0f;
        for (std::size_t i = 0; i < ch.p.size(); ++i)
        {
            if (ch.s[i] < next || ch.fall[i] >= 0 || !owned(ch.p[i]))
                continue;
            // (a stream's whitewater throws no spray worth seeing; a river's does)
            const float turbulence = (float)(hermite(0.012, 0.06, ch.slope[i]) * ch.rapids[i]);
            if (turbulence < 0.75f || ch.speed[i] < 1.8f || ch.halfWidth[i] < 4.0f)
                continue;
            next = ch.s[i] + 30.0f;
            Emitter e;
            e.p = ch.p[i];
            e.level = ch.level[i] + 0.2f;
            e.dir = ch.dir[i];
            e.spread = std::max(1.2f * ch.halfWidth[i], 2.0f);
            e.radius = 2.5f;
            e.rise = 0.35f;
            e.drift = 0.4f * ch.speed[i];
            e.life = 2.4f;
            e.density = 0.3f * turbulence;
            e.count = 2;
            emitters.push_back(e);
        }
    }

    if (emitters.empty())
        return nullptr;

    // Keep the particle count of a tile bounded (a wide fall in a dense rapids network).
    std::size_t total = 0;
    for (auto& e : emitters)
        total += (std::size_t)e.count;
    const std::size_t maxParticles = 3000u;
    const double keepEvery = total > maxParticles ? (double)total / maxParticles : 1.0;

    // Local frame at the tile center.
    const SpatialReference* geoSRS = mapSRS->getGeographicSRS();
    osg::Vec2d center = field.origin + field.size * 0.5;
    double centerLevel = 0.0;
    for (auto& e : emitters)
        centerLevel += e.level;
    centerLevel /= (double)emitters.size();
    GeoPoint centerPoint(geoSRS, mercToLon(center.x()), mercToLat(center.y()), centerLevel, ALTMODE_ABSOLUTE);
    osg::Matrixd local2world, world2local;
    centerPoint.createLocalToWorld(local2world);
    world2local.invert(local2world);

    // Each particle is a quad of four identical corners; the shader expands, lifts, drifts and
    // fades it over its life.
    osg::ref_ptr<osg::Vec3Array> positions = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);  // r = density
    osg::ref_ptr<osg::Vec4Array> shape = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);   // corner xy, seed, radius
    osg::ref_ptr<osg::Vec4Array> motion = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);  // drift east/north, rise, life
    osg::ref_ptr<osg::DrawElementsUInt> elements = new osg::DrawElementsUInt(GL_TRIANGLES);
    osg::BoundingBox bounds;

    auto hash = [](unsigned v)
    {
        v ^= v >> 16; v *= 0x7feb352dU; v ^= v >> 15; v *= 0x846ca68bU; v ^= v >> 16;
        return v;
    };
    auto unit = [](unsigned v) { return (float)(v & 0xFFFFFFu) / 16777216.0f; };
    static const float corners[4][2] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };

    double carry = 0.0;
    unsigned particles = 0u;
    for (std::size_t ei = 0; ei < emitters.size(); ++ei)
    {
        const Emitter& e = emitters[ei];
        const osg::Vec2d down(e.dir.x(), e.dir.y()), across(-e.dir.y(), e.dir.x());
        for (int j = 0; j < e.count; ++j)
        {
            carry += 1.0;
            if (carry < keepEvery)
                continue;
            carry -= keepEvery;

            // Deterministic per particle (seeded by its absolute position) so rebuilt tiles match.
            osg::Vec2d m = field.toMercator(e.p);
            unsigned h = hash((unsigned)(std::llround(m.x() * 4.0) * 73856093LL ^ std::llround(m.y() * 4.0) * 19349663LL)
                ^ (unsigned)(j * 83492791));
            const float seed = unit(h);
            const float r1 = unit(hash(h + 1u)), r2 = unit(hash(h + 2u)), r3 = unit(hash(h + 3u));
            const osg::Vec2d p = e.p + (across * ((r1 - 0.5) * e.spread) + down * ((r2 - 0.5) * 0.6 * e.radius)) / k;
            const float radius = e.radius * (0.75f + 0.5f * r3);
            const float life = e.life * (0.8f + 0.4f * r2);

            osg::Vec2d pm = field.toMercator(p);
            GeoPoint point(geoSRS, mercToLon(pm.x()), mercToLat(pm.y()), e.level, ALTMODE_ABSOLUTE);
            osg::Vec3d world;
            point.toWorld(world);
            osg::Vec3d local = world * world2local;
            osg::Vec3d up;
            point.createWorldUpVector(up);
            osg::Vec3d upLocal = osg::Matrixd::transform3x3(up, world2local);
            upLocal.normalize();

            const unsigned base = (unsigned)positions->size();
            for (int c = 0; c < 4; ++c)
            {
                positions->push_back(local);
                normals->push_back(upLocal);
                colors->push_back(osg::Vec4(e.density, 0.0f, 0.0f, 1.0f));
                shape->push_back(osg::Vec4(corners[c][0], corners[c][1], seed, radius));
                motion->push_back(osg::Vec4(e.dir.x() * e.drift, e.dir.y() * e.drift, e.rise, life));
            }
            elements->addElement(base), elements->addElement(base + 1), elements->addElement(base + 2);
            elements->addElement(base), elements->addElement(base + 2), elements->addElement(base + 3);

            // everywhere the particle can wander over its life
            const float reach = std::abs(e.rise) * life + e.drift * life + 1.3f * radius + 1.0f;
            bounds.expandBy(osg::BoundingBox(local - osg::Vec3d(reach, reach, reach), local + osg::Vec3d(reach, reach, reach)));
            ++particles;
        }
    }
    if (particles == 0u)
        return nullptr;

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    geom->setName("River spray");
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
    geom->setVertexArray(positions.get());
    geom->setNormalArray(normals.get());
    geom->setColorArray(colors.get());
    geom->setTexCoordArray(0, shape.get());
    geom->setTexCoordArray(1, motion.get());
    geom->addPrimitiveSet(elements.get());
    geom->setInitialBound(bounds);

    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform(local2world);
    xform->addChild(geom.get());
    if (particleCount) *particleCount = particles;
    return xform;
}

namespace
{
    //! Douglas-Peucker simplification of a polyline; keeps both ends.
    TileField::Polyline simplifyLine(const TileField::Polyline& in, double tolerance)
    {
        if (in.size() <= 2 || tolerance <= 0.0)
            return in;
        std::vector<char> keep(in.size(), 0);
        keep.front() = keep.back() = 1;
        std::vector<std::pair<std::size_t, std::size_t>> stack{ { 0u, in.size() - 1u } };
        const double tol2 = tolerance * tolerance;
        while (!stack.empty())
        {
            std::pair<std::size_t, std::size_t> span = stack.back();
            stack.pop_back();
            double worst = -1.0, t;
            std::size_t index = 0;
            for (std::size_t i = span.first + 1; i < span.second; ++i)
            {
                double d2 = distance2ToSegment(in[i], in[span.first], in[span.second], t);
                if (d2 > worst)
                    worst = d2, index = i;
            }
            if (worst > tol2)
            {
                keep[index] = 1;
                stack.push_back({ span.first, index });
                stack.push_back({ index, span.second });
            }
        }
        TileField::Polyline out;
        for (std::size_t i = 0; i < in.size(); ++i)
            if (keep[i]) out.push_back(in[i]);
        return out;
    }
}

std::shared_ptr<const TileField::Polylines>
TileField::constraintLines(int toleranceKey, double tolerance) const
{
    {
        std::lock_guard<std::mutex> lock(_linesMutex);
        auto i = _lines.find(toleranceKey);
        if (i != _lines.end())
            return i->second;
    }

    // Fields overlap through their context buffers; each segment belongs to the field whose
    // tile holds its midpoint, so neighboring fields never hand the mesher the same segment.
    auto owned = [&](const osg::Vec2d& a, const osg::Vec2d& b)
    {
        osg::Vec2d m = (a + b) * 0.5;
        return m.x() >= 0.0 && m.y() >= 0.0 && m.x() < size.x() && m.y() < size.y();
    };

    // Banks: the real shoreline only (not where the water was cut at the context border),
    // chained back into lines where consecutive ring segments connect.
    Polylines raw;
    const Segment* prev = nullptr;
    for (const Segment& s : shoreSegs)
    {
        if (s.index != 0 || !owned(s.a, s.b))
        {
            prev = nullptr;
            continue;
        }
        if (!prev || !(prev->b == s.a))
            raw.push_back(Polyline{ s.a });
        raw.back().push_back(s.b);
        prev = &s;
    }

    // Channel centerlines give the terrain mesh vertices along the bed.
    for (const Channel& ch : channels)
    {
        bool continues = false;
        for (std::size_t i = 0; i + 1 < ch.p.size(); ++i)
        {
            if (!owned(ch.p[i], ch.p[i + 1]))
            {
                continues = false;
                continue;
            }
            if (!continues)
                raw.push_back(Polyline{ ch.p[i] });
            raw.back().push_back(ch.p[i + 1]);
            continues = true;
        }
    }

    // Simplifying the whole owned lines (not per terrain tile) keeps neighboring terrain
    // tiles in agreement about where a line crosses their shared edge.
    auto result = std::make_shared<Polylines>();
    for (auto& line : raw)
    {
        Polyline simple = simplifyLine(line, tolerance);
        if (simple.size() >= 2)
            result->push_back(std::move(simple));
    }

    std::lock_guard<std::mutex> lock(_linesMutex);
    std::shared_ptr<const Polylines>& slot = _lines[toleranceKey];
    if (!slot)
        slot = result;
    return slot;
}

void
RiverDetail::getOutlines(const TileField& field, const osg::Vec2d& clipMin, const osg::Vec2d& clipMax,
    int toleranceKey, double tolerance, FeatureList& out)
{
    // Box test in field-local coordinates.
    const osg::Vec2d lo = clipMin - field.origin, hi = clipMax - field.origin;
    auto touches = [&](const osg::Vec2d& a, const osg::Vec2d& b)
    {
        return std::max(a.x(), b.x()) >= lo.x() && std::min(a.x(), b.x()) <= hi.x() &&
            std::max(a.y(), b.y()) >= lo.y() && std::min(a.y(), b.y()) <= hi.y();
    };

    std::shared_ptr<const TileField::Polylines> lines = field.constraintLines(toleranceKey, tolerance);
    osg::ref_ptr<MultiGeometry> parts = new MultiGeometry();
    for (const TileField::Polyline& line : *lines)
    {
        // runs of consecutive segments that touch the box
        osg::ref_ptr<LineString> run;
        for (std::size_t i = 0; i + 1 < line.size(); ++i)
        {
            if (!touches(line[i], line[i + 1]))
            {
                if (run.valid() && run->size() >= 2) parts->add(run.get());
                run = nullptr;
                continue;
            }
            if (!run.valid())
            {
                run = new LineString();
                run->push_back(line[i].x() + field.origin.x(), line[i].y() + field.origin.y(), 0.0);
            }
            run->push_back(line[i + 1].x() + field.origin.x(), line[i + 1].y() + field.origin.y(), 0.0);
        }
        if (run.valid() && run->size() >= 2)
            parts->add(run.get());
    }

    if (!parts->getComponents().empty())
        out.push_back(new Feature(parts.get(), SpatialReference::get("spherical-mercator")));
}
