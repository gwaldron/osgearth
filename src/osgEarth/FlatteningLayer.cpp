/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/FlatteningLayer>
#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Containers>
#include <osgEarth/rtree.h>
#include <osgEarth/Metrics>

using namespace osgEarth;
using namespace osgEarth::Contrib;

REGISTER_OSGEARTH_LAYER(flattenedelevation, FlatteningLayer);
REGISTER_OSGEARTH_LAYER(flattened_elevation, FlatteningLayer);

#define LC "[FlatteningLayer] "

#define OE_TEST OE_DEBUG

namespace
{
    // linear interpolation between a and b
    double inline mix(double a, double b, double t)
    {
        return a + (b - a)*t;
    }

    // smoothstep (cos approx) interpolation between a and b
    double inline smoothstep(double a, double b, double t)
    {
        // smoothstep (approximates cosine):
        t = t * t*(3.0 - 2.0*t);
        return a + (b - a)*t;
    }

    double inline smootherstep(double a, double b, double t)
    {
        t = t * t*t*(t*(t*6.0 - 15.0) + 10.0);
        return a + (b - a)*t;
    }

    // clamp "a" to [lo..hi].
    double inline clamp(double a, double lo, double hi)
    {
        return osg::maximum(osg::minimum(a, hi), lo);
    }

    typedef osg::Vec3d POINT;
    typedef osg::Vec3d VECTOR;

    // iterator that permits the use of looping indexes.
    template<typename T>
    struct CircleIterator {
        CircleIterator(const std::vector<T>& v) : _v(v) { }
        int size() const { return _v.size(); }
        const T& operator[](int i) const { return _v[i % _v.size()]; }
        const std::vector<T>& _v;
    };

    // is P inside the CCW triangle ABC?
    bool triangleContains(const POINT& A, const POINT& B, const POINT& C, const POINT& P)
    {
        VECTOR AB = B - A, BC = C - B, CA = A - C;
        if (((P - A) ^ AB).z() > 0.0) return false;
        if (((P - B) ^ BC).z() > 0.0) return false;
        if (((P - C) ^ CA).z() > 0.0) return false;
        return true;
    }

    // Find a point internal to the polygon.
    // This will not always work with polygons that contain holes,
    // so we need to come up with a different algorithm if this becomes a problem.
    // Maybe try a random point generator and profile it.
    osg::Vec3d inline getInternalPoint(const Polygon* p)
    {
        // Simple test: if the centroid is in the polygon, use it.
        osg::Vec3d centroid = p->getBounds().center();
        if (p->contains2D(centroid.x(), centroid.y()))
            return centroid;

        // Concave/holey polygon, so try the hard way.
        // Ref: http://apodeline.free.fr/FAQ/CGAFAQ/CGAFAQ-3.html

        CircleIterator<POINT> vi(p->asVector());

        for (int i = 0; i < vi.size(); ++i)
        {
            const POINT& V = vi[i];
            const POINT& A = vi[i - 1];
            const POINT& B = vi[i + 1];

            if (((V - A) ^ (B - V)).z() > 0.0) // Convex vertex? (assume CCW winding)
            {
                double minDistQV2 = DBL_MAX;   // shortest distance from test point to candidate point
                int indexBestQ;                // index of best candidate point

                // loop over all other verts (besides A, V, B):
                for (int j = i + 2; j < i + 2 + vi.size() - 3; ++j)
                {
                    const POINT& Q = vi[j];

                    if (triangleContains(A, V, B, Q))
                    {
                        double distQV2 = (Q - V).length2();
                        if (distQV2 < minDistQV2)
                        {
                            minDistQV2 = distQV2;
                            indexBestQ = j;
                        }
                    }
                }

                POINT result;

                // If no inside point was found, return the midpoint of AB.
                if (minDistQV2 == DBL_MAX)
                {
                    result = (A + B)*0.5;
                }

                // Otherwise, use the midpoint of QV.
                else
                {
                    const POINT& Q = vi[indexBestQ];
                    result = (Q + V)*0.5;
                }

                // make sure the resulting point doesn't fall within any of the
                // polygon's holes.
                if (p->contains2D(result.x(), result.y()))
                {
                    return result;
                }
            }
        }

        // Will only happen is holes prevent us from finding an internal point.
        OE_WARN << LC << "getInternalPoint failed miserably\n";
        return p->getBounds().center();
    }

    double getDistanceSquaredToClosestEdge(const osg::Vec3d& P, const Polygon* poly)
    {
        double Dmin = DBL_MAX;
        ConstSegmentIterator segIter(poly, true);
        while (segIter.hasMore())
        {
            const Segment segment = segIter.next();
            const POINT& A = segment.first;
            const POINT& B = segment.second;
            const VECTOR AP = P - A, AB = B - A;
            double t = clamp((AP*AB) / AB.length2(), 0.0, 1.0);
            VECTOR PROJ = A + AB * t;
            double D = (P - PROJ).length2();
            if (D < Dmin) Dmin = D;
        }
        return Dmin;
    }


    struct Widths {
        Widths(const Widths& rhs) {
            bufferWidth = rhs.bufferWidth;
            lineWidth = rhs.lineWidth;
        }

        Widths(double bufferWidth, double lineWidth) {
            this->bufferWidth = bufferWidth;
            this->lineWidth = lineWidth;
        }

        double bufferWidth;
        double lineWidth;
    };

    typedef std::vector<Widths> WidthsList;

    // Creates a heightfield that flattens an area intersecting the input polygon geometry.
    // The height of the area is found by sampling a point internal to the polygon.
    // bufferWidth = width of transition from flat area to natural terrain.
    bool integratePolygons(const TileKey& key, osg::HeightField* hf, const MultiGeometry* geom, const SpatialReference* geomSRS,
        WidthsList& widths, ElevationPool* pool, ElevationPool::WorkingSet* workingSet,
        bool fillAllPixels, ProgressCallback* progress)
    {
        bool wroteChanges = false;

        const GeoExtent& ex = key.getExtent();

        double col_interval = ex.width() / (double)(hf->getNumColumns() - 1);
        double row_interval = ex.height() / (double)(hf->getNumRows() - 1);

        POINT Pex, P, internalP;
        GeoPoint EP(geomSRS, 0, 0, 0);

        bool needsTransform = ex.getSRS() != geomSRS;

        for (unsigned col = 0; col < hf->getNumColumns(); ++col)
        {
            Pex.x() = ex.xMin() + (double)col * col_interval;

            for (unsigned row = 0; row < hf->getNumRows(); ++row)
            {
                // check for cancelation periodically
                //if (progress && progress->isCanceled())
                //    return false;

                Pex.y() = ex.yMin() + (double)row * row_interval;

                if (needsTransform)
                    ex.getSRS()->transform(Pex, geomSRS, P);
                else
                    P = Pex;

                bool done = false;
                double minD2 = DBL_MAX;//bufferWidth * bufferWidth; // minimum distance(squared) to closest polygon edge
                double bufferWidth = 0.0;

                const Polygon* bestPoly = 0L;

                for (unsigned int geomIndex = 0; geomIndex < geom->getNumComponents(); geomIndex++)
                {
                    Geometry* component = geom->getComponents()[geomIndex].get();
                    Widths width = widths[geomIndex];
                    ConstGeometryIterator giter(component, false);
                    while (giter.hasMore() && !done)
                    {
                        const Polygon* polygon = dynamic_cast<const Polygon*>(giter.next());
                        if (polygon)
                        {
                            // Does the point P fall within the polygon?
                            if (polygon->contains2D(P.x(), P.y()))
                            {
                                // yes, flatten it to the polygon's centroid elevation;
                                // and we're dont with this point.
                                done = true;
                                bestPoly = polygon;
                                minD2 = -1.0;
                                bufferWidth = width.bufferWidth;
                            }

                            // If not in the polygon, how far to the closest edge?
                            else
                            {
                                double D2 = getDistanceSquaredToClosestEdge(P, polygon);
                                if (D2 < minD2)
                                {
                                    minD2 = D2;
                                    bestPoly = polygon;
                                    bufferWidth = width.bufferWidth;
                                }
                            }
                        }
                    }
                }

                if (bestPoly && minD2 != 0.0)
                {
                    float h;
                    POINT internalP = getInternalPoint(bestPoly);
                    EP.x() = internalP.x(), EP.y() = internalP.y();
                    float elevInternal = pool->getSample(EP, workingSet).elevation();

                    if (minD2 < 0.0)
                    {
                        h = elevInternal;
                    }
                    else
                    {
                        EP.x() = P.x(), EP.y() = P.y();
                        float elevNatural = pool->getSample(EP, workingSet).elevation();
                        double blend = clamp(sqrt(minD2) / bufferWidth, 0.0, 1.0); // [0..1] 0=internal, 1=natural
                        h = smootherstep(elevInternal, elevNatural, blend);
                    }

                    hf->setHeight(col, row, h);
                    wroteChanges = true;
                }

                else if (fillAllPixels)
                {
                    EP.x() = P.x(), EP.y() = P.y();
                    float h = pool->getSample(EP, workingSet).elevation();
                    hf->setHeight(col, row, h);
                    // do not set wroteChanges
                }
            }
        }

        return wroteChanges;
    }



    struct Sample {
        double D2;      // distance to segment squared
        osg::Vec3d A;   // endpoint of segment
        osg::Vec3d B;   // other endpoint of segment;
        double AElev;   // The elevation at start of segment
        double BElev;   // The elevation at end of segment
        double T;       // segment parameter of closest point

        // used later:
        float elevPROJ; // elevation at point on segment
        float elev;     // flattened elevation
        double D;       // distance

        double innerRadius;
        double outerRadius;

        bool operator < (struct Sample& rhs) const { return D2 < rhs.D2; }
    };

    typedef std::vector<Sample> Samples;

    bool EQ2(const osg::Vec3d& a, const osg::Vec3d& b) {
        return osg::equivalent(a.x(), b.x()) && osg::equivalent(a.y(), b.y());
    }

    bool isSampleValid(const Sample* b1, Samples& samples)
    {
        for (unsigned i = 0; i < samples.size(); ++i)
        {
            Sample* b2 = &samples[i];
            if (b1 == b2) continue;
            if (b1->T == 0.0 && (EQ2(b1->A, b2->A) || EQ2(b1->A, b2->B))) return false;
            if (b1->T == 1.0 && (EQ2(b1->B, b2->A) || EQ2(b1->B, b2->B))) return false;
        }
        return true;
    }

    // Inverse Direct Weighting (IDW) interpolation
    float interpolateSamplesIDW(Samples& samples)
    {
        // higher powerParam corresponds to increased proximity favoring
        const double powerParam = 2.5;

        if (samples.size() == 0) return 0.0f;
        if (samples.size() == 1) return samples[0].elev;

        double numer = 0.0;
        double denom = 0.0;
        for (unsigned i = 0; i < samples.size(); ++i)
        {
            if (osg::equivalent(samples[i].D, 0.0)) {
                numer = samples[i].elev, denom = 1.0;
                break;
            }
            else {
                double w = pow(1.0 / samples[i].D, powerParam);
                numer += w * samples[i].elev;
                denom += w;
            }
        }

        return numer / denom;
    }

    // Linear interpolation (simple mean)
    float interpolateSamplesLinear(Samples& samples)
    {
        double numer = 0.0;
        for (unsigned i = 0; i < samples.size(); ++i)
            numer += samples[i].elev;
        return samples.size() > 0 ? (numer / (double)(samples.size())) : FLT_MAX;
    }

    struct LineSegment
    {
        LineSegment(const osg::Vec3d& _a, const osg::Vec3d& _b, unsigned int _geomIndex) :
            A(_a),
            B(_b),
            geomIndex(_geomIndex)
        {
            AB = B - A;
            length2 = AB.length2();
        }

        osg::Vec3d A;
        osg::Vec3d B;
        osg::Vec3d AB;
        double length2;
        unsigned int geomIndex;
        double AElev = NO_DATA_VALUE;
        double BElev = NO_DATA_VALUE;;
    };

    typedef std::vector< LineSegment > LineSegmentList;

    typedef RTree<unsigned, double, 2> LineSegmentIndex;

    void buildSegmentList(const MultiGeometry* geom, LineSegmentList& segments, LineSegmentIndex& index)
    {
        for (unsigned int geomIndex = 0; geomIndex < geom->getNumComponents(); geomIndex++)
        {
            Geometry* component = geom->getComponents()[geomIndex].get();

            ConstGeometryIterator giter(component);
            while (giter.hasMore())
            {
                const Geometry* part = giter.next();

                for (int i = 0; i < part->size() - 1; ++i)
                {
                    // AB is a candidate line segment:
                    const osg::Vec3d& A = (*part)[i];
                    const osg::Vec3d& B = (*part)[i + 1];
                    segments.emplace_back(A, B, geomIndex);

                    double min[2] = { osg::minimum(A.x(), B.x()), osg::minimum(A.y(), B.y()) };
                    double max[2] = { osg::maximum(A.x(), B.x()), osg::maximum(A.y(), B.y()) };

                    unsigned int segmentIndex = segments.size() - 1;

                    index.Insert(min, max, segmentIndex);
                }
            }
        }
    }

    /**
     * Create a heightfield that flattens the terrain around linear geometry.
     * lineWidth = width of completely flat area
     * bufferWidth = width of transition from flat area to natural terrain
     *
     * Note: this algorithm only samples elevation data from the source (elevation pool).
     * As it progresses, however, it is creating new modified elevation data -- but later
     * points will continue to derive their source data from the original data. This means
     * there will be some discontinuities in the final data, especially along the edges of
     * the flattening buffer.
     *
     * There is not perfect solution for this, but one improvement would be to copy the
     * source elevation into the heightfield as a starting point, and then sample that
     * modifiable heightfield as we go along.
     */
    bool integrateLines(const TileKey& key, osg::HeightField* hf, LineSegmentList& segments, LineSegmentIndex& index, const SpatialReference* geomSRS,
        WidthsList& widths, ElevationPool* pool, ElevationPool::WorkingSet* workingSet,
        bool fillAllPixels, ProgressCallback* progress)
    {
        OE_PROFILING_ZONE;

        double maxBufferDistance = 0.0;
        for (auto& w : widths)
        {
            double d = w.bufferWidth + w.lineWidth;
            if (d > maxBufferDistance) maxBufferDistance = d;
        }

        bool wroteChanges = false;

        GeoExtent ex = key.getExtent();
        if (ex.getSRS() != geomSRS)
        {
            ex = ex.transform(geomSRS);
        }

        double col_interval = ex.width() / (double)(hf->getNumColumns() - 1);
        double row_interval = ex.height() / (double)(hf->getNumRows() - 1);

        osg::Vec3d P, PROJ;
        GeoPoint EP(geomSRS, 0, 0, 0);

        ElevationSample elevSample;

        for (unsigned row = 0; row < hf->getNumRows(); ++row)
        {
            P.y() = ex.yMin() + (double)row * row_interval;

            std::vector< unsigned int > hits;

            double searchMin[2] = { ex.xMin() - maxBufferDistance, P.y() - maxBufferDistance };
            double searchMax[2] = { ex.xMax() + maxBufferDistance, P.y() + maxBufferDistance };
            index.Search(searchMin, searchMax, &hits, ~0u);

            // If there are no hits just skip the whole row.
            if (hits.size() == 0)
            {
                continue;
            }

            for (unsigned col = 0; col < hf->getNumColumns(); ++col)
            {
                P.x() = ex.xMin() + (double)col * col_interval;

                // For each point, we need to find the closest line segments to that point
                // because the elevation values on these line segments will be the flattening
                // value. There may be more than one line segment that falls within the search
                // radius; we will collect up to MaxSamples of these for each heightfield point.
                static const unsigned Maxsamples = 4;
                Samples samples;

                unsigned int hitIndex = 0;
                // What is the maximum number of hits we see in this area?
                for (auto hit : hits)
                {
                    LineSegment& segment = segments[hit];

                    const Widths& w = widths[segment.geomIndex];

                    double innerRadius = w.lineWidth * 0.5;
                    double outerRadius = innerRadius + w.bufferWidth;
                    double outerRadius2 = outerRadius * outerRadius;

                    // AB is a candidate line segment:
                    const osg::Vec3d& A = segment.A;
                    const osg::Vec3d& B = segment.B;

                    //osg::Vec3d AB = B - A;    // current segment AB
                    const osg::Vec3d& AB = segment.AB;

                    double t;                 // parameter [0..1] on segment AB
                    double D2;                // shortest distance from point P to segment AB, squared
                    //double L2 = AB.length2(); // length (squared) of segment AB
                    double L2 = segment.length2; // length (squared) of segment AB

                    osg::Vec3d AP = P - A;    // vector from endpoint A to point P

                    if (L2 == 0.0)
                    {
                        // trivial case: zero-length segment
                        t = 0.0;
                        D2 = AP.length2();
                    }
                    else
                    {
                        // Calculate parameter "t" [0..1] which will yield the closest point on AB to P.
                        // Clamping it means the closest point won't be beyond the endpoints of the segment.
                        t = clamp((AP * AB) / L2, 0.0, 1.0);

                        // project our point P onto segment AB:
                        PROJ.set(A + AB * t);

                        // measure the distance (squared) from P to the projected point on AB:
                        D2 = (P - PROJ).length2();
                    }

                    // If the distance from our point to the line segment falls within
                    // the maximum flattening distance, store it.
                    if (D2 <= outerRadius2)
                    {
                        // see if P is a new sample.
                        Sample* b;
                        if (samples.size() < Maxsamples)
                        {
                            // If we haven't collected the maximum number of samples yet,
                            // just add this to the list:
                            samples.emplace_back(std::move(Sample()));
                            b = &samples.back();
                        }
                        else
                        {
                            // If we are maxed out on samples, find the farthest one we have so far
                            // and replace it if the new point is closer:
                            unsigned max_i = 0;
                            for (unsigned i = 1; i < samples.size(); ++i)
                                if (samples[i].D2 > samples[max_i].D2)
                                    max_i = i;

                            b = &samples[max_i];

                            if (b->D2 < D2)
                                b = 0L;
                        }

                        if (b)
                        {
                            b->D2 = D2;
                            b->A = A;
                            b->B = B;
                            b->T = t;

                            // Sample the segment start and end elevations if they haven't been previously set.
                            if (segment.AElev == NO_DATA_VALUE)
                            {
                                EP.x() = segment.A.x(), EP.y() = segment.A.y();
                                segment.AElev = pool->getSample(EP, workingSet).elevation();
                            }

                            if (segment.BElev == NO_DATA_VALUE)
                            {
                                EP.x() = segment.B.x(), EP.y() = segment.B.y();
                                segment.BElev = pool->getSample(EP, workingSet).elevation();
                            }

                            b->AElev = segment.AElev;
                            b->BElev = segment.BElev;

                            b->innerRadius = innerRadius;
                            b->outerRadius = outerRadius;
                        }
                    }
                }

                // Remove unnecessary sample points that lie on the endpoint of a segment
                // that abuts another segment in our list.
                for (unsigned i = 0; i < samples.size();) {
                    if (!isSampleValid(&samples[i], samples)) {
                        samples[i] = samples[samples.size() - 1];
                        samples.resize(samples.size() - 1);
                    }
                    else ++i;
                }

                // Now that we are done searching for line segments close to our point,
                // we will collect the elevations at our sample points and use them to
                // create a new elevation value for our point.
                if (samples.size() > 0)
                {
                    // The original elevation at our point:
                    //float elevP = pool->getElevation(P.x(), P.y());

                    EP.x() = P.x(), EP.y() = P.y();

                    elevSample = pool->getSample(EP, workingSet);

                    float elevP = elevSample.elevation().getValue();

                    for (unsigned i = 0; i < samples.size(); ++i)
                    {
                        Sample& sample = samples[i];

                        sample.D = sqrt(sample.D2);

                        // Blend factor. 0 = distance is less than or equal to the inner radius;
                        //               1 = distance is greater than or equal to the outer radius.
                        double blend = clamp(
                            (sample.D - sample.innerRadius) / (sample.outerRadius - sample.innerRadius),
                            0.0, 1.0);

                        if (sample.T == 0.0)
                        {
                            sample.elevPROJ = sample.AElev;
                            if (sample.elevPROJ == NO_DATA_VALUE)
                                sample.elevPROJ = elevP;
                        }
                        else if (sample.T == 1.0)
                        {
                            sample.elevPROJ = sample.BElev;
                            if (sample.elevPROJ == NO_DATA_VALUE)
                                sample.elevPROJ = elevP;
                        }
                        else
                        {
                            float elevA = sample.AElev;
                            if (elevA == NO_DATA_VALUE)
                                elevA = elevP;

                            float elevB = sample.BElev;
                            if (elevB == NO_DATA_VALUE)
                                elevB = elevP;

                            // linear interpolation of height from point A to point B on the segment:
                            sample.elevPROJ = mix(elevA, elevB, sample.T);
                        }

                        // smoothstep interpolation of along the buffer (perpendicular to the segment)
                        // will gently integrate the new value into the existing terrain.
                        sample.elev = smootherstep(sample.elevPROJ, elevP, blend);
                    }

                    // Finally, combine our new elevation values and set the new value in the output.
                    float finalElev = interpolateSamplesIDW(samples);
                    if (finalElev < FLT_MAX)
                        hf->setHeight(col, row, finalElev);
                    else
                        hf->setHeight(col, row, elevP);

                    wroteChanges = true;
                }

                else if (fillAllPixels)
                {
                    // No close segments were found, so just copy over the source data.
                    EP.x() = P.x(), EP.y() = P.y();
                    float h = pool->getSample(EP, workingSet).elevation();
                    hf->setHeight(col, row, h);

                    // Note: do not set wroteChanges to true.
                }
            }
        }

        return wroteChanges;
    }


    bool integrate(const TileKey& key, osg::HeightField* hf, const MultiGeometry* geom, const SpatialReference* geomSRS,
        WidthsList& widths, ElevationPool* pool, ElevationPool::WorkingSet* workingSet,
        bool fillAllPixels, ProgressCallback* progress)
    {
        if (geom->isLinear())
        {
            LineSegmentList segments;
            LineSegmentIndex index;
            buildSegmentList(geom, segments, index);
            return integrateLines(key, hf, segments, index, geomSRS, widths, pool, workingSet, fillAllPixels, progress);
        }
        else
            return integratePolygons(key, hf, geom, geomSRS, widths, pool, workingSet, fillAllPixels, progress);
    }
}

//........................................................................

Config
FlatteningLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();

    featureSource().set(conf, "features");

    if (filters().empty() == false)
    {
        Config temp;
        for (unsigned i = 0; i < filters().size(); ++i)
            temp.add(filters()[i].getConfig());
        conf.set("filters", temp);
    }

    conf.set("line_width", _lineWidth);
    conf.set("buffer_width", _bufferWidth);
    conf.set("fill", _fill);

    if (_script.valid())
    {
        Config scriptConf("script");

        if (!_script->name.empty())
            scriptConf.set("name", _script->name);
        if (!_script->language.empty())
            scriptConf.set("language", _script->language);
        if (_script->uri.isSet())
            scriptConf.set("url", _script->uri->base());
        if (!_script->profile.empty())
            scriptConf.set("profile", _script->profile);
        else if (!_script->code.empty())
            scriptConf.setValue(_script->code);

        conf.add(scriptConf);
    }

    return conf;
}

void
FlatteningLayer::Options::fromConfig(const Config& conf)
{
    fill().init(false);
    lineWidth().init(40);
    bufferWidth().init(40);
    URIContext uriContext = URIContext(conf.referrer());

    featureSource().get(conf, "features");

    const Config& filtersConf = conf.child("filters");
    for (ConfigSet::const_iterator i = filtersConf.children().begin(); i != filtersConf.children().end(); ++i)
        filters().push_back(ConfigOptions(*i));

    conf.get("line_width", _lineWidth);
    conf.get("buffer_width", _bufferWidth);
    conf.get("fill", _fill);

    // TODO:  Separate out ScriptDef from Stylesheet and include it as a standalone class, along with this loading code.
    ConfigSet scripts = conf.children("script");
    for (ConfigSet::iterator i = scripts.begin(); i != scripts.end(); ++i)
    {
        _script = new StyleSheet::ScriptDef();

        // load the code from a URI if there is one:
        if (i->hasValue("url"))
        {
            _script->uri = URI(i->value("url"), uriContext);
            OE_INFO << "Loading script from \"" << _script->uri->full() << std::endl;
            _script->code = _script->uri->getString();
        }
        else
        {
            _script->code = i->value();
        }

        // name is optional and unused at the moment
        _script->name = i->value("name");

        std::string lang = i->value("language");
        _script->language = lang.empty() ? "javascript" : lang;

        std::string profile = i->value("profile");
        _script->profile = profile;
    }
}

//........................................................................

OE_LAYER_PROPERTY_IMPL(FlatteningLayer, NumericExpression, LineWidth, lineWidth);
OE_LAYER_PROPERTY_IMPL(FlatteningLayer, NumericExpression, BufferWidth, bufferWidth);
OE_LAYER_PROPERTY_IMPL(FlatteningLayer, bool, Fill, fill);

void
FlatteningLayer::init()
{
    ElevationLayer::init();

    // Experiment with this and see what will work.
    _pool = new ElevationPool();
}

Config
FlatteningLayer::getConfig() const
{
    Config c = ElevationLayer::getConfig();
    return c;
}

Status
FlatteningLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    // ensure the caller named a feature source:
    Status fsStatus = options().featureSource().open(getReadOptions());
    if (fsStatus.isError())
        return fsStatus;

    _filterChain = FeatureFilterChain::create(options().filters(), getReadOptions());
    
    const Profile* profile = getProfile();
    if (!profile)
    {
        profile = Profile::create(Profile::GLOBAL_GEODETIC);
        setProfile(profile);
    }

    return Status::NoError;
}

FlatteningLayer::~FlatteningLayer()
{
    //nop
}

void
FlatteningLayer::setFeatureSource(FeatureSource* layer)
{
    options().featureSource().setLayer(layer);
}

void
FlatteningLayer::addedToMap(const Map* map)
{
    ElevationLayer::addedToMap(map);

    // Initialize the elevation pool with our map:
    OE_INFO << LC << "Attaching elevation pool to map\n";
    _pool->setMap(map);

    options().featureSource().addedToMap(map);

    // Collect all elevation layers preceding this one and use them for flattening.
    ElevationLayerVector layers;
    map->getLayers(layers);
    for (ElevationLayerVector::iterator i = layers.begin(); i != layers.end(); ++i) {
        if (i->get() == this) {
            layers.erase(i);
            break;
        }
        else {
            OE_INFO << LC << "Using: " << i->get()->getName() << "\n";
        }
    }
    if (!layers.empty())
    {
        _elevWorkingSet.setElevationLayers(layers);
    }
}

void
FlatteningLayer::removedFromMap(const Map* map)
{
    options().featureSource().removedFromMap(map);

    ElevationLayer::removedFromMap(map);
}

FeatureList
FlatteningLayer::getFeatures(const TileKey& key)
{
    OpenThreads::ScopedLock< Threading::Mutex > lk(_featuresCacheMutex);

    FeaturesLRU::Record result;
    _featuresCache.get(key, result);
    if (result.valid())
    {
        return result.value();
    }

    Query query;
    query.tileKey() = key;

    FeatureList features;
    osg::ref_ptr<FeatureCursor> cursor = getFeatureSource()->createFeatureCursor(query, _filterChain.get(), nullptr, nullptr);
    if (cursor.valid())
    {
        cursor->fill(features);
        _featuresCache.insert(key, features);
    }
    return features;
}

GeoHeightField
FlatteningLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return GeoHeightField::INVALID;
    }

    if (!getFeatureSource())
    {
        setStatus(Status(Status::ServiceUnavailable, "No feature source"));
        return GeoHeightField::INVALID;
    }

    if (getFeatureSource()->getStatus().isError())
    {
        setStatus(getFeatureSource()->getStatus());
        return GeoHeightField::INVALID;
    }

    const FeatureProfile* featureProfile = getFeatureSource()->getFeatureProfile();
    if (!featureProfile)
    {
        setStatus(Status(Status::ConfigurationError, "Feature profile is missing"));
        return GeoHeightField::INVALID;
    }

    const SpatialReference* featureSRS = featureProfile->getSRS();
    if (!featureSRS)
    {
        setStatus(Status(Status::ConfigurationError, "Feature profile has no SRS"));
        return GeoHeightField::INVALID;
    }

    //if (_pool->getElevationLayers().empty())
    //{
    //    OE_WARN << LC << "Internal error - Pool layer set is empty\n";
    //    return GeoHeightField::INVALID;
    //}

    // If the feature source has a tiling profile, we are going to have to map the incoming
    // TileKey to a set of intersecting TileKeys in the feature source's tiling profile.
    GeoExtent queryExtent;
    if (featureProfile->getTilingProfile())
        queryExtent = featureProfile->getTilingProfile()->clampAndTransformExtent(key.getExtent());
    else
        queryExtent = key.getExtent().transform(featureSRS);

    if (!queryExtent.isValid())
    {
        return GeoHeightField::INVALID;
    }

    // Lat/Long extent:
    GeoExtent geoExtent = queryExtent.transform(featureSRS->getGeographicSRS());
    if (!geoExtent.isValid())
    {
        return GeoHeightField::INVALID;
    }

    // Buffer the query extent to include the potentially flattened area.
    /*
    double linewidth = SpatialReference::transformUnits(
        options().lineWidth().get(),
        featureSRS,
        geoExtent.getCentroid().y());

    double bufferwidth = SpatialReference::transformUnits(
        options().bufferWidth().get(),
        featureSRS,
        geoExtent.getCentroid().y());
    */
    // TODO:  JBFIX.  Add a "max" setting somewhere.
    double linewidth = 10.0;
    double bufferwidth = 10.0;
    double queryBuffer = 0.5*linewidth + bufferwidth;
    queryExtent.expand(queryBuffer, queryBuffer);

    // We must do all the feature processing in a projected system since we're using vector math.
#if 0
    osg::ref_ptr<const SpatialReference> workingSRS = queryExtent.getSRS();
    //if (workingSRS->isGeographic())
    {
        osg::Vec3d refPos = queryExtent.getCentroid();
        workingSRS = workingSRS->createTangentPlaneSRS(refPos);
    }
#else
    const SpatialReference* workingSRS = queryExtent.getSRS()->isGeographic() ? SpatialReference::get("spherical-mercator") :
        queryExtent.getSRS();
#endif

    bool needsTransform = !featureSRS->isHorizEquivalentTo(workingSRS);

    osg::ref_ptr< StyleSheet > styleSheet = new StyleSheet();
    styleSheet->setScript(options().getScript());
    osg::ref_ptr< Session > session = new Session(_map.get(), styleSheet.get());

    // We will collection all the feature geometries in this multigeometry:
    MultiGeometry geoms;
    WidthsList widths;

    if (featureProfile->getTilingProfile())
    {
        // Tiled source, must resolve complete set of intersecting tiles:
        std::vector<TileKey> intersectingKeys;
        featureProfile->getTilingProfile()->getIntersectingTiles(queryExtent, key.getLOD(), intersectingKeys);

        std::unordered_set<TileKey> featureKeys;
        for (int i = 0; i < intersectingKeys.size(); ++i)
        {
            if ((int)intersectingKeys[i].getLOD() > featureProfile->getMaxLevel())
                featureKeys.insert(intersectingKeys[i].createAncestorKey(featureProfile->getMaxLevel()));
            else
                featureKeys.insert(intersectingKeys[i]);
        }

        // Query and collect all the features we need for this tile.
        for (std::unordered_set<TileKey>::const_iterator i = featureKeys.begin(); i != featureKeys.end(); ++i)
        {
            Query query;
            query.tileKey() = *i;

            FeatureList features = const_cast<FlatteningLayer*>(this)->getFeatures(*i);
            for (auto& feature: features)
            {
                double lineWidth = 0.0;
                double bufferWidth = 0.0;
                if (options().lineWidth().isSet())
                {
                    NumericExpression lineWidthExpr(options().lineWidth().get());
                    lineWidth = feature->eval(lineWidthExpr, session.get());
                }

                if (options().bufferWidth().isSet())
                {
                    NumericExpression bufferWidthExpr(options().bufferWidth().get());
                    bufferWidth = feature->eval(bufferWidthExpr, session.get());
                }

                // Transform the feature geometry to our working (projected) SRS.
                if (needsTransform)
                    feature->transform(workingSRS);

                lineWidth = SpatialReference::transformUnits(
                    Distance(lineWidth, Units::METERS),
                    featureSRS,
                    geoExtent.getCentroid().y());

                bufferWidth = SpatialReference::transformUnits(
                    Distance(bufferWidth, Units::METERS),
                    featureSRS,
                    geoExtent.getCentroid().y());

                //TODO: optimization: test the geometry bounds against the expanded tilekey bounds
                //      in order to discard geometries we don't care about
                geoms.getComponents().push_back(feature->getGeometry());
                widths.push_back(Widths(bufferWidth, lineWidth));
            }
        }
    }
    else
    {
        // Non-tiled feaure source, just query arbitrary extent:
        // Set up the query; bounds must be in the feature SRS:
        Query query;
        query.bounds() = queryExtent.bounds();

        osg::ref_ptr<FeatureCursor> cursor = getFeatureSource()->createFeatureCursor(
            query,
            _filterChain.get(),
            nullptr,
            progress);

        // Run the query and fill the list.
        while (cursor.valid() && cursor->hasMore())
        {
            Feature* feature = cursor->nextFeature();

            double lineWidth = 0.0;
            double bufferWidth = 0.0;
            if (options().lineWidth().isSet())
            {
                NumericExpression lineWidthExpr(options().lineWidth().get());
                lineWidth = feature->eval(lineWidthExpr, session.get());
            }

            if (options().bufferWidth().isSet())
            {
                NumericExpression bufferWidthExpr(options().bufferWidth().get());
                bufferWidth = feature->eval(bufferWidthExpr, session.get());
            }

            // Transform the feature geometry to our working (projected) SRS.
            if (needsTransform)
                feature->transform(workingSRS);

            lineWidth = SpatialReference::transformUnits(
                Distance(lineWidth, Units::METERS),
                featureSRS,
                geoExtent.getCentroid().y());

            bufferWidth = SpatialReference::transformUnits(
                Distance(bufferWidth, Units::METERS),
                featureSRS,
                geoExtent.getCentroid().y());

            // Transform the feature geometry to our working (projected) SRS.
            if (needsTransform)
                feature->transform(workingSRS);

            //TODO: optimization: test the geometry bounds against the expanded tilekey bounds
            //      in order to discard geometries we don't care about

            geoms.getComponents().push_back(feature->getGeometry());
            widths.push_back(Widths(bufferWidth, lineWidth));
        }
    }

    if (!geoms.getComponents().empty())
    {
        // Make an empty heightfield to populate:
        osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(
            queryExtent,
            osgEarth::ELEVATION_TILE_SIZE,
            osgEarth::ELEVATION_TILE_SIZE,
            0u,                 // no border
            true);              // initialize to HAE (0.0) heights

        // Initialize to NO DATA.
        hf->getFloatArray()->assign(hf->getNumColumns()*hf->getNumRows(), NO_DATA_VALUE);

        bool fill = (options().fill() == true);

        bool wrote_to_hf = integrate(
            key,
            hf.get(),
            &geoms,
            workingSRS,
            widths,
            _pool.get(),
            &_elevWorkingSet,
            fill,
            progress);

        if (wrote_to_hf)
        {
            return GeoHeightField(hf.get(), key.getExtent());
        }
        else
        {
            return GeoHeightField::INVALID;
        }
    }
    else
    {
        return GeoHeightField::INVALID;
    }
}
