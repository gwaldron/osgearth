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
        return mix(a,b,t);
    }

    double inline smootherstep(double a, double b, double t)
    {
        t = t * t*t*(t*(t*6.0 - 15.0) + 10.0);
        return mix(a,b,t);
    }

    // clamp "a" to [lo..hi].
    double inline clamp(double a, double lo, double hi)
    {
        return osg::maximum(osg::minimum(a, hi), lo);
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

    struct Sample {
        double D2;      // distance to segment squared
        osg::Vec3d A;   // endpoint A of segment
        osg::Vec3d B;   // endpoint B of segment
        osg::Vec3d P;   // closest point on segment
        double T;       // segment parameter of closest point
        float elevPROJ; // sampled (terrain) elevation at P
        float elev;     // flattened elevation
        double innerRadius2;
        double outerRadius2;

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
            if (equivalent(samples[i].D2, 0.0)) {
                numer = samples[i].elev, denom = 1.0;
                break;
            }
            else {
                double w = pow(1.0 / samples[i].D2, powerParam);
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
        double BElev = NO_DATA_VALUE;
    };

    using LineSegmentList = std::vector<LineSegment>;

    using LineSegmentIndex = RTree<unsigned, double, 2>;

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

                    // check for duplicate segments (can happen if features share common edges but are modeled as seperate geometries)
                    bool duplicate=false;
                    for(auto& s : segments)
                    {
                        if(A == s.A && B == s.B ||
                           A == s.B && B == s.A)
                        {
                            duplicate=true;
                            break;
                        }
                    }
                    if(!duplicate)
                    {
                        segments.emplace_back(A, B, geomIndex);

                        double min[2] = { osg::minimum(A.x(), B.x()), osg::minimum(A.y(), B.y()) };
                        double max[2] = { osg::maximum(A.x(), B.x()), osg::maximum(A.y(), B.y()) };

                        unsigned int segmentIndex = segments.size() - 1;

                        index.Insert(min, max, segmentIndex);
                    }
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
    bool integrateLines(
        const TileKey& key,
        osg::HeightField* hf, 
        LineSegmentList& segments, 
        LineSegmentIndex& index, 
        const SpatialReference* geomSRS,
        WidthsList& widths,
        ElevationPool* pool, 
        ElevationPool::WorkingSet* workingSet,
        bool fillAllPixels,
        ProgressCallback* progress)
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

            std::vector<unsigned> hits;

            double searchMin[2] = { ex.xMin() - maxBufferDistance, P.y() - maxBufferDistance };
            double searchMax[2] = { ex.xMax() + maxBufferDistance, P.y() + maxBufferDistance };

            index.Search(
                searchMin, searchMax,
                [&hits](const unsigned& hit)
                {
                    hits.push_back(hit);
                    return true;
                });

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
                    double L2 = segment.length2; // length (squared) of segment AB

                    osg::Vec3d AP = P - A;    // vector from endpoint A to point P
                    AP.z() = 0.0; // zero the heights so the distances are calculated correctly in 2d, no matter if height is set on the feature data
                    // P.z() = 0.0; // enable this line if this functions gets called with a non empty heightfield

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
                        PROJ.z() = 0.0; // height is what should be determined, 2D distances

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
                                if (samples[i].D2 > samples[max_i].D2) // FIXME: distance should be changed to a weight factor as there could be line segments from further away that influence as well
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
                            if(t==0.0)
                                b->P = A;
                            else
                                b->P = PROJ;
                            b->T = t;
                            b->innerRadius2 = innerRadius * innerRadius;
                            b->outerRadius2 = outerRadius2;
                        }
                    }
                }

                // // Remove unnecessary sample points that lie on the endpoint of a segment
                // // that abuts another segment in our list.
                // for (unsigned i = 0; i < samples.size();) 
                // {
                //     if (!isSampleValid(&samples[i], samples))
                //     {
                //         samples[i] = samples[samples.size() - 1];
                //         samples.resize(samples.size() - 1);
                //     }
                //     else ++i;
                // }

                // Now that we are done searching for line segments close to our point,
                // we will collect the elevations at our sample points and use them to
                // create a new elevation value for our point.
                if (samples.size() > 0)
                {
                    EP.x() = P.x(), EP.y() = P.y();

                    elevSample = pool->getSample(EP, workingSet);

                    float elevP = elevSample.elevation().getValue();

                    for (unsigned i = 0; i < samples.size(); ++i)
                    {
                        Sample& sample = samples[i];
                        // Sample the elevations at the closest point on AB
                        EP.x() = sample.P.x(), EP.y() = sample.P.y();
                        sample.elevPROJ = pool->getSample(EP, workingSet).elevation();
                        sample.elev = mix(sample.A.z(),sample.B.z(),sample.T);
                        // Blend factor. 0 = inside inner radius, 1 = outside outer radius
                        double blend = lerpstep(sample.innerRadius2,sample.outerRadius2,sample.D2);
                        // smoothstep interpolation of terrain and feature elevation
                        sample.elev = smootherstep(sample.elev, sample.elevPROJ, blend);
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
        LineSegmentList segments;
        LineSegmentIndex index;
        buildSegmentList(geom, segments, index);
        return integrateLines(key, hf, segments, index, geomSRS, widths, pool, workingSet, fillAllPixels, progress);
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

    addDataExtent(DataExtent(
        profile->getExtent(),
        getMinLevel(),
        getMaxDataLevel()));

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

    // Collect all elevation layers excluding this one and use them for flattening.
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

inline void addFeatureIfRelevant(const FlatteningLayer* fl, Feature* feature, osg::ref_ptr< Session >& session, const SpatialReference* workingSRS, const GeoExtent& geoExtent, const GeoExtent& queryExtent, bool needsTransform, MultiGeometry& geoms, WidthsList& widths)
{
    double lineWidth = 0.0;
    double bufferWidth = 0.0;
    if (fl->options().lineWidth().isSet())
    {
        NumericExpression lineWidthExpr(fl->options().lineWidth().get());
        lineWidth = feature->eval(lineWidthExpr, session.get());
    }
    if (fl->options().bufferWidth().isSet())
    {
        NumericExpression bufferWidthExpr(fl->options().bufferWidth().get());
        bufferWidth = feature->eval(bufferWidthExpr, session.get());
    }

    if (lineWidth > 0.0 || bufferWidth > 0.0)
    {
        GeoExtent geoExtent = queryExtent.transform(feature->getSRS()->getGeographicSRS());
        const SpatialReference* geographicSrs = feature->getSRS()->getGeographicSRS();

        lineWidth = SpatialReference::transformUnits(
            Distance(lineWidth, Units::METERS),
            geographicSrs,
            geoExtent.getCentroid().y());

        bufferWidth = SpatialReference::transformUnits(
            Distance(bufferWidth, Units::METERS),
            geographicSrs,
            geoExtent.getCentroid().y());

        // skip features not relevant for this tile
        GeoExtent queryExtentTf = queryExtent.transform(geographicSrs);
        double queryBuffer = 0.5 * lineWidth + bufferWidth;
        GeoExtent featureWithFlattenBounds = feature->getExtent();
        // GeoExtent extentTransformed = profile->clampAndTransformExtent(feature->getExtent()); // ignored that extents possibility cannot be transformed ... it may be an issue
        // where to get a profile from if featureSource has non set?
        featureWithFlattenBounds.transform(geographicSrs);
        featureWithFlattenBounds.expand(queryBuffer,queryBuffer);
        if (featureWithFlattenBounds.intersects(queryExtentTf, true))
        {
            // Transform the feature geometry to our working (projected) SRS.
            if (needsTransform)
                feature->transform(workingSRS);

             if (fl->options().lineWidth().isSet())
            {
                NumericExpression lineWidthExpr(fl->options().lineWidth().get());
                lineWidth = feature->eval(lineWidthExpr, session.get());
            }
            if (fl->options().bufferWidth().isSet())
            {
                NumericExpression bufferWidthExpr(fl->options().bufferWidth().get());
                bufferWidth = feature->eval(bufferWidthExpr, session.get());
            }

            lineWidth = SpatialReference::transformUnits(
                Distance(lineWidth, Units::METERS),
                workingSRS,
                geoExtent.getCentroid().y());

            bufferWidth = SpatialReference::transformUnits(
                Distance(bufferWidth, Units::METERS),
                workingSRS,
                geoExtent.getCentroid().y());

            geoms.getComponents().push_back(feature->getGeometry());
            widths.push_back(Widths(bufferWidth, lineWidth));
        }
    }
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

    if(!options().lineWidth().isSet() && !options().bufferWidth().isSet())
    {
        setStatus(Status(Status::ConfigurationError, "Nothing to do, no flattening parameters"));
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

    // expand queryExtend with the default widths - this is not optimal because through 
    // per-geometry widths that are larger and at a tile border, there could be some neighboring tiles left out!
    // TODO:  JBFIX.  Add a "max" setting somewhere.
    double linewidth = SpatialReference::transformUnits(
        Distance(options().lineWidth().get().eval(), Units::METERS),
        featureSRS,
        geoExtent.getCentroid().y());

    double bufferwidth = SpatialReference::transformUnits(
        Distance(options().bufferWidth().get().eval(), Units::METERS),
        featureSRS,
        geoExtent.getCentroid().y());
    double queryBuffer = 0.5 * linewidth + bufferwidth;
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

        unsigned featureCount = 0;
        std::vector<FeatureList> lists;
        lists.reserve(featureKeys.size());
        for (auto featureKey : featureKeys)
        {
            lists.emplace_back(std::move(const_cast<FlatteningLayer*>(this)->getFeatures(featureKey)));
            featureCount += lists.back().size();
        }
        
        // preallocate some (too much) memory for speed
        geoms.getComponents().reserve(featureCount);
        widths.reserve(featureCount);

        // Now collect all the features we need for this tile.
        for(auto& list : lists)
        {
            for(auto& feature : list)
            {
                addFeatureIfRelevant(this,feature,session,workingSRS,geoExtent,queryExtent,needsTransform,geoms,widths);
            }
        }
    }
    else
    {
        // Non-tiled feature source, just query arbitrary extent:
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
            addFeatureIfRelevant(this,feature,session,workingSRS,geoExtent,queryExtent,needsTransform,geoms,widths);
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
