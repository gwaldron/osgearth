/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthUtil/FlatteningLayer>
#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Map>
#include <osgEarth/Progress>
#include <osgEarth/Utils>
#include <osgEarthFeatures/FeatureCursor>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthSymbology/Query>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[FlatteningTileSource] "

#define OE_TEST OE_DEBUG

namespace
{
    // linear interpolation between a and b
    double inline mix(double a, double b, double t)
    {
        return a + (b-a)*t;
    }

    // smoothstep (cos approx) interpolation between a and b
    double inline smoothstep(double a, double b, double t)
    {
        // smoothstep (approximates cosine):
        double mu = t*t*(3.0-2.0*t);
        return a + (b-a)*mu;
    }
    
    // clamp "a" to [lo..hi].
    double inline clamp(double a, double lo, double hi)
    {
        return std::max(std::min(a, hi), lo);
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
        VECTOR AB = B-A, BC = C-B, CA = A-C;
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
            const POINT& A = vi[i-1];
            const POINT& B = vi[i+1];

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
                    result = (A+B)*0.5;
                }

                // Otherwise, use the midpoint of QV.
                else
                {
                    const POINT& Q = vi[indexBestQ];
                    result = (Q+V)*0.5;
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
            const VECTOR AP = P-A, AB = B-A;
            double t = clamp((AP*AB)/AB.length2(), 0.0, 1.0);
            VECTOR PROJ = A + AB*t;
            double D = (P - PROJ).length2();
            if (D < Dmin) Dmin = D;
        }
        return Dmin;
    }
    
    // Creates a heightfield that flattens an area intersecting the input polygon geometry.
    // The height of the area is found by sampling a point internal to the polygon.
    // bufferWidth = width of transition from flat area to natural terrain.
    bool integratePolygons(const TileKey& key, osg::HeightField* hf, const Geometry* geom, const SpatialReference* geomSRS,
                           double bufferWidth, ElevationEnvelope* envelope, ProgressCallback* progress)
    {
        bool wroteChanges = false;

        const GeoExtent& ex = key.getExtent();

        double col_interval = ex.width() / (double)(hf->getNumColumns()-1);
        double row_interval = ex.height() / (double)(hf->getNumRows()-1);

        POINT Pex, P, internalP;

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
                double minD2 = bufferWidth * bufferWidth; // minimum distance(squared) to closest polygon edge

                const Polygon* bestPoly = 0L;

                ConstGeometryIterator giter(geom, false);
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
                        }

                        // If not in the polygon, how far to the closest edge?
                        else
                        {
                            double D2 = getDistanceSquaredToClosestEdge(P, polygon);
                            if (D2 < minD2)
                            {
                                minD2 = D2;
                                bestPoly = polygon;
                            }
                        }
                    }
                }

                if (bestPoly && minD2 != 0.0)
                {
                    float h;
                    POINT internalP = getInternalPoint(bestPoly);
                    float elevInternal = envelope->getElevation(internalP.x(), internalP.y());

                    if (minD2 < 0.0)
                    {
                        h = elevInternal;
                    }
                    else
                    {
                        float elevNatural = envelope->getElevation(P.x(), P.y());
                        double blend = clamp(sqrt(minD2)/bufferWidth, 0.0, 1.0); // [0..1] 0=internal, 1=natural
                        h = smoothstep(elevInternal, elevNatural, blend);
                    }

                    hf->setHeight(col, row, h);
                    wroteChanges = true;
                }

            }
        }

        return wroteChanges;
    }


    // Create a heightfield that flattens the terrain around linear geometry.
    // lineWidth = width of completely flat area
    // bufferWidth = width of transition from flat area to natural terrain
    bool integrateLines(const TileKey& key, osg::HeightField* hf, const Geometry* geom, const SpatialReference* geomSRS,
                        double lineWidth, double bufferWidth, ElevationEnvelope* envelope, ProgressCallback* progress)
    {
        bool wroteChanges = false;

        const GeoExtent& ex = key.getExtent();

        double col_interval = ex.width() / (double)(hf->getNumColumns()-1);
        double row_interval = ex.height() / (double)(hf->getNumRows()-1);

        osg::Vec3d Pex, P, PROJ, bestA, bestB;
        double bestT;

        double innerRadius = lineWidth * 0.5;
        double outerRadius = innerRadius + bufferWidth;
        double outerRadius2 = outerRadius * outerRadius;

        bool needsTransform = ex.getSRS() != geomSRS;

        //OE_INFO << "ir=" << innerRadius << ", or=" << outerRadius << std::endl;

        for (unsigned col = 0; col < hf->getNumColumns(); ++col)
        {
            Pex.x() = ex.xMin() + (double)col * col_interval;

            for (unsigned row = 0; row < hf->getNumRows(); ++row)
            {
                // check for cancelation periodically
                //if (progress && progress->isCanceled())
                //    return false;

                Pex.y() = ex.yMin() + (double)row * row_interval;

                // Move the point into the working SRS if necessary
                if (needsTransform)
                    ex.getSRS()->transform(Pex, geomSRS, P);
                else
                    P = Pex;

                double closestD2 = DBL_MAX; // closest distance (squared) from P to a segment

                ConstGeometryIterator giter(geom);
                while (giter.hasMore())
                {
                    const Geometry* part = giter.next();
                
                    for (int i = 0; i < part->size()-1; ++i)
                    {
                        const osg::Vec3d& A = (*part)[i];
                        const osg::Vec3d& B = (*part)[i+1];
                    
                        osg::Vec3d AB = B - A;    // current segment AB

                        double t;                 // parameter [0..1] on segment AB
                        double D2;                // shortest distance from point P to segment AB, squared
                        double L2 = AB.length2(); // length (squared) of segment AB
                        osg::Vec3d AP = P - A;    // vector from endpoint A to point P

                        if (L2 == 0.0)
                        {
                            // trivial case: zero-length segment
                            t = 0.0;
                            D2 = AP.length2();
                        }
                        else
                        {
                            // calculate parameter "t" [0..1] which will yield the closest point on AB to P:
                            t = clamp((AP * AB)/L2, 0.0, 1.0);

                            // project our point P onto segment AB:
                            PROJ.set( A + AB*t );

                            // measure the distance from P to the segment AB (squared):
                            D2 = (P - PROJ).length2();
                        }

                        // if P is closer to AB than to any other segment thus far, remember the segment
                        // so we sample it later.
                        if (D2 < closestD2)
                        {
                            closestD2 = D2;
                            bestA = A;
                            bestB = B;
                            bestT = t;
                        }
                    }
                }
                
                if (closestD2 <= outerRadius2)
                {
                    double closestD = sqrt(closestD2);

                    // Blend factor. 0 = distance is less than or equal to the inner radius;
                    //               1 = distance is greater than or equal to the outer radius.
                    double blend = clamp(
                        (closestD - innerRadius) / (outerRadius - innerRadius),
                        0.0, 1.0);

                    float elevP = envelope->getElevation(P.x(), P.y());

                    float elevPROJ;

                    if (bestT == 0.0)
                    {
                        elevPROJ = envelope->getElevation(bestA.x(), bestA.y());
                        if (elevPROJ == NO_DATA_VALUE)
                            elevPROJ = elevP;
                    }
                    else if (bestT == 1.0)
                    {
                        elevPROJ = envelope->getElevation(bestB.x(), bestB.y());
                        if (elevPROJ == NO_DATA_VALUE)
                            elevPROJ = elevP;
                    }
                    else
                    {
                        float elevA = envelope->getElevation(bestA.x(), bestA.y());
                        if (elevA == NO_DATA_VALUE)
                            elevA = elevP;

                        float elevB = envelope->getElevation(bestB.x(), bestB.y());
                        if (elevB == NO_DATA_VALUE)
                            elevB = elevP;

                        // linear interpolation of height from point A to point B:
                        elevPROJ = mix(elevA, elevB, bestT);
                    }

                    // smoothstep interpolation of height along the buffer:
                    float h = smoothstep(elevPROJ, elevP, blend);

                    hf->setHeight(col, row, h);

                    wroteChanges = true;
                }                
            }
        }

        return wroteChanges;
    }
    
    bool integrate(const TileKey& key, osg::HeightField* hf, const Geometry* geom, const SpatialReference* geomSRS,
                   double lineWidth, double bufferWidth, ElevationEnvelope* envelope, ProgressCallback* progress)
    {
        if (geom->isLinear())
            return integrateLines(key, hf, geom, geomSRS, lineWidth, bufferWidth, envelope, progress);
        else
            return integratePolygons(key, hf, geom, geomSRS, bufferWidth, envelope, progress);
    }
}



FlatteningTileSource::FlatteningTileSource(const FlatteningLayerOptions& options) :
TileSource(TileSourceOptions()),
FlatteningLayerOptions(options)
{
    setName("FlatteningTileSource");
}

Status
FlatteningTileSource::initialize(const osgDB::Options* readOptions)
{
    _readOptions = Registry::instance()->cloneOrCreateOptions(readOptions);

    const Profile* profile = getProfile();
    if ( !profile )
    {
        profile = Registry::instance()->getGlobalGeodeticProfile();
        setProfile( profile );
    }

    // ready!
    return Status::OK();
}


osg::HeightField*
FlatteningTileSource::createHeightField(const TileKey& key, ProgressCallback* progress)
{
    if (!_featureSource.valid())
    {
        OE_DEBUG << LC << "No feature source.\n";
        return 0L;
    }

    if (_pool->getElevationLayers().empty())
    {
        OE_WARN << LC << "Internal error - Pool layer set is empty\n";
        return 0L;
    }


    OE_START_TIMER(create);

    const GeoExtent& ex = key.getExtent();

    // We will collection all the feature geometries in this multi:
    MultiGeometry geoms;
      
    // Resolve the list of tile keys that intersect the incoming extent:
    std::vector<TileKey> featureKeys;
    if (_featureSource->getFeatureProfile() && _featureSource->getFeatureProfile()->getProfile())
    {
        _featureSource->getFeatureProfile()->getProfile()->getIntersectingTiles(key, featureKeys);    
    }
    else
    {
        featureKeys.push_back(key);
    }

    const SpatialReference* featureSRS = _featureSource->getFeatureProfile()->getSRS();

    // We must do all the feature processing in a projected system since we're using vector math.
    const SpatialReference* workingSRS =
        ex.getSRS()->isGeographic() ? SpatialReference::get("spherical-mercator") :
        ex.getSRS();

    bool needsTransform = !featureSRS->isHorizEquivalentTo(workingSRS);


    for (int i = 0; i < featureKeys.size(); ++i)
    {
        const TileKey& fkey = featureKeys[i];

        Bounds bounds;

        Query query;

        if (_featureSource->getFeatureProfile()->getTiled())
        {
            // A Tiled source should contain a natural buffer, that is, 
            // the features should extend some distance outside the tile
            // key extents. (At least outerWidth meters).
            query.tileKey() = fkey;
        }
        else
        {
            // Query a bounding box that's a bit larger than the key so that features
            // extending outside the box (or close to the box) will still affect
            // the flattening algorithm
            Distance boundsBuffer = (lineWidth().get()*0.5) + bufferWidth().get();
            double radius = SpatialReference::transformUnits(boundsBuffer, ex.getSRS(), ex.yMin()>=0?ex.yMax():ex.yMin());
            //OE_INFO << "radius = " << radius << std::endl;

            bounds = ex.bounds();
            bounds.expandBy(bounds.xMin() - radius, bounds.yMin() - radius, 0);
            bounds.expandBy(bounds.xMax() + radius, bounds.yMax() + radius, 0);
            bounds.transform(ex.getSRS(), featureSRS);
            query.bounds() = bounds;
        }

        // Query the source.
        osg::ref_ptr<FeatureCursor> cursor = _featureSource->createFeatureCursor(query);

        if (cursor.valid() && cursor->hasMore())
        {
            //const SpatialReference* featureSRS = _featureSource->getFeatureProfile()->getSRS();
            while (cursor->hasMore())
            {
                Feature* feature = cursor->nextFeature();

                // Transform the feature geometry to our working (projected) SRS.
                if (needsTransform)
                    feature->transform(workingSRS);

                //TODO: optimization: test the geometry bounds against the expanded tilekey bounds
                //      in order to discard geometries we don't care about

                geoms.getComponents().push_back(feature->getGeometry());

                //if (progress && progress->isCanceled())
                //    break;
            }
        }
    }

    if (!geoms.getComponents().empty())
    {
        osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(
            ex,
            257, 257,           // base tile size for elevation data
            0u,                 // no border
            true);              // initialize to HAE (0.0) heights

        // Initialize to NO DATA.
        hf->getFloatArray()->assign(hf->getNumColumns()*hf->getNumRows(), NO_DATA_VALUE);

        // Create an elevation query envelope at the LOD we are creating
        osg::ref_ptr<ElevationEnvelope> envelope = _pool->createEnvelope(workingSRS, key.getLOD());

        // Resolve the buffering widths:
        double lineWidthLocal = lineWidth()->as(workingSRS->getUnits());
        double bufferWidthLocal = bufferWidth()->as(workingSRS->getUnits());
        
        if(integrate(key, hf, &geoms, workingSRS, lineWidthLocal, bufferWidthLocal, envelope, progress) || (progress && progress->isCanceled()))
        {
            //double t_create = OE_GET_TIMER(create);
            //OE_INFO << LC << key.str() << " : t=" << t_create << "s\n";

            // If integrate made any changes, return the new heightfield.
            // (Or if the operation was canceled...return it anyway and it 
            // will be discarded).
            return hf.release();
        }
    }

    return 0L;
}

//........................................................................

namespace
{
    struct MapCallbackAdapter : public MapCallback
    {
        MapCallbackAdapter(FlatteningLayer* obj) : _object(obj) { }
        
        void onElevationLayerAdded(ElevationLayer* layer, unsigned index)
        {
            osg::ref_ptr<FlatteningLayer> object;
            if (_object.lock(object))
            {
                if (object->getFlatteningLayerOptions().elevationBaseLayer().isSetTo(layer->getName()))
                {
                    object->setBaseLayer(layer);
                    _activeBaseLayer = layer->getName();
                }
            }
        }

        void onElevationLayerRemoved(ElevationLayer* layer, unsigned index)
        {
            osg::ref_ptr<FlatteningLayer> object;
            if (_object.lock(object))
            {
                if (_activeBaseLayer == layer->getName())
                {
                    object->setBaseLayer(0L);
                    _activeBaseLayer.clear();
                }
            }
        }

        osg::observer_ptr<FlatteningLayer> _object;
        std::string _activeBaseLayer;
    };
}


//........................................................................

#undef  LC
#define LC "[FlatteningLayer] "

FlatteningLayer::FlatteningLayer(const FlatteningLayerOptions& options) :
ElevationLayer(&_localOptionsConcrete),
_localOptions(&_localOptionsConcrete),
_localOptionsConcrete(options),
_mapCallback(0L)
{
    // always call base class initialize
    ElevationLayer::init();
    
    // Experiment with this and see what will work.
    //_pool.setTileSize(65u);
    _pool.setTileSize(257u);

    OE_TEST << LC << "Initialized!\n";
}

const Status&
FlatteningLayer::open()
{
    // ensure the caller named a feature source:
    if (!options().featureSourceOptions().isSet())
    {
        return setStatus(Status::Error(Status::ConfigurationError, "Missing required feature source"));
    }

    // open the feature source:
    _featureSource = FeatureSourceFactory::create(options().featureSourceOptions().get());
    const Status& fsStatus = _featureSource->open(_readOptions.get());
        
    // if the feature source won't open, bail out.
    if (fsStatus.isError())
    {
        return setStatus(fsStatus);
    }

    return ElevationLayer::open();
}

TileSource*
FlatteningLayer::createTileSource()
{
    OE_TEST << LC << "Creating tile source\n";
    _ts = new FlatteningTileSource(options());
    _ts->setElevationPool(&_pool);
    _ts->setFeatureSource(_featureSource.get());
    return _ts;
}

FlatteningLayer::~FlatteningLayer()
{
    //nop
}

void
FlatteningLayer::setBaseLayer(ElevationLayer* layer)
{
    OE_TEST << LC << "Setting base layer to "
        << (layer ? layer->getName() : "null") << std::endl;

    ElevationLayerVector layers;

    if (layer)
    {
        // Instead of using all the map's elevation layers, we will use a custom set
        // consisting of just the base layer. Otherwise, the map will return elevation data
        // for the very layer we are trying to build!
        layers.push_back(layer);
    }

    _pool.setElevationLayers(layers);
}

void
FlatteningLayer::addedToMap(const Map* map)
{   
    if (options().elevationBaseLayer().isSet())
    {  
        // Initialize the elevation pool with our map:
        OE_INFO << LC << "Attaching elevation pool to map\n";
        _pool.setMap( map );

        // Listen for the addition or removal of our base layer:
        _mapCallback = map->addMapCallback(new MapCallbackAdapter(this));

        // see if the base layer is already loaded:
        osg::ref_ptr<ElevationLayer> baseLayer = map->getLayerByName<ElevationLayer>(
            options().elevationBaseLayer().get());

        if (baseLayer.valid())
        {
            setBaseLayer(baseLayer.get());
        }
    }
}

void
FlatteningLayer::removedFromMap(const Map* map)
{
    if (_mapCallback)
        map->removeMapCallback(_mapCallback);
    _mapCallback = 0L;
}

