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
    void scaleCoordsToLOD(double& u, double& v, int baseLOD, const TileKey& key)
    {

        double dL = (double)((int)key.getLOD() - baseLOD);
        double factor = pow(2.0, dL); //exp2(dL);
        double invFactor = 1.0/factor;

        u *= invFactor;
        v *= invFactor;

        if (factor >= 1.0)
        {
            unsigned nx, ny;
            key.getProfile()->getNumTiles(key.getLOD(), nx, ny);

            double tx = (double)key.getTileX();
            double ty = (double)ny - (double)key.getTileY() - 1.0;

            double ax = floor(tx * invFactor);
            double ay = floor(ty * invFactor);
            
            double bx = ax * factor;
            double by = ay * factor;
            
            double cx = bx + factor;
            double cy = by + factor;

            u += (tx - bx) / (cx - bx);
            v += (ty - by) / (cy - by);
        }
    }

    double inline mix(double a, double b, double t)
    {
        return a*(1.0-t) + b*t;
    }

    double inline smoothstep(double a, double b, double t)
    {
        // smoothstep (approximates cosine):
        double mu = t*t*(3.0-2.0*t);
        return a*(1.0-mu) + b*mu;
    }

    bool integrate(const TileKey& key, osg::HeightField* hf, const Geometry* geom, double innerRadius, double outerRadius, ElevationEnvelope* envelope, ProgressCallback* progress)
    {
        bool wroteChanges = false;

        const GeoExtent& ex = key.getExtent();

        double col_interval = ex.width() / (double)(hf->getNumColumns()-1);
        double row_interval = ex.height() / (double)(hf->getNumRows()-1);

        osg::Vec3d P, PROJ, bestA, bestB;
        double bestT;

        double outerRadius2 = outerRadius*outerRadius;

        for (unsigned col = 0; col < hf->getNumColumns(); ++col)
        {
            P.x() = ex.xMin() + (double)col * col_interval;

            for (unsigned row = 0; row < hf->getNumRows(); ++row)
            {
                // check for cancelation periodically
                //if (progress && progress->isCanceled())
                //    return false;

                P.y() = ex.yMin() + (double)row * row_interval;

                double shortestD2 = DBL_MAX;

                ConstGeometryIterator giter(geom, false);
                while (giter.hasMore())
                {
                    const Geometry* part = giter.next();
                
                    for (int i = 0; i < part->size()-1; ++i)
                    {
                        const osg::Vec3d& A = part->at(i);
                        const osg::Vec3d& B = part->at(i+1);
                    
                        osg::Vec3d AB = B - A;

                        double t;  // parameter [0..1] on segment
                        double D2; // shortest distance from point to segment, squared
                        double L2 = AB.length2();
                        osg::Vec3d AP = P - A;

                        if (L2 == 0.0)
                        {
                            // trivial case: segment is zero-length
                            t = 0.0;
                            D2 = AP.length2();
                        }
                        else
                        {
                            // calculate parameter and project our point on to the segment
                            t = osg::clampBetween((AP * AB)/L2, 0.0, 1.0);
                            PROJ.set( A + AB*t );
                            D2 = (P - PROJ).length2();
                        }

                        if (D2 < shortestD2)
                        {
                            shortestD2 = D2;
                            bestA = A;
                            bestB = B;
                            bestT = t;
                        }
                    }
                
                    if (shortestD2 <= outerRadius2)
                    {
                        double shortestD = sqrt(shortestD2);

                        // Blend factor. 0 = distance is less than or equal to the inner radius;
                        //               1 = distance is greater than or equal to the outer radius.
                        double blend = osg::clampBetween(
                            (shortestD - innerRadius) / (outerRadius - innerRadius),
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
                            //elevPROJ = smoothstep(elevA, elevB, bestT);
                        }

                        // smoothstep interpolation of height along the buffer:
                        float h = smoothstep(elevPROJ, elevP, blend);

                        hf->setHeight(col, row, h);

                        wroteChanges = true;
                    }
                }
            }
        }

        return wroteChanges;
    }
}




FlatteningTileSource::FlatteningTileSource() :
TileSource(TileSourceOptions())
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

    // all points <= innerRadius from the centerline are flattened
    double innerRadius = _innerBuffer;

    // all points > innerRadius and <= outerRadius and smoothstep blended
    double outerRadius = _outerBuffer;

    // adjust those values based on latitude in a geographic map
    if (ex.getSRS()->isGeographic())
    {
        double latMid = fabs(ex.yMin() + ex.height()*0.5);
        double metersPerDegAtEquator = (ex.getSRS()->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double metersPerDegree = metersPerDegAtEquator * cos(osg::DegreesToRadians(latMid));
        innerRadius = innerRadius / metersPerDegree;
        outerRadius = outerRadius / metersPerDegree;
    }

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
            // Query a bounding box that's a bit larger than the key so that lines
            // extending outside the box (or close to the box) will still affect
            // the flattening algorithm
            bounds = ex.bounds();
            bounds.expandBy(bounds.xMin() - outerRadius, bounds.yMin() - outerRadius, 0);
            bounds.expandBy(bounds.xMax() + outerRadius, bounds.yMax() + outerRadius, 0);
            bounds.transform(ex.getSRS(), _featureSource->getFeatureProfile()->getSRS());
            query.bounds() = bounds;
        }

        // Query the source.
        osg::ref_ptr<FeatureCursor> cursor = _featureSource->createFeatureCursor(query);

        if (cursor.valid() && cursor->hasMore())
        {
            const SpatialReference* featureSRS = _featureSource->getFeatureProfile()->getSRS();
            bool needsTransform = !featureSRS->isHorizEquivalentTo(key.getExtent().getSRS());

            while (cursor->hasMore())
            {
                Feature* feature = cursor->nextFeature();

                // xform to the key's coordinate system
                if (needsTransform)
                    feature->transform(key.getExtent().getSRS());

                //TODO: test the geometry bounds against the expanded tilekey bounds
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
        osg::ref_ptr<ElevationEnvelope> envelope = _pool->createEnvelope(ex.getSRS(), key.getLOD());

        if(integrate(key, hf, &geoms, innerRadius, outerRadius, envelope, progress) || (progress && progress->isCanceled()))
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
    _pool.setTileSize(65u);

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
    _ts = new FlatteningTileSource();
    _ts->setElevationPool(&_pool);
    _ts->setInnerBuffer(options().flatBuffer().get());
    _ts->setOuterBuffer(options().totalBuffer().get());
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

