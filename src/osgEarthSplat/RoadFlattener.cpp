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
#include "RoadFlattener"
#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarthFeatures/FeatureCursor>
#include <osgEarthSymbology/Query>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarth/Map>

using namespace osgEarth;
using namespace osgEarth::Splat;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define LC "[RoadHFTileSource] "


#define USE_BASE_LAYER

namespace
{
    Geometry* cubic(const Geometry* in, double size)
    {
        Geometry* out = new LineString();
        //out->push_back(in->front());
        for (Geometry::const_iterator i = in->begin(); i != in->end()-1; ++i)
        {
            const osg::Vec3d& p0 = i == in->begin() ? *i : *(i-1);
            const osg::Vec3d& p1 = *(i);
            const osg::Vec3d& p2 = *(i+1);
            const osg::Vec3d& p3 = (i+2 == in->end())? *(i+1) : *(i+2);

            double len = (p2 - p1).length();
            int segs = ceil(len/size);
            if (segs>1)
            {
            //    OE_INFO << "segs = " << segs << "\n";
                for (int k = 0; k < segs; ++k)
                {
                    double t = (double)k / (double)segs;
                    osg::Vec3d a0 = p3-p2-p0+p1;
                    osg::Vec3d a1 = p0-p1-a0;
                    osg::Vec3d a2 = p2-p0;
                    osg::Vec3d a3 = p1;
                    osg::Vec3d p = (a0*t*t*t + a1*t*t + a2*t + a3);
                    out->push_back(p);
                }
            }
            else
            {
                out->push_back(p1);
            }
        }
        out->push_back(in->back());
        return out;
    }

    double inline mix(double a, double b, double t)
    {
        return a*(1.0-t) + b*t;
    }

    bool integrate(const TileKey& key, osg::HeightField* hf, const Geometry* geom, double innerWidth, double outerWidth, ElevationEnvelope* envelope)
    {
        bool debug = key.str() == "13/3117/2210";
        if (debug)
            int k=0;

        bool wroteChanges = false;

        const GeoExtent& ex = key.getExtent();

        double col_interval = ex.width() / (double)(hf->getNumColumns()-1);
        double row_interval = ex.height() / (double)(hf->getNumRows()-1);

        osg::Vec3d P, PROJ, bestA, bestB;
        double bestT;

        for (unsigned col = 0; col < hf->getNumColumns(); ++col)
        {
            P.x() = ex.xMin() + (double)col * col_interval;

            for (unsigned row = 0; row < hf->getNumRows(); ++row)
            {
                P.y() = ex.yMin() + (double)row * row_interval;

                double shortestD2 = DBL_MAX;

                for (int i = 0; i < geom->size()-1; ++i)
                {
                    const osg::Vec3d& A = geom->at(i);
                    const osg::Vec3d& B = geom->at(i+1);
                    
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
                        PROJ = A;
                    }
                    else
                    {
                        // calculate parameter and project our point on to the segment
                        t = osg::clampBetween((AP * AB)/L2, 0.0, 1.0);
                        PROJ = A + AB*t;
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
                
                if (true) //shortestD != DBL_MAX) //<= ex.width() && shortestD <= ex.height()) //outerWidth || bestT > 0.0 || bestT < 1.0)
                {
                    double shortestD = sqrt(shortestD2);

                    double blend = osg::clampBetween(
                        (shortestD - innerWidth) / (outerWidth - innerWidth),
                        0.0, 1.0);

                    if (blend < 1.0) // try "true", it's smooth but some segments get lost... there's prob an error below.
                    {
                        float elevPROJ;

                        if (bestT == 0.0)
                        {
                            elevPROJ = envelope->getElevation(bestA.x(), bestA.y());
                        }
                        else if (bestT == 1.0)
                        {
                            elevPROJ = envelope->getElevation(bestB.x(), bestB.y());
                        }
                        else
                        {
                            elevPROJ = mix(
                                envelope->getElevation(bestA.x(), bestA.y()),
                                envelope->getElevation(bestB.x(), bestB.y()),
                                bestT );
                        }
                    
                        float elevP = envelope->getElevation(P.x(), P.y());
                    
                        float h = mix(elevPROJ, elevP, blend);

                        hf->setHeight(col, row, h);

                        wroteChanges = true;
                    }
                }
            }
        }

        return wroteChanges;
    }
}




RoadHFTileSource::RoadHFTileSource(const RoadHFOptions& options) :
TileSource(options),
RoadHFOptions(options)
{
    setName("RoadHF");

    // We want full-size elevation tiles in the sampling pool.
    _pool.setTileSize(129u); //33u);
}

Status
RoadHFTileSource::initialize(const osgDB::Options* readOptions)
{
    _readOptions = Registry::instance()->cloneOrCreateOptions(readOptions);

    const Profile* profile = getProfile();
    if ( !profile )
    {
        profile = Registry::instance()->getGlobalGeodeticProfile();
        setProfile( profile );
    }

#ifdef USE_BASE_LAYER

    if (!elevationBaseLayer().isSet())
    {
        return Status::Error(Status::ConfigurationError, "Required property base_layer is not set");
    }

    const Map* map = dynamic_cast<const Map*>(osg::getUserObject(readOptions, "osgEarth.Map"));
    if (!map)
    {
        OE_WARN << LC << "Massive failure! No Map in the readOptions!\n";
        return Status::Error(Status::AssertionFailure, "No osgEarth::Map found in read options");
    }

    _elevationLayer = map->getLayerByName<ElevationLayer>(elevationBaseLayer().get());
    if (!_elevationLayer.valid())
    {
        return Status::Error(Status::ServiceUnavailable, "Specified base layer could not be found in the Map");
    }

    if (!_elevationLayer->getStatus().isOK())
    {
        return Status::Error(Status::ServiceUnavailable, "Specified base layer reported an error");
    }

    // Set up the elevation pool with our map:
    _pool.setMap( map );

    // Instead of using all the map's elevation layers, we will use a custom set
    // consisting of just the base layer. Otherwise, the map will return elevation data
    // for the very layer we are trying to build!
    ElevationLayerVector layers;
    layers.push_back(_elevationLayer.get());
    _pool.setElevationLayers(layers);

#else
    // load up the source elevation layer:
    if (!elevationLayerOptions().isSet())
        return Status::Error(Status::ConfigurationError, "Missing required elevation layer");

    ElevationLayerOptions elo;
    elo.driver() = elevationLayerOptions().get();
    _elevationLayer = new ElevationLayer(elo);
    _elevationLayer->setReadOptions(readOptions);
    const Status& elStatus = _elevationLayer->open();
    if (elStatus.isError())
        return elStatus;
#endif

    // load up the feature source for flattening vectors
    if (featureSourceOptions().isSet())
    {
        _featureSource = FeatureSourceFactory::create(featureSourceOptions().get());
        const Status& fsStatus = _featureSource->open(_readOptions.get());
        if (fsStatus.isError())
            return fsStatus;
    }

    // ready!
    return Status::OK();
}

CachePolicy
RoadHFTileSource::getCachePolicyHint() const
{
    return CachePolicy::NO_CACHE;
}


osg::HeightField*
RoadHFTileSource::createHeightField(const TileKey& key, ProgressCallback* progress)
{
    if (!_elevationLayer.valid())
    {
        OE_WARN << LC << "No elevation layer.\n";
        return 0L;
    }

    const GeoExtent& ex = key.getExtent();

    double inner = innerWidth().get() * 0.5;
    double outer = outerWidth().get() * 0.5;

    if (ex.getSRS()->isGeographic())
    {
        double latMid = fabs(ex.yMin() + ex.height()*0.5);
        double metersPerDegAtEquator = (ex.getSRS()->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double metersPerDegree = metersPerDegAtEquator * cos(osg::DegreesToRadians(latMid));
        inner = inner / metersPerDegree;
        outer = outer / metersPerDegree;
    }

    Bounds bounds;
    osg::ref_ptr<FeatureCursor> cursor;
    if (_featureSource.valid() && key.getLOD() == 13)
    {
        //OE_INFO << LC << "Query.\n";
        Query query;
        bounds = ex.bounds();
        bounds.expandBy(bounds.xMin() - outer, bounds.yMin() - outer, 0);
        bounds.expandBy(bounds.xMax() + outer, bounds.yMax() + outer, 0);
        bounds.transform(ex.getSRS(), _featureSource->getFeatureProfile()->getSRS());
        query.bounds() = bounds;
        //query.tileKey() = key;
        cursor = _featureSource->createFeatureCursor(query);
        if (cursor.valid() && !cursor->hasMore())
            cursor = 0L;
    }

    if (cursor.valid() && cursor->hasMore())
    {
        //OE_INFO << LC << "Cursor.\n";
        osg::ref_ptr<osg::HeightField> hf = HeightFieldUtils::createReferenceHeightField(
            ex,
            257, 257,           // base tile size for elevation data
            0u,                 // 1 sample border around the data makes it 259x259
            true);              // initialize to HAE (0.0) heights

        
        hf->getFloatArray()->assign(hf->getNumColumns()*hf->getNumRows(), NO_DATA_VALUE);

#ifdef USE_BASE_LAYER
        osg::ref_ptr<ElevationEnvelope> envelope = _pool.createEnvelope(ex.getSRS(), key.getLOD());
#else
        // Fetch the base elevation data, falling back if necessary        
        ElevationLayerVector elevationLayers;
        elevationLayers.push_back(_elevationLayer.get());
        bool realData = elevationLayers.populateHeightField(hf.get(), key, 0L, INTERP_BILINEAR, progress);
#endif

        int count = 0;
        while (cursor->hasMore())
        {
            Feature* feature = cursor->nextFeature();

            //if (feature->getString("mvt_layer") != "road")
            //    continue;

            // xform to the key's coordinate system
            // TODO: revisit this
            if (!key.getExtent().getSRS()->isHorizEquivalentTo(feature->getSRS()))
                feature->transform(key.getExtent().getSRS());

            ConstGeometryIterator geomIter(feature->getGeometry(), false);
            while (geomIter.hasMore())
            {
                const Geometry* geom = geomIter.next();
                if (integrate(key, hf, geom, inner, outer, envelope))
                    ++count;
            }
        }

        //if (debug) OE_INFO << "key = " << key.str() << ", segments = " << count << std::endl;

        return count > 0 ? hf.release() : 0L;
        //return realData ? hf.release() : 0L;
    }

    else
    {
        //OE_INFO << LC << "Calling EL::createHeightField for " << key.str() << std::endl;
        //return _elevationLayer->createHeightField(key, progress).takeHeightField();
        return 0L;
    }

    //return realData ? hf.release() : 0L;
}
