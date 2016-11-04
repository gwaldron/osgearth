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


namespace
{
    double inline smoothstep(double a, double b, double t)
    {
        // smoothsetp (approximates cosine):
        double mu = t*t*(3.0-2.0*t);
        return a*(1.0-mu) + b*mu;
    }

    bool integrate(const TileKey& key, osg::HeightField* hf, const Geometry* geom, double innerRadius, double outerRadius, ElevationEnvelope* envelope)
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

                            elevPROJ = smoothstep(elevA, elevB, bestT);
                        }

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




RoadHFTileSource::RoadHFTileSource(const RoadHFOptions& options) :
TileSource(options),
RoadHFOptions(options)
{
    setName("RoadHF");

    // Experiment with this and see what will work.
    _pool.setTileSize(65u); //33u); //257u); //129u); //33u);
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


osg::HeightField*
RoadHFTileSource::createHeightField(const TileKey& key, ProgressCallback* progress)
{
    if (!_elevationLayer.valid())
    {
        OE_WARN << LC << "No elevation layer.\n";
        return 0L;
    }

    const GeoExtent& ex = key.getExtent();

    // all points <= innerRadius from the centerline are flattened
    double innerRadius = innerWidth().get() * 0.5;

    // all points > innerRadius and <= outerRadius and smoothstep blended
    double outerRadius = outerWidth().get() * 0.5;

    // adjust those values based on latitude in a geographic map
    if (ex.getSRS()->isGeographic())
    {
        double latMid = fabs(ex.yMin() + ex.height()*0.5);
        double metersPerDegAtEquator = (ex.getSRS()->getEllipsoid()->getRadiusEquator() * 2.0 * osg::PI) / 360.0;
        double metersPerDegree = metersPerDegAtEquator * cos(osg::DegreesToRadians(latMid));
        innerRadius = innerRadius / metersPerDegree;
        outerRadius = outerRadius / metersPerDegree;
    }

    Bounds bounds;
    osg::ref_ptr<FeatureCursor> cursor;
    if (_featureSource.valid())
    {
        Query query;

        if (_featureSource->getFeatureProfile()->getTiled())
        {
            // A Tiled source should contain a natural buffer, that is, 
            // the features should extend some distance outside the tile
            // key extents. (At least outerWidth meters).
            query.tileKey() = key;
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

        // Initialize to NO DATA.
        hf->getFloatArray()->assign(hf->getNumColumns()*hf->getNumRows(), NO_DATA_VALUE);

        // Create an elevation query envelope at the LOD we are creating
        osg::ref_ptr<ElevationEnvelope> envelope = _pool.createEnvelope(ex.getSRS(), key.getLOD());

        MultiGeometry geoms;
        int count = 0;
        while (cursor->hasMore())
        {
            Feature* feature = cursor->nextFeature();

            // xform to the key's coordinate system
            if (!key.getExtent().getSRS()->isHorizEquivalentTo(feature->getSRS()))
                feature->transform(key.getExtent().getSRS());

            geoms.getComponents().push_back(feature->getGeometry());
        }

        if(integrate(key, hf, &geoms, innerRadius, outerRadius, envelope))
        {
            // If integrate made any changes, return the new heightfield
            return hf.release();
        }
    }

    return 0L;
}
