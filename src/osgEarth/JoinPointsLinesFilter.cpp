/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2019 Pelican Mapping
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
#include <osgEarth/JoinPointsLinesFilter>

#include <osgEarth/Filter>
#include <osgEarth/FeatureCursor>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/FeatureSource>
#include <osgEarth/FilterContext>
#include <osgEarth/Geometry>
#include <osgEarth/ECEF>
#include <map>
#include <algorithm>
#include <cmath>

#define LC "[JointPointsLines FeatureFilter] "

using namespace osgEarth;

/**
   Add features from line strings to point features that make up their
   geometry. One use is to join Open Street Map nodes with the ways
   that contain them, a relation that is lost by OGR.

   Optionally this filter calculates a "heading" attribute for each
   point, which is the rotation around Z of the "tangent" of the
   linestring at the point.
 */
namespace
{
    void transpose3x3(osg::Matrix& m)
    {
        std::swap(m(0,1), m(1,0));
        std::swap(m(0,2), m(2,0));
        std::swap(m(1,2), m(2,1));
    }

    double calculateGeometryHeading(const osg::Vec3d& point, const osg::Vec3d& previous, const osg::Vec3d& next,
                                    FilterContext& context)
    {
        const SpatialReference* targetSRS = 0L;
        if (context.getSession()->isMapGeocentric())
        {
            targetSRS = context.getSession()->getMapSRS();
        }
        else
        {
            targetSRS = context.profile()->getSRS()->getGeocentricSRS();
        }
        osg::Vec3d point3d(point);

        osg::Matrixd orientation;
        osg::Vec3d world3d;
    
        ECEF::transformAndGetRotationMatrix(point3d, context.profile()->getSRS(), world3d,
                                            targetSRS, orientation);
        // XXX OSG bug weirdness, fixed in OSG next
        osg::Matrixd toLocal(orientation);
        transpose3x3(toLocal);
        
        osg::Vec2d in;
        osg::Vec2d out;
        if (previous.x() != DBL_MAX)
        {
            osg::Vec3d prevWorld;
            ECEF::transformAndLocalize(previous, context.profile()->getSRS(), prevWorld,
                                       targetSRS);
            osg::Vec3d inWorld = world3d - prevWorld;
            osg::Vec3d inLocal = inWorld * toLocal;
            in.x() = inLocal.x();
            in.y() = inLocal.y();
            in.normalize();
        
        }
        if (next.x() != DBL_MAX)
        {
            osg::Vec3d nextWorld;
            ECEF::transformAndLocalize(next, context.profile()->getSRS(), nextWorld,
                                       targetSRS);
            osg::Vec3d outWorld = nextWorld - world3d;
            osg::Vec3d outLocal = outWorld * toLocal;
            out.x() = outLocal.x();
            out.y() = outLocal.y();
            out.normalize();
        }

        osg::Vec2d direction = in + out;
        double heading = std::atan2(-direction.x(), direction.y());
        if (heading < -osg::PI_2) heading += osg::PI;
        if (heading >= osg::PI_2) heading -= osg::PI;
        return osg::RadiansToDegrees(heading);
    }

    double calculateGeometryHeading(const osg::Vec3d& point, const osg::Vec3d& previous, const osg::Vec3d& next,
                                    const SpatialReference* sourceSRS,
                                    const SpatialReference* targetSRS)
    {
        osg::Vec3d point3d(point);

        osg::Matrixd orientation;
        osg::Vec3d world3d;

        ECEF::transformAndGetRotationMatrix(point3d, sourceSRS, world3d, targetSRS, orientation);
        // XXX OSG bug weirdness, fixed in OSG next
        osg::Matrixd toLocal(orientation);
        transpose3x3(toLocal);
        osg::Vec2d in;
        osg::Vec2d out;
        if (previous.x() != DBL_MAX)
        {
            osg::Vec3d prevWorld;
            ECEF::transformAndLocalize(previous, sourceSRS, prevWorld, targetSRS);
            osg::Vec3d inWorld = world3d - prevWorld;
            osg::Vec3d inLocal = inWorld * toLocal;
            in.x() = inLocal.x();
            in.y() = inLocal.y();
            in.normalize();
        }
        if (next.x() != DBL_MAX)
        {
            osg::Vec3d nextWorld;
            ECEF::transformAndLocalize(next, sourceSRS, nextWorld, targetSRS);
            osg::Vec3d outWorld = nextWorld - world3d;
            osg::Vec3d outLocal = outWorld * toLocal;
            out.x() = outLocal.x();
            out.y() = outLocal.y();
            out.normalize();
        }
        osg::Vec2d direction = in + out;
        double heading = std::atan2(-direction.x(), direction.y());
        if (heading < -osg::PI_2) heading += osg::PI;
        if (heading >= osg::PI_2) heading -= osg::PI;
        return osg::RadiansToDegrees(heading);
    }
}

Status JoinPointsLinesFilter::initialize(const osgDB::Options* readOptions)
{
    Status fsStatus = lineSource().open(readOptions);
    if (fsStatus.isError())
        return fsStatus;

    return Status::OK();
}

void JoinPointsLinesFilter::getLineFeatures(const GeoExtent& extent, FeatureList& features)
{
    FeatureSource* fs = lineSource().getLayer();
    if (!fs)
        return;

    //TODO: should this be Profile::transformAndClampExtent instead?
    GeoExtent localExtent = extent.transform( fs->getFeatureProfile()->getSRS() );
    Query query;
    query.bounds() = localExtent.bounds();
    if (localExtent.intersects( fs->getFeatureProfile()->getExtent()))
    {
        osg::ref_ptr< FeatureCursor > cursor = fs->createFeatureCursor(query);
        while (cursor->hasMore())
        {
            Feature* feature = cursor->nextFeature();
            if (feature->getGeometry()->getType() == Geometry::TYPE_LINESTRING)
            {
                features.push_back(feature);
            }
        }
    }     
}
FilterContext JoinPointsLinesFilter::push(FeatureList& input, FilterContext& context)
{
    PointMap pointMap;
        
    for(FeatureList::iterator i = input.begin(); i != input.end(); ++i)
    {
        Feature* feature = i->get();
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_POINT || geom->getType() == Geometry::TYPE_POINTSET)
        {
            // Are there multiple points? Does it matter?
            for(Geometry::iterator i = geom->begin(); i != geom->end(); ++i)
            {
                const osg::Vec3d& pt = *i;
                pointMap[pt] = PointEntry(feature);
            }
        }
    }
    for (FeatureList::iterator i = input.begin(); i != input.end(); ++i)
    {
        Feature* feature = i->get();
        Geometry* geom = feature->getGeometry();
        if (geom->getType() != Geometry::TYPE_LINESTRING)
            continue;
        const int size = geom->size();
        for (int i = 0; i < size; ++i)
        {
            osg::Vec3d key = (*geom)[i];
            PointMap::iterator ptItr = pointMap.find(key);
            if (ptItr == pointMap.end() && createPointFeatures().get())
            {
                std::pair<PointMap::iterator, bool> ret
                    = pointMap.insert(PointMap::value_type(key, PointEntry(0L)));
                ptItr = ret.first;
            }
            if (ptItr != pointMap.end())
            {
                PointEntry &point = ptItr->second;
                point.lineFeatures.push_back(feature);
                if (i > 0)
                {
                    point.previous = (*geom)[i - 1];
                }
                if (i < size - 1)
                {
                    point.next = (*geom)[i + 1];
                }
            }
        }
    }
    const SpatialReference* targetSRS = 0L;
    if (context.getSession()->isMapGeocentric())
    {
        targetSRS = context.getSession()->getMapSRS();
    }
    else
    {
        targetSRS = context.profile()->getSRS()->getGeocentricSRS();
    }
    for(PointMap::iterator i = pointMap.begin(); i != pointMap.end(); ++i)
    {        
        PointEntry& entry = i->second;
        Feature* pointFeature = entry.pointFeature.get();
        if (!pointFeature)
        {
            Point* geom = new Point;
            geom->set(i->first);
            pointFeature = new Feature(geom, entry.lineFeatures.front()->getSRS(), Style(), 0L);
            input.push_back(pointFeature);
        }
        for(FeatureList::iterator i = entry.lineFeatures.begin(); i != entry.lineFeatures.end(); ++i)
        {
            Feature* lineFeature = i->get();
            const AttributeTable& attrTable = lineFeature->getAttrs();
            for (auto& attr_entry : attrTable)
            {
                if (attr_entry.first[0] != '@' && !pointFeature->hasAttr(attr_entry.first))
                {
                    pointFeature->set(attr_entry.first, attr_entry.second);
                }
            }
        }
        pointFeature->set("heading", calculateGeometryHeading(i->first, entry.previous, entry.next,
                                                              context.profile()->getSRS(), targetSRS));
    }

    return context;
}
