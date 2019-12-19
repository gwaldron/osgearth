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

double calculateGeometryHeading(const osg::Vec2d& point, const osg::Vec3d& previous, const osg::Vec3d& next,
                                FilterContext& context)
{
    const SpatialReference* targetSRS = nullptr;
    if (context.getSession()->isMapGeocentric())
    {
        targetSRS = context.getSession()->getMapSRS();
    }
    else
    {
        targetSRS = context.profile()->getSRS()->getGeocentricSRS();
    }
    osg::Vec3d point3d(point, 0.0);

    osg::Matrixd orientation;
    osg::Vec3d world3d;
    
    ECEF::transformAndGetRotationMatrix(point3d, context.profile()->getSRS(), world3d,
                                        targetSRS, orientation);
    // XXX OSG bug weirdness
    osg::Matrixd toLocal(orientation);
    toLocal.transpose3x3(toLocal);
    
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
    return osg::RadiansToDegrees(heading);
}

Status JoinPointsLinesFilter::initialize(const osgDB::Options* readOptions)
{
    Status fsStatus = _lineSource.open(lineSource(), readOptions);
    if (fsStatus.isError())
        return fsStatus;

    return Status::OK();
}

void JoinPointsLinesFilter::getLineFeatures(const GeoExtent& extent, FeatureList& features)
{
    FeatureSource* fs = _lineSource.getLayer();
    if (!fs)
        return;

    //TODO: should this be Profile::transformAndClampExtent instead?
    GeoExtent localExtent = extent.transform( fs->getFeatureProfile()->getSRS() );
    Query query;
    query.bounds() = localExtent.bounds();
    if (localExtent.intersects( fs->getFeatureProfile()->getExtent()))
    {
        osg::ref_ptr< FeatureCursor > cursor = fs->createFeatureCursor( query, nullptr );
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
        
    for (auto& feature : input)
    {
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_POINTSET)
        {
            // Are there multiple points? Does it matter?
            for (osg::Vec3d& pt : *geom)
            {
                osg::Vec2d key(pt.x(), pt.y());
                pointMap[key] = PointEntry(feature);
            }
        }
    }
    for (auto& feature : input)
    {
        Geometry* geom = feature->getGeometry();
        if (geom->getType() != Geometry::TYPE_LINESTRING)
            continue;
        const int size = geom->size();
        for (int i = 0; i < size; ++i)
        {
            osg::Vec2d key((*geom)[i].x(), (*geom)[i].y());
            auto ptItr = pointMap.find(key);
            if (ptItr != pointMap.end())
            {
                PointEntry &point = ptItr->second;
                point.lineFeatures.push_back(feature);
                if (i > 0)
                {
                    point.previous = osg::Vec3d((*geom)[i - 1].x(), (*geom)[i - 1].y(), 0.0);
            
                }
                if (i < size - 1)
                {
                    point.next = osg::Vec3d((*geom)[i + 1].x(), (*geom)[i + 1].y(), 0.0);
                }
            }
        }
    }
    for (auto& kv : pointMap)
    {
        PointEntry& entry = kv.second;
        Feature* pointFeature = entry.pointFeature.get();
        for (auto& lineFeature : entry.lineFeatures)
        {
            const AttributeTable& attrTable = lineFeature->getAttrs();
            for (const auto& attrEntry : attrTable)
            {
                if (attrEntry.first[0] != '@' && !pointFeature->hasAttr(attrEntry.first))
                {
                    pointFeature->set(attrEntry.first, attrEntry.second);
                }
            }
        }
        pointFeature->set("heading", calculateGeometryHeading(kv.first, entry.previous, entry.next,
                                                              context));
    }

    return context;
}
