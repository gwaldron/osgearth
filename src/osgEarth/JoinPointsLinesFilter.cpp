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

osg::Vec2d quantize(const osg::Vec2d& v)
{
    osg::Vec2d out;
    out.x() = (int)v.x();
    out.y() = (int)v.y();
    return out;
}

double calculateGeometryHeading(const osg::Vec2d& point, const osg::Vec3d& previous, const osg::Vec3d& next,
                                FilterContext& context)
{
#if 1
    osg::Vec2d in, out;
    if (previous.x() != DBL_MAX)
    {
        in = point - osg::Vec2d(previous.x(), previous.y());
        in.normalize();
    }
    if (next.x() != DBL_MAX)
    {
        out = osg::Vec2d(next.x(), next.y()) - point;
        out.normalize();
    }
    osg::Vec2d direction = in + out;
    direction.normalize();
    double heading = std::atan2(-direction.x(), direction.y());
    while (heading < -osg::PI_2) heading += osg::PI;
    while (heading >= osg::PI_2) heading -= osg::PI;
    return osg::RadiansToDegrees(heading);

#else

    const SpatialReference* targetSRS = 0L;
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
    // XXX OSG bug weirdness, fixed in OSG next
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
    if (heading < -osg::PI_2) heading += osg::PI;
    if (heading >= osg::PI_2) heading -= osg::PI;
    return osg::RadiansToDegrees(heading);
#endif
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
        osg::ref_ptr< FeatureCursor > cursor = fs->createFeatureCursor( query, 0L);
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
                osg::Vec2d key = quantize(osg::Vec2d(pt.x(), pt.y()));
                pointMap[key] = PointEntry(feature);
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
            osg::Vec2d key = quantize(osg::Vec2d((*geom)[i].x(), (*geom)[i].y()));
            PointMap::iterator ptItr = pointMap.find(key);
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
    for(PointMap::iterator i = pointMap.begin(); i != pointMap.end(); ++i)
    {        
        PointEntry& entry = i->second;
        Feature* pointFeature = entry.pointFeature.get();
        for(FeatureList::iterator i = entry.lineFeatures.begin(); i != entry.lineFeatures.end(); ++i)
        {
            Feature* lineFeature = i->get();
            const AttributeTable& attrTable = lineFeature->getAttrs();
            for(AttributeTable::const_iterator j = attrTable.begin(); j != attrTable.end(); ++j)
            {
                const AttributeTable::value_type& attrEntry = *j;
                if (attrEntry.first[0] != '@' && !pointFeature->hasAttr(attrEntry.first))
                {
                    pointFeature->set(attrEntry.first, attrEntry.second);
                }
            }
        }
        pointFeature->set("heading", calculateGeometryHeading(i->first, entry.previous, entry.next,
                                                              context));
    }

    return context;
}
