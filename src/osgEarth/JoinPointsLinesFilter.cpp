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
            if (feature->getGeometry()->isLinear())
            {
                features.push_back(feature);
            }
        }
    }     
}

namespace
{
    bool eq2d(const osg::Vec3d& lhs, const osg::Vec3d& rhs)
    {
        return
            equivalent(lhs.x(), rhs.x(), 1e-3) &&
            equivalent(lhs.y(), rhs.y(), 1e-3);
    }    
}

FilterContext JoinPointsLinesFilter::push(FeatureList& input, FilterContext& context)
{
    PointMap pointMap;

    // collect all point features (towers and poles) that fall within the working extent.
    FeatureList points;
    for(auto& feature : input)
    {
        if (feature->getGeometry()->isPointSet())
        {
            GeometryIterator iter(feature->getGeometry(), false);
            iter.forEach([&](auto* geom)
                {
                    for(auto& pt : *geom)
                    {
                        pointMap[pt] = PointEntry(feature);
                    }
                });

            points.push_back(feature);
        }
    }

    // collect all linear features as single linestrings:
    FeatureList lines;
    for (auto& feature : input) {
        auto* g = feature->getGeometry();
        if (g && g->isLinear()) {
            GeometryIterator iter(g, false);
            iter.forEach([&](auto* geom)
                {
                    if (geom->size() >= 2) {
                        auto* new_feature = new Feature(*feature);
                        new_feature->setGeometry(geom->cloneAs(Geometry::TYPE_LINESTRING));
                        lines.emplace_back(new_feature);
                    }
                });
        }
    }

    // combine linestrings with common endpoints:
    for (int changes = 1; changes > 0; )
    {
        changes = 0;
        for (auto& feature : lines)
        {
            if (!feature.valid())
                continue;

            auto* geom = feature->getGeometry();
            auto& start = geom->front();
            auto& end = geom->back();

            for (auto& other : lines)
            {
                if (other.valid() && other != feature)
                {
                    auto* other_geom = other->getGeometry();

                    if (eq2d(start, other_geom->back()))
                    {
                        geom->erase(geom->begin());
                        geom->insert(geom->begin(), other_geom->begin(), other_geom->end());
                        changes++;
                        other = nullptr;
                    }
                    else if (eq2d(start, other_geom->front()))
                    {
                        geom->erase(geom->begin());
                        geom->insert(geom->begin(), other_geom->rbegin(), other_geom->rend());
                        changes++;
                        other = nullptr;
                    }
                    else if (eq2d(end, other_geom->front()))
                    {
                        geom->erase(geom->end() - 1);
                        geom->insert(geom->end(), other_geom->begin(), other_geom->end());
                        changes++;
                        other = nullptr;
                    }
                    else if (eq2d(end, other_geom->front()))
                    {
                        geom->erase(geom->end() - 1);
                        geom->insert(geom->end(), other_geom->rbegin(), other_geom->rend());
                        changes++;
                        other = nullptr;
                    }
                }
            }
        }
    }

    // remove the onces that were null'd out during connection:
    FeatureList temp;
    for(auto& feature : lines)
        if (feature.valid())
            temp.push_back(feature);
    lines.swap(temp);

    // associate all linears to their component points
    for(auto& feature : lines)
    {
        if (feature.valid() && feature->getGeometry()->isLinear())
        {   
            auto* output = new LineString();

            GeometryIterator iter(feature->getGeometry(), false);
            iter.forEach([&](Geometry* geom)
                {
                    const int size = geom->size();
                    for (int i = 0; i < size; ++i)
                    {
                        osg::Vec3d key = (*geom)[i];

                        PointMap::iterator ptItr = pointMap.find(key);

                        if (ptItr == pointMap.end() && createPointFeatures() == true)
                        {
                            auto ret = pointMap.emplace(key, PointEntry(nullptr));
                            ptItr = ret.first;
                        }

                        if (ptItr != pointMap.end())
                        {
                            PointEntry& point = ptItr->second;
                            point.lineFeatures.push_back(feature);
                            
                            if (!output->empty())
                            {
                                point.previous = output->back();
                                auto& prev_point = pointMap.find(point.previous);
                                prev_point->second.next = key;
                            }

                            output->push_back(key);
                        }
                    }
                });

            feature->setGeometry(output);
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

    for(auto& i : pointMap)
    {        
        PointEntry& entry = i.second;
        Feature* pointFeature = entry.pointFeature.get();
        if (!pointFeature)
        {
            Point* geom = new Point;
            geom->set(i.first);
            pointFeature = new Feature(geom, entry.lineFeatures.front()->getSRS(), Style(), 0L);
            input.push_back(pointFeature);
        }

        for(auto& lineFeature : entry.lineFeatures)
        {
            const AttributeTable& attrTable = lineFeature->getAttrs();
            for (auto& attr_entry : attrTable)
            {
                if (attr_entry.first[0] != '@' && !pointFeature->hasAttr(attr_entry.first))
                {
                    pointFeature->set(attr_entry.first, attr_entry.second);
                }
            }
        }
        pointFeature->set("heading",
            calculateGeometryHeading(
                i.first, entry.previous, entry.next,
                context.profile()->getSRS(), targetSRS));        
    }

    // combine all new features for output.
    input.clear();
    input.reserve(points.size() + lines.size());
    input.insert(input.end(), points.begin(), points.end());
    input.insert(input.end(), lines.begin(), lines.end());

    return context;
}
