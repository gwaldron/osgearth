/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019-2020 Pelican Mapping
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

#include <osgEarth/PowerlineLayer>
#include <osgEarth/AltitudeFilter>
#include <osgEarth/JoinPointsLinesFilter>
#include <osgEarth/ElevationQuery>
#include <osgEarth/PolygonizeLines>
#include <osgEarth/ECEF>
#include <osgEarth/GeometryUtils>

#include <algorithm>
#include <iterator>

using namespace osgEarth;

#define LC "[PowerlineLayer]"

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(PowerlineModel, PowerlineLayer);

void PowerlineLayer::ModelOptions::fromConfig(const Config& conf)
{
    if (conf.hasChild("attachment_points"))
    {
        osg::ref_ptr<Geometry> attachGeom = GeometryUtils::geometryFromWKT(conf.child("attachment_points").value());
        std::copy(attachGeom->asVector().begin(), attachGeom->asVector().end(),
                  std::back_inserter(attachment_points()));
    }
    if (conf.hasChild("uri"))
    {
        uri() = conf.child("uri").value();
    }
}

Config PowerlineLayer::ModelOptions::getConfig() const
{
    Config conf;
    if (!attachment_points().empty())
    {
        osg::ref_ptr<Geometry> attachGeom = new LineString(&attachment_points());
        conf.set("attachment_points", GeometryUtils::geometryToWKT(attachGeom.get()));
    }
    conf.set("uri", uri());
    return conf;
}

PowerlineLayer::Options::Options()
    : FeatureModelLayer::Options()
{
    fromConfig(_conf);
}

PowerlineLayer::Options::Options(const ConfigOptions& options)
    : FeatureModelLayer::Options(options)
{
    fromConfig(_conf);
}

// XXX cropFeatures is required to be true in order to include line
// features using their extent instead of their centroid. What we
// really want is to include line features by extent without cropping
// them.
void PowerlineLayer::Options::fromConfig(const Config& conf)
{
    LayerReference<FeatureSource>::get(conf, "line_features", _lineSourceLayer, _lineSource);
    FeatureDisplayLayout layout = _layout.get();
    layout.cropFeatures() = true;
    _layout = layout;
    ConfigSet models = conf.children("tower_model");
    for (auto& modelConf : models)
    {
        towerModels().push_back(ModelOptions(modelConf));
    }
}

Config
PowerlineLayer::Options::getConfig() const
{
    Config conf = FeatureModelLayer::Options::getConfig();
    LayerReference<FeatureSource>::set(conf, "line_features", _lineSourceLayer, _lineSource);
    for (auto& modelOption : towerModels())
    {
        conf.add("tower_model", modelOption.getConfig());
    }
    return conf;
}

void PowerlineLayer::Options::mergeConfig(const Config& conf)
{
    FeatureModelLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

class PowerlineFeatureNodeFactory : public GeomFeatureNodeFactory
{
public:
    PowerlineFeatureNodeFactory(const PowerlineLayer::Options& options);
    bool createOrUpdateNode(FeatureCursor* cursor, const Style& style,
                            const FilterContext& context,
                            osg::ref_ptr<osg::Node>& node);
private:
    FeatureList makeCableFeatures(FeatureList& powerFeatures, FeatureList& towerFeatures,
                                  const FilterContext& cx);
    std::string _lineSourceLayer;
    FeatureSource::Options _lineSource;
    Vec3dVector _attachments;
    std::string _modelName;
};

PowerlineFeatureNodeFactory::PowerlineFeatureNodeFactory(const PowerlineLayer::Options& options)
    : GeomFeatureNodeFactory(options),
      _lineSourceLayer(options.lineSourceLayer().get()),
      _lineSource(options.lineSource().get())
{
    if (options.towerModels().empty())
        return;
    // Just use first model for now
    const PowerlineLayer::ModelOptions& modelOption = options.towerModels().front();
    std::copy(modelOption.attachment_points().begin(), modelOption.attachment_points().end(),
              std::back_inserter(_attachments));
    _modelName = modelOption.uri().get();
}

FeatureNodeFactory*
PowerlineLayer::createFeatureNodeFactoryImplementation() const
{
    return new PowerlineFeatureNodeFactory(options());
}

namespace
{
    Feature* getPointFeature(PointMap& pointMap, const osg::Vec3d& key)
    {
        auto itr = findPoint(pointMap, key);
        if (itr == pointMap.end())
        {
            return nullptr;
        }
        else
        {
            return itr->second.pointFeature.get();
        }
    }

    double calculateHeading(Geometry* geom, int index)
    {
        osg::Vec3d& point = (*geom)[index];
        osg::Vec3d in, out;
        if (index > 0)
        {
            in = point - (*geom)[index - 1];
            in.normalize();
        }
        if (index < geom->size()-1)
        {
            out = (*geom)[index + 1] - point;
            out.normalize();
        }
        osg::Vec3d direction = in + out;
        direction.normalize();
        double heading = std::atan2(-direction.x(), direction.y());
        if (heading < -osg::PI_2) heading += osg::PI;
        if (heading >= osg::PI_2) heading -= osg::PI;
        return osg::RadiansToDegrees(heading);
    }
}

FeatureList PowerlineFeatureNodeFactory::makeCableFeatures(FeatureList& powerFeatures,
                                                           FeatureList& towerFeatures, const FilterContext& cx)

{
    FeatureList result;
    const Session* session = cx.getSession();

    // the map against which we'll be doing elevation clamping
    osg::ref_ptr<const Map> map = session->getMap();
    if (!map.valid() || _attachments.empty())
        return result;

    const SpatialReference* mapSRS = map->getSRS();
    osg::ref_ptr<const SpatialReference> featureSRS = cx.profile()->getSRS();

    // establish an elevation query interface based on the features' SRS.
    ElevationQuery eq(map.get());

    PointMap pointMap;
    for (auto& feature : towerFeatures)
    {
        Geometry* geom = feature->getGeometry();
        for (osg::Vec3d& pt : *geom)
        {
            getPoint(pointMap, pt) = PointEntry(feature.get());
        }
    }

    const SpatialReference* targetSRS = nullptr;
    if (cx.getSession()->isMapGeocentric())
    {
        targetSRS = cx.getSession()->getMapSRS();
    }
    else
    {
        targetSRS = featureSRS->getGeocentricSRS();
    }
        
    for (auto& feature : powerFeatures)
    {
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_LINESTRING)
        {
            std::vector<float> elevations;
            eq.getElevations(geom->asVector(), feature->getSRS(), elevations);
            std::vector<osg::Vec3d> worldPts(geom->size());
            std::vector<osg::Matrixd> orientations(geom->size());
            for (int i = 0; i < geom->size(); ++i)
            {
                osg::Vec3d geodeticPt((*geom)[i].x(), (*geom)[i].y(), elevations[i]);
                ECEF::transformAndGetRotationMatrix(geodeticPt, featureSRS.get(), worldPts[i],
                                                    targetSRS, orientations[i]);
            }
            // New feature for the cable
            const int size = geom->size();
 
            for (int cable = 0; cable < 2; ++cable)
            {
                Feature* newFeature = new Feature(*feature);
                LineString* newGeom = new LineString(size);

                for (int i = 0; i < size; ++i)
                {
                    double heading = 0.0;
                    auto itr = findPoint(pointMap, (*geom)[i]);
                    if (itr != pointMap.end())
                    {
                        heading = itr->second.pointFeature->getDouble("heading", 0.0);
                    }
                    else
                    {
                        heading = calculateHeading(geom, i);
                    }
                    osg::Matrixd headingMat;
                    headingMat.makeRotate(osg::DegreesToRadians(heading), osg::Vec3d(0.0, 0.0, 1.0));
                    osg::Vec3d worldAttach = _attachments[cable] * headingMat * orientations[i] + worldPts[i];
                    osg::Vec3d wgs84; // intermediate point
                    osg::Vec3d mapAttach;
                    featureSRS->getGeographicSRS()->transformFromWorld(worldAttach, wgs84);
                    featureSRS->getGeographicSRS()->transform(wgs84, featureSRS.get(), mapAttach);
                    newGeom->push_back(mapAttach);
                }
                newFeature->setGeometry(newGeom);
                result.push_back(newFeature);
            }
        }
    }
    return result;
}

bool PowerlineFeatureNodeFactory::createOrUpdateNode(FeatureCursor* cursor, const Style& style,
                                                     const FilterContext& context,
                                                     osg::ref_ptr<osg::Node>& node)
{
    FilterContext sharedCX = context;
    FeatureList workingSet; 
    cursor->fill(workingSet);
    osgEarth::Util::JoinPointsLinesFilter pointsLinesFilter;
    pointsLinesFilter.lineSourceLayer() = "lines";
    pointsLinesFilter.lineSource() = _lineSource;
    FilterContext localCX = pointsLinesFilter.push(workingSet, sharedCX);
    // Render towers and lines (cables) seperately
    // Could write another filter for this?
    FeatureList pointSet;
    for (auto& feature : workingSet)
    {
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_POINTSET)
        {
            pointSet.push_back(feature);
        }
    }
    osg::ref_ptr<FeatureListCursor> listCursor = new FeatureListCursor(pointSet);
    osg::ref_ptr<osg::Node> pointsNode;
    GeomFeatureNodeFactory::createOrUpdateNode(listCursor.get(), style, localCX, pointsNode);
    osg::ref_ptr<osg::Group> results(new osg::Group);
    results->addChild(pointsNode.get());
    FeatureList cableFeatures =  makeCableFeatures(workingSet, pointSet, localCX);

    Style lineStyle;
    osg::ref_ptr<LineSymbol> lineSymbol = lineStyle.getOrCreateSymbol<LineSymbol>();

#if 0
    PolygonizeLinesFilter polyLineFilter(lineStyle);
    osg::Node* cables = polyLineFilter.push(cableFeatures, localCX);
    results->addChild(cables);
#endif
    GeometryCompiler compiler;
    osg::Node* cables = compiler.compile(cableFeatures, lineStyle, localCX);
    results->addChild(cables);

    node = results;
    return true;
}
