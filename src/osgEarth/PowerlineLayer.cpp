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
#include <osgEarth/Network>

#include <algorithm>
#include <iterator>

using namespace osgEarth;

#define LC "[PowerlineLayer]"

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(PowerlineModel, PowerlineLayer);

// Render power lines with their towers as models.
// 
// This code was originally written to use OSM data served in a tiled
// data source. In that model, features are provided for the power
// towers and power lines, then the attributes of both (tower height,
// line voltage, numbers of lines) are useful, but there's not
// explicit link between the two kinds of features. On the other hand,
// power line data could come from an shp file or other GDAL layer in
// which only the line features are available. To handle this, we
// generate features for the towers if they aren't available.

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
    _point_features.init(true);

    conf.get("point_features", point_features());
    lineSource().get(conf, "line_features");
    FeatureDisplayLayout layout = _layout.get();
    layout.cropFeatures() = true;
    _layout = layout;
    ConfigSet models = conf.children("tower_model");
    for(ConfigSet::const_iterator i = models.begin(); i != models.end(); ++i)
    {
        towerModels().push_back(ModelOptions(*i));
    }
}

Config
PowerlineLayer::Options::getConfig() const
{
    Config conf = FeatureModelLayer::Options::getConfig();
    lineSource().set(conf, "line_features");
    for (std::vector<ModelOptions>::const_iterator i = towerModels().begin();
        i != towerModels().end();
        ++i)
    {
        conf.add("tower_model", i->getConfig());
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
    PowerlineFeatureNodeFactory(const PowerlineLayer::Options& options, StyleSheet* styles);

    bool createOrUpdateNode(FeatureCursor* cursor, const Style& style,
                            const FilterContext& context,
                            osg::ref_ptr<osg::Node>& node,
                            const Query& query);
private:
    FeatureList makeCableFeatures(FeatureList& powerFeatures, FeatureList& towerFeatures,
                                  const FilterContext& cx, const Query& query);
    std::string _lineSourceLayer;
    FeatureSource::Options _lineSource;
    Vec3dVector _attachments;
    std::string _modelName;
    osg::ref_ptr<StyleSheet> _styles;
    bool _point_features;
};

PowerlineFeatureNodeFactory::PowerlineFeatureNodeFactory(const PowerlineLayer::Options& options, StyleSheet* styles)
    : GeomFeatureNodeFactory(options),    
      _lineSourceLayer(options.lineSourceLayerName().get()),
      _lineSource(options.lineSourceEmbeddedOptions().get()),
      _styles(styles),
      _point_features(true)
{
    if (options.towerModels().empty())
        return;
    // Just use first model for now
    const PowerlineLayer::ModelOptions& modelOption = options.towerModels().front();
    std::copy(modelOption.attachment_points().begin(), modelOption.attachment_points().end(),
              std::back_inserter(_attachments));
    _modelName = modelOption.uri().get();
    _point_features = options.point_features().get();
}

FeatureNodeFactory*
PowerlineLayer::createFeatureNodeFactoryImplementation() const
{
    return new PowerlineFeatureNodeFactory(options(), getStyleSheet());
}

namespace
{
    // Network fun

    // Identify an edge by its feature and position in the geometry
    // list

    struct EdgeNode
    {
        EdgeNode()
            : id(0), position(0)
        {
        }
        EdgeNode(FeatureID id_, int position_)
            : id(id_), position(position_)
        {
        }
        FeatureID id;
        int position;
        bool operator<(const EdgeNode& rhs) const
        {
            return (id < rhs.id) || (id == rhs.id && position < rhs.position);
        }
    };
    
    typedef osg::Vec3d NetworkNode;

    typedef Network<EdgeNode, NetworkNode> PowerNetwork;

    void addFeature(PowerNetwork& network, const Feature* feature)
    {
            const Geometry* geom = feature->getGeometry();
            if (geom->getType() == Geometry::TYPE_LINESTRING)
            {
                FeatureID fid = feature->getFID();
                for (int seg = 0; seg < geom->size() - 1; ++seg)
                {
                    network.addEdge(EdgeNode(fid, seg), (*geom)[seg], (*geom)[seg - 1]);
                }
            }
    }

    void addFeatures(PowerNetwork& network, const FeatureList& list)
    {
        for (FeatureList::const_iterator i = list.begin(); i != list.end(); ++i)
        {
            addFeature(network, i->get());
        }        
    }
    
    Feature* getPointFeature(PointMap& pointMap, const osg::Vec3d& key)
    {
        PointMap::iterator itr = findPoint(pointMap, key);
        if (itr == pointMap.end())
        {
            return 0L;
        }
        else
        {
            return itr->second.pointFeature.get();
        }
    }

    double calculateHeading(osg::Vec3d& point, osg::Vec3d* previous, osg::Vec3d* next)
    {
        osg::Vec3d in, out;
        if (previous)
        {
            in = point - *previous;
            in.normalize();
        }
        if (next)
        {
            out = *next - point;
            out.normalize();
        }
        osg::Vec3d direction = in + out;
        direction.normalize();
        double heading = std::atan2(-direction.x(), direction.y());
        if (heading < -osg::PI_2) heading += osg::PI;
        if (heading >= osg::PI_2) heading -= osg::PI;
        return osg::RadiansToDegrees(heading);

    }

    double calculateHeading(Geometry* geom, int index)
    {
        osg::Vec3d& point = (*geom)[index];
        osg::Vec3d* previous = 0L;
        osg::Vec3d* next = 0L;
        if (index > 0)
        {
            previous = &(*geom)[index - 1];
        }
        if (index < geom->size() - 1)
        {
            next = &(*geom)[index + 1];
        }
        return calculateHeading(point, previous, next);
    }

    FeatureList findNeighborLineFeatures(const FilterContext& context, const Query& query)
    {
        const Session* session = context.getSession();
        FeatureList result;

        if (!query.tileKey().isSet())
            return result;
        for (int i =-1; i <= 1; ++i)
        {
            for (int j = -1; j <=1; ++j)
            {
                if (!(i == 0 && j == 0))
                {
                    TileKey neighborKey = query.tileKey().get().createNeighborKey(i, j);
                    Query newQuery(query);
                    newQuery.bounds().unset();
                    newQuery.tileKey() = neighborKey;
                    FeatureCursor* cursor = session->getFeatureSource()->createFeatureCursor(newQuery, 0L);
                    while (cursor->hasMore())
                    {
                        Feature* feature = cursor->nextFeature();
                        Geometry* geom = feature->getGeometry();
                        if (geom->getType() == Geometry::TYPE_LINESTRING)
                        {
                            result.push_back(feature);
                        }
                    }
                }
            }
        }
        return result;
    }
}

FeatureList PowerlineFeatureNodeFactory::makeCableFeatures(FeatureList& powerFeatures,
                                                           FeatureList& towerFeatures, const FilterContext& cx,
                                                           const Query& query)

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

    PowerNetwork linesNetwork;
    addFeatures(linesNetwork, powerFeatures);
    FeatureList neighbors = findNeighborLineFeatures(cx, query);
    addFeatures(linesNetwork, neighbors);
    linesNetwork.buildNetwork();
    // Old code
    PointMap pointMap;
    for (FeatureList::iterator i = towerFeatures.begin(); i != towerFeatures.end(); ++i)
    {
        Feature* feature = i->get();
        Geometry* geom = feature->getGeometry();
        for(Geometry::iterator i = geom->begin(); i != geom->end(); ++i)
        {
            const osg::Vec3d& pt = *i;
            getPoint(pointMap, pt) = PointEntry(feature);
        }
    }

    const SpatialReference* targetSRS = 0L;
    if (cx.getSession()->isMapGeocentric())
    {
        targetSRS = cx.getSession()->getMapSRS();
    }
    else
    {
        targetSRS = featureSRS->getGeocentricSRS();
    }

    for (FeatureList::iterator i = powerFeatures.begin(); i != powerFeatures.end(); ++i)
    {
        Feature* feature = i->get();
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
                    PointMap::iterator itr = findPoint(pointMap, (*geom)[i]);
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
                                                     osg::ref_ptr<osg::Node>& node,
                                                     const Query& query)
{
    FilterContext sharedCX = context;
    FeatureList workingSet; 
    cursor->fill(workingSet);

    Style towerStyle = style;
    if (_styles->getStyle("towers"))
        towerStyle = *_styles->getStyle("towers");
    Style cableStyle;
    if (_styles->getStyle("cables", false))
        cableStyle = *_styles->getStyle("cables", false);
    else
    {
        // defaults
        osg::ref_ptr<LineSymbol> lineSymbol = cableStyle.getOrCreateSymbol<LineSymbol>();
        lineSymbol->stroke()->color() = Color("#6f6f6f");
        lineSymbol->stroke()->width() = 1.5f;
        lineSymbol->tessellationSize() = Distance(0.25, Units::KILOMETERS);
        lineSymbol->useGLLines() = true;
    }
    // Render towers and lines (cables) seperately
    // Features for the tower models. This normally comes from feature
    // data in a layer, but it can be synthesized using only the line
    // features.
    FeatureList pointSet;
    FilterContext localCX = sharedCX;
    
    osgEarth::Util::JoinPointsLinesFilter pointsLinesFilter;
    pointsLinesFilter.lineSourceLayerName() = "lines";
    pointsLinesFilter.lineSourceEmbeddedOptions() = _lineSource;
    if (!_point_features)
    {
        pointsLinesFilter.createPointFeatures() = true;
    }
    localCX = pointsLinesFilter.push(workingSet, sharedCX);
    for(FeatureList::iterator i = workingSet.begin(); i != workingSet.end(); ++i)
    {
        Feature* feature = i->get();
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_POINTSET
            || geom->getType() == Geometry::TYPE_POINT)
        {
            pointSet.push_back(feature);
        }
    }

    osg::ref_ptr<FeatureListCursor> listCursor = new FeatureListCursor(pointSet);
    osg::ref_ptr<osg::Node> pointsNode;
    GeomFeatureNodeFactory::createOrUpdateNode(listCursor.get(), towerStyle, localCX, pointsNode, query);
    osg::ref_ptr<osg::Group> results(new osg::Group);
    results->addChild(pointsNode.get());
    FeatureList cableFeatures =  makeCableFeatures(workingSet, pointSet, localCX, query);
    GeometryCompiler compiler;
    osg::Node* cables = compiler.compile(cableFeatures, cableStyle, localCX);
    results->addChild(cables);
    node = results;
    return true;
}
