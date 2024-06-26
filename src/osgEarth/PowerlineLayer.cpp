/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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

#include <osgEarth/Array>
#include <osgEarth/PowerlineLayer>
#include <osgEarth/AltitudeFilter>
#include <osgEarth/JoinPointsLinesFilter>
#include <osgEarth/ElevationQuery>
#include <osgEarth/PolygonizeLines>
#include <osgEarth/ECEF>
#include <osgEarth/GeometryUtils>
#include <osgEarth/Network>
#include <osgEarth/Math>

#include <algorithm>
#include <iterator>
#include <sstream>

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
    if (conf.hasChild("name"))
    {
        name() = conf.child("name").value();
    }
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
    else if (conf.hasChild("model"))
    {
        uri() = conf.child("model").value();
    }
    conf.get("max_sag", maxSag());
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
    conf.set("max_sag", maxSag());
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
    _point_features.init(false);

    conf.get("point_features", point_features());
    lineSource().get(conf, "line_features");
    FeatureDisplayLayout layout = _layout.get();
    layout.cropFeatures() = true;
    _layout = layout;
    if (conf.hasChild("line_expr"))
    {
        lineExpr() = conf.child("line_expr").value();
    }
    if (conf.hasChild("cable_expr"))
    {
        cableExpr() = conf.child("cable_expr").value();
    }
    ConfigSet models = conf.children("tower_model");
    for(ConfigSet::const_iterator i = models.begin(); i != models.end(); ++i)
    {
        towerModels().push_back(ModelOptions(*i));
    }
    referrer = conf.referrer();
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

struct PowerlineRenderData
{
    PowerlineRenderData(float maxSag = 5.0)
        : _maxSag(maxSag)
    {}
    Vec3dVector _attachments;
    std::string _modelName;
    float _maxSag;
};

void parsePowerlineRenderData(PowerlineRenderData& renderData, const std::string& renderDataString)
{
    std::stringstream renderDataStream(renderDataString);
    Config renderDataConfig;
    renderDataConfig.fromXML(renderDataStream);
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
                                  FilterContext& cx, const Query& query, const Style& style);
    PowerlineLayer::ModelOptions evalTowerModel(Feature* f, const FilterContext& context);
    std::string _lineSourceLayer;
    FeatureSource::Options _lineSource;
    bool _point_features;
    optional<StringExpression> _lineExpr;
    optional<StringExpression> _cableExpr; 
    std::vector<PowerlineLayer::ModelOptions> _renderData;
    PowerlineLayer::Options _powerlineOptions;
};

PowerlineFeatureNodeFactory::PowerlineFeatureNodeFactory(const PowerlineLayer::Options& options, StyleSheet* styles)
    : GeomFeatureNodeFactory(options),    
      _lineSourceLayer(options.lineSource().externalLayerName().get()),
      _point_features(options.point_features().get()),
      _powerlineOptions(options)
{
    if (options.lineSource().embeddedOptions() != nullptr)
    {
        _lineSource = *options.lineSource().embeddedOptions();
    }

    if (options.towerModels().empty())
        return;
    _renderData = options.towerModels();
    if (options.lineExpr().isSet())
    {
        _lineExpr = options.lineExpr().get();
    }
    if (options.cableExpr().isSet())
    {
        _cableExpr = options.cableExpr().get();
    }
        
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
                    network.addEdge(EdgeNode(fid, seg), (*geom)[seg], (*geom)[seg + 1]);
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
                    FeatureCursor* cursor = session->getFeatureSource()->createFeatureCursor(newQuery);
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

// Make a catenary curve between two points
//
// A catenary is of the form y = a * cosh((x + k) / a) + c. We want to
// find the curve that passes between two points of different
// heights. This curve is determined by several parameters which may
// be hard to know, such as the maximum tension in the cable and its
// weight. Instead, we assume that the cable is a certain factor
// longer (slack )than the straight-line distance between the two
// attachment points.
//
// Arguments are in ECEF, orientation is at p1. We use that frame for
// the whole catenary.

namespace
{
    void toRow(osg::Matrixd& mat, osg::Vec3d& vec, int row)
    {
        for (int i = 0; i < 3; ++i)
        {
            mat(row, i) = vec[i];
        }
    }

    // Function for a in terms of arc length L, height difference h,
    // and horizontal distance d. This will be solved by a
    // Newton-Raphson solver, so we need both the function and its
    // derivative.

    struct GFunc
    {
        GFunc(double d_, double L, double h)
            : d(d_), rootLh(sqrt(L * L - h * h))
        {}

        double operator()(double a) const
        {
            return 2.0 * a * sinh(d / (2.0 * a)) - rootLh;
        }
        double d;
        double rootLh;
    };

    struct GFuncDeriv
    {
        GFuncDeriv(double d_)
            : d(d_)
        {
        }
        double operator()(double a) const
        {
            double da = d / a;
            return 2.0 * sinh(da/2.0) - da * cosh(da/2.0);
        }
        double d;
    };

    // Support around evaluating the catenary formula x = a * cosh((x + x1) / a) + C
    // and solving for the parameters a,x1, and C given the arc length of the curve L and the
    // horizontal and vertical displacement of the endpoints.
    struct CatenaryFunc
    {
        double a;
        double x1;
        double C;
        double L;
        double d;
        double h;
        CatenaryFunc(double a_, double x1_, double C_, double L_ = 0.0, double d_ = 0.0, double h_ = 0.0)
            : a(a_), x1(x1_), C(C_), L(L_), d(d_), h(h_)
        {
        }
        double operator()(double x) const
        {
            return a * cosh((x + x1) /a) + C;
        }
        static CatenaryFunc solveIt(double L, double d, double h)
        {
            GFunc gfunc(d, L, h);
            GFuncDeriv gfuncDeriv(d);
            bool validSoln = false;
            const double a = solve(gfunc, gfuncDeriv, d / 2, 1.0e-6, validSoln);
            // Parameters of the curve
            const double x1 = (a * log((L + h) / (L - h)) - d) / 2.0;
            const double C = -a * cosh(x1 / a);
            return CatenaryFunc(a, x1, C, L, d, h);
        }
    };
    // Find the difference between a catenary's minimum and refHeight. minHeight should be negative
    // i.e., below both towers in this usage.  d, h, and L are as in CatenaryFunc, but now L is a
    // free variable.
    struct MinCatHeight
    {
        double d;
        double h;
        double refHeight;
        MinCatHeight(double d_, double h_, double refHeight_ = 0.0)
            : d(d_), h(h_), refHeight(refHeight_)
        {
        }
        double operator()(double L) const
        {
            if (L * L < d * d + h * h)
            {
                // not happening
                return 0.0;
            }
            CatenaryFunc func = CatenaryFunc::solveIt(L, d, h);
            double minHeight = 0.0;
            if (func.x1 > 0.0)
            {
                // minimum is out of the range of the curve (x < 0)
                return 0.0;
            }
            // Could use func.a + func.C
            return (func(-func.x1) -refHeight);
        }
    };
}


void makeCatenary(osg::Vec3d p1, osg::Vec3d p2, const osg::Matrixd& orientation, double slack,
                  double maxSag,
                  std::vector<osg::Vec3d>& result, float tessellationSize)
{
    // Create a frame centered at p1 with orientation normal to
    // earth's surface
    osg::Matrixd FrameP1(orientation);
    toRow(FrameP1, p1, 3);
    // p1 in this frame is at origin. Get p2 into the local frame
    osg::Matrixd FrameP1Inv;
    FrameP1Inv.invert(FrameP1);
    osg::Vec3d p2local = p2 * FrameP1Inv;
    // The math for solving the catenary assumes that the higher point
    // is to the right on the X axis. Build a frame that reflects
    // that.
    //
    // If it turns out that p1 is higher, then at the end we will need
    // to generate points in reverse so that the resulting feature
    // makes sense.
    bool swapped = false;
    osg::Vec3d Xaxis;
    if (p2local.z() < 0.0)
    {
        swapped = true;
        Xaxis = osg::Vec3d(-p2local.x(), -p2local.y(), 0.0);
    }
    else
    {
        Xaxis = osg::Vec3d(p2local.x(), p2local.y(), 0.0);
    }
    // Horizontal distance between points
    const double d = Xaxis.normalize();
    osg::Vec3d Yaxis = osg::Vec3d(0.0, 0.0, 1.0) ^ Xaxis;
    osg::Matrixd FrameCat;
    toRow(FrameCat, Xaxis, 0);
    toRow(FrameCat, Yaxis, 1);
    // Height difference between points; should be positive
    double h = 0.0;
    if (swapped)
    {
        h = -p2local.z();
        toRow(FrameCat, p2local, 3);
    }
    else
    {
        h = p2local.z();
    }
    // straight distance between attachment points; obviously a lower limit on the cable length
    double straightDist = p2local.length();
    CatenaryFunc func = CatenaryFunc::solveIt(straightDist * slack, d, h);

    double xMin = -func.x1;
    double yMin = func(xMin);
    if (0.0 < xMin && yMin < -maxSag)
    {
        // Cable droops too much, so try a more expensive strategy
        MinCatHeight minimum(d, h, -maxSag);
        // No downside in choosing a very low lower bound
        double newGuess = ((slack - 1.0) * .01 + 1.0) * straightDist;
        // centimeter tolerence is fine
        double Lmin = solveBisect(minimum, newGuess, straightDist * slack, 0.01, 8);
        func = CatenaryFunc::solveIt(Lmin, d, h);
    }
    const osg::Vec3d P1(0.0, 0.0, 0.0), P2(d, 0.0, h);
    double begin, inc;
    int numSteps = ceil(p2local.length() / tessellationSize);
    std::vector<osg::Vec3d> cablePts;
    if (swapped)
    {
        inc = -d / numSteps;
        begin = d + inc;
        cablePts.push_back(P2);
    }
    else
    {
        inc = d / numSteps;
        begin = inc;
        cablePts.push_back(P1);
    }
    double x = begin;
    for (int i = 1; i <= numSteps; ++i, x += inc)
    {
        double z = func(x);
        cablePts.push_back(osg::Vec3d(x, 0.0, z));
    }
    osg::Matrixd cat2world = FrameCat * FrameP1;
    for (std::vector<osg::Vec3d>::iterator itr = cablePts.begin(), end = cablePts.end();
         itr != end;
         ++itr)
    {
        *itr = *itr * cat2world;
    }
    result.insert(result.end(), cablePts.begin(), cablePts.end());
}

int chooseAttachment(const osg::Matrixd& towerFrame0, const osg::Matrixd& towerFrame1,
                     const osg::Vec3d& attachStart, const osg::Vec3d& attachEnd0, const osg::Vec3d& attachEnd1)
{
    // Get everything into tower1's local frame
    osg::Matrixd world2Tower1 = osg::Matrixd::inverse(towerFrame0);
    osg::Vec3d t2 = osg::Vec3d(towerFrame1(3,0), towerFrame1(3, 1), towerFrame1(3, 2))
        * world2Tower1;
    osg::Vec3d end0 = attachEnd0 * towerFrame1 * world2Tower1;
    osg::Vec3d end1 = attachEnd1 * towerFrame1 * world2Tower1;
    // Project the line segments into the XY plane.
    // Segment 2D does just that.
    Segment2d towers(osg::Vec3d(0.0, 0.0, 0.0), t2);
    Segment2d line0(attachStart, end0);
    Segment2d line1(attachStart, end1);
    osg::Vec2d intersection;
    // the line that doesn't intersect the towers' centerline is the
    // right choice.
    if (!towers.intersect(line0, intersection))
    {
        return 0;
    }
    else if (!towers.intersect(line1, intersection))
    {
        return 1;
    }
    else
    {
        // ??? They both suck.
        return 0;
    }
}

osg::Matrixd getLocalToWorld(const osg::Vec3d& geodeticPt,
                             const SpatialReference* inputSRS,
                             const SpatialReference* outputSRS)
{
    osg::Matrixd result;
    osg::Vec3d worldPt;
    ECEF::transformAndGetRotationMatrix(geodeticPt, inputSRS, worldPt,
                                        outputSRS, result);
    result.setTrans(worldPt);
    return result;
}

namespace
{
    bool evalStyle(Feature* f, FilterContext& context, const StringExpression& styleExpr, Style& combinedStyle)
    {
        StyleSheet* sheet = context.getSession()->styles();
        // See if multiple selectors become necessary
        const StyleSelector& sel = sheet->getSelectors().begin()->second;
        StringExpression styleExprCopy(styleExpr);
        const std::string& styleString = f->eval(styleExprCopy, &context);
        if (!styleString.empty() && styleString != "null")
        {
            // resolve the style:

            // if the style string begins with an open bracket, it's an inline style definition.
            if (styleString.length() > 0 && styleString[0] == '{')
            {
                Config conf("style", styleString);
                conf.setReferrer(sel. styleExpression().get().uriContext().referrer());
                conf.set("type", "text/css");
                combinedStyle = Style(conf);
            }

            // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
            // style in this case: for style expressions, the user must be explicity about
            // default styling; this is because there is no other way to exclude unwanted
            // features.
            else
            {
                const Style* selectedStyle = context.getSession()->styles()->getStyle(styleString, false);
                if (selectedStyle)
                    combinedStyle = *selectedStyle;
            }
            return true;
        }
        return false;
    }

    void setCableStyleDefaults(Style& cableStyle)
    {
        osg::ref_ptr<LineSymbol> lineSymbol = cableStyle.getOrCreateSymbol<LineSymbol>();
        if (!lineSymbol->stroke()->width().isSet())
        {
            lineSymbol->stroke().mutable_value().width() = .05;
            lineSymbol->stroke().mutable_value().widthUnits() = Units::METERS;
        }
        if (!lineSymbol->tessellationSize().isSet())
        {
            lineSymbol->tessellationSize() = Distance(20, Units::METERS);
        }
        if (!lineSymbol->useWireLines().isSet())
        {
            lineSymbol->useWireLines() = true;
        }
    }

    void setModelStyleDefaults(Style& modelStyle, bool force = true)
    {
        osg::ref_ptr<ModelSymbol> modelSymbol = modelStyle.getOrCreate<ModelSymbol>();
        osg::ref_ptr<AltitudeSymbol> altitudeSymbol = modelStyle.getOrCreate<AltitudeSymbol>();
        if (!modelSymbol->orientationFromFeature().isSet() || force)
        {
            modelSymbol->orientationFromFeature() = true;
        }
        if (!altitudeSymbol->clamping().isSet() || force)
        {
            altitudeSymbol->clamping()  = AltitudeSymbol::CLAMP_TO_TERRAIN;
        }
    }

    void setModelStyleDefaults(Style& modelStyle, const std::string& modelName, const std::string& referrer, bool force = true)
    {
        setModelStyleDefaults(modelStyle, force);
        osg::ref_ptr<ModelSymbol> modelSymbol = modelStyle.getOrCreate<ModelSymbol>();
        if (!modelSymbol->url().isSet() || force)
        {
            modelSymbol->url() = "\"" + modelName + "\"";
            modelSymbol->url().mutable_value().setURIContext(referrer);
        }
    }
}

PowerlineLayer::ModelOptions PowerlineFeatureNodeFactory::evalTowerModel(Feature *f, const FilterContext& cx)
{
    if (_powerlineOptions.lineExpr().isSet())
    {
        StringExpression lineExprCopy(_powerlineOptions.lineExpr().get());
        std::string renderDataString = f->eval(lineExprCopy, &cx);
        if (renderDataString[0] == '<')
        {
            PowerlineLayer::ModelOptions featureRenderData;
            std::stringstream renderDataStream(renderDataString);
            Config renderDataConfig;
            renderDataConfig.fromXML(renderDataStream);
            ConfigSet models = renderDataConfig.children("tower_model");
            featureRenderData.fromConfig(models.front());
            return featureRenderData;
        }
        else
        {
            // It's the name of a model
            for (auto& towerModel : _powerlineOptions.towerModels())
            {
                if (towerModel.name().isSet() && towerModel.name() == renderDataString)
                {
                    return towerModel;
                }
            }
            // No model? Shouldn't happen
            return PowerlineLayer::ModelOptions();

        }
    }
    else
    {
        return _powerlineOptions.towerModels().front();
    }
}

FeatureList PowerlineFeatureNodeFactory::makeCableFeatures(FeatureList& powerFeatures,
                                                           FeatureList& towerFeatures, FilterContext& cx,
                                                           const Query& query,
                                                           const Style& cableStyle)

{
    FeatureList result;
    const Session* session = cx.getSession();

    // the map against which we'll be doing elevation clamping
    osg::ref_ptr<const Map> map = session->getMap();
    if (!map.valid() || (_renderData[0].attachment_points().empty() && !_lineExpr.isSet()))
        return result;

    const SpatialReference* mapSRS = map->getSRS();
    osg::ref_ptr<const SpatialReference> featureSRS = cx.profile()->getSRS();

    // establish an elevation query interface based on the features'
    // SRS. XXX This should be based on the style sheet option

    ElevationQuery eq(map.get());

    // Network stuff not really working yet.
    PowerNetwork linesNetwork;
    addFeatures(linesNetwork, powerFeatures);
    if (false)
    {
        FeatureList neighbors = findNeighborLineFeatures(cx, query);
        addFeatures(linesNetwork, neighbors);
        linesNetwork.buildNetwork();
    }
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

    StringExpression lineExprCopy;
    StringExpression cableExprCopy;

    if (_cableExpr.isSet())
    {
        cableExprCopy = _cableExpr.get();
    }

    for (FeatureList::iterator i = powerFeatures.begin(); i != powerFeatures.end(); ++i)
    {
        Feature* feature = i->get();
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_LINESTRING)
        {
            const int size = geom->size();
            std::vector<osg::Matrixd> towerMats;
            towerMats.reserve(size);
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
                    OE_NOTICE << LC << "tower not found!\n";
                    break;
                }
                const osg::Vec3d Z(0.0, 0.0, 1.0);
                osg::Matrixd headingMat = osg::Matrixd::rotate(osg::DegreesToRadians(heading), Z);
                osg::Matrixd geodMat = getLocalToWorld(itr->first, featureSRS.get(), targetSRS);
                towerMats.push_back(headingMat * geodMat);
            }
            PowerlineLayer::ModelOptions featureRenderData = evalTowerModel(feature, cx);
            Style localStyle;
            if (_cableExpr.isSet())
            {
                evalStyle(feature, cx, _cableExpr.get(), localStyle);
                setCableStyleDefaults(localStyle);
            }
            const Style& styleRef = _cableExpr.isSet() ? localStyle : cableStyle;
            
            // For various reasons the headings of successive towers can be inconsistant, causing
            // the cables between attachment points to cross each other. Ideally, the points are
            // specified in pairs. If the attachment point being used causes a cable to cross over
            // the center line between towers, then switch to the other attachment point. If the
            // attachment points are not in pairs... awkward...
            Array::View<osg::Vec3d> attachments(featureRenderData.attachment_points().data(),
                                                featureRenderData.attachment_points().size() / 2, 2);
            for (int attachRow = 0; attachRow < featureRenderData.attachment_points().size() / 2; ++attachRow)
            {
                for (int startingAttachment = 0; startingAttachment < 2; ++startingAttachment)
                {
                    // New feature for each cable
                    Feature* newFeature = new Feature(*feature);
                    LineString* newGeom = new LineString(size);
                    int currAttachment = startingAttachment;
                    std::vector<osg::Vec3d> cablePoints;
                    cablePoints.push_back(attachments(attachRow, currAttachment) * towerMats[0]);
                    for (int i = 1; i < size; ++i)
                    {
                        int next = chooseAttachment(towerMats[i - 1], towerMats[i],
                                                    attachments(attachRow, currAttachment),
                                                    attachments(attachRow, 0), attachments(attachRow, 1));
                        osg::Vec3d worldAttach = attachments(attachRow, next) * towerMats[i];
                        cablePoints.push_back(worldAttach);
                        currAttachment = next;
                    }
                    const bool catenary = true;
                    std::vector<osg::Vec3d> *cableSource = 0L;
                    std::vector<osg::Vec3d> catenaryPoints;
                    if (catenary)
                    {
                        for (int i = 0; i < cablePoints.size() -1; ++i)
                        {
                            makeCatenary(cablePoints[i], cablePoints[i + 1], towerMats[i], 1.002, featureRenderData.maxSag().get(),
                                         catenaryPoints,
                                         styleRef.get<LineSymbol>()->tessellationSize()->as(Units::METERS));
                        }
                        cableSource = &catenaryPoints;
                    }
                    else
                    {
                        cableSource = &cablePoints;
                    }
                    for (std::vector<osg::Vec3d>::iterator itr = cableSource->begin();
                         itr != cableSource->end();
                         ++itr)
                    {
                        osg::Vec3d wgs84, mapAttach;
                        featureSRS->getGeographicSRS()->transformFromWorld(*itr, wgs84);
                        featureSRS->getGeographicSRS()->transform(wgs84, featureSRS.get(), mapAttach);
                        newGeom->push_back(mapAttach);
                    }
                    newFeature->setGeometry(newGeom);
                    result.push_back(newFeature);
                }
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

    Style cableStyle;
    if (!_cableExpr.isSet())
    {
        
        if (const Style* sessionCableStyle = context.getSession()->styles()->getStyle("cables"))
            cableStyle = *sessionCableStyle;
        else
        {
            // defaults
            osg::ref_ptr<LineSymbol> lineSymbol = cableStyle.getOrCreateSymbol<LineSymbol>();
            lineSymbol->stroke().mutable_value().color() = Color("#6f6f6f");
            lineSymbol->stroke().mutable_value().width() = 1.5f;
            lineSymbol->useGLLines() = true;
        }
        osg::ref_ptr<LineSymbol> lineSymbol = cableStyle.getOrCreateSymbol<LineSymbol>();
        if (!lineSymbol->stroke()->width().isSet())
        {
            lineSymbol->stroke().mutable_value().width() = .05;
            lineSymbol->stroke().mutable_value().widthUnits() = Units::METERS;
        }
        if (!lineSymbol->tessellationSize().isSet())
        {
            lineSymbol->tessellationSize() = Distance(20, Units::METERS);
        }
        if (!lineSymbol->useWireLines().isSet())
        {
            lineSymbol->useWireLines() = true;
        }
    }

    // Render towers and lines (cables) seperately
    // Features for the tower models. This normally comes from feature
    // data in a layer, but it can be synthesized using only the line
    // features.
    FeatureList pointSet;
    FilterContext localCX = sharedCX;
    
    osgEarth::Util::JoinPointsLinesFilter pointsLinesFilter;
    pointsLinesFilter.lineSource().setExternalLayerName("lines");
    pointsLinesFilter.lineSource().setEmbeddedOptions(_lineSource);
    if (!_point_features)
    {
        pointsLinesFilter.createPointFeatures() = true;
    }
    localCX = pointsLinesFilter.push(workingSet, sharedCX);
    Style towerStyle;
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
    osg::ref_ptr<osg::Node> pointsNode;
    osg::ref_ptr<FeatureListCursor> listCursor = new FeatureListCursor(pointSet);
    StyleSheet* sheet = context.getSession()->styles();
    // The tower model should come from the "model" attribute of
    // tower_model if it is not specified in the tower model style
    // (usually called "towers"). If the tower_model is chosen
    // dynamically, the model set there still needs to be the default
    // in the model style.
    auto& selectors = sheet->getSelectors();
    bool useSelectorExp = !selectors.empty() && selectors.begin()->second.styleExpression().isSet();
    Style combinedStyle;
    // Create the graph for the tower models.
    if (useSelectorExp || _powerlineOptions.lineExpr().isSet())
    {
        // The style is different for each feature, either explicitly
        // due to the style selector, or implicitly from a tower_model
        // expression that selects different models.
        osg::Group* towersNode = new osg::Group;
        pointsNode = towersNode;
        while (listCursor->hasMore())
        {
            osg::ref_ptr<osg::Node> pointsNode;
            Feature* f = listCursor->nextFeature();

            if (useSelectorExp)
            {
                const StyleSelector& sel = selectors.begin()->second;
                StringExpression styleExprCopy(sel.styleExpression().get());
                const std::string& styleString = f->eval(styleExprCopy, &context);
                if (!styleString.empty() && styleString != "null")
                {
                    // resolve the style:
                    // if the style string begins with an open bracket, it's an inline style definition.
                    if (styleString.length() > 0 && styleString[0] == '{')
                    {
                        Config conf("style", styleString);
                        conf.setReferrer(sel.styleExpression().get().uriContext().referrer());
                        conf.set("type", "text/css");
                        combinedStyle = Style(conf);
                    }

                    // otherwise, look up the style in the stylesheet. Do NOT fall back on a default
                    // style in this case: for style expressions, the user must be explicity about
                    // default styling; this is because there is no other way to exclude unwanted
                    // features.
                    else
                    {
                        const Style* selectedStyle = context.getSession()->styles()->getStyle(styleString, false);
                        if (selectedStyle)
                            combinedStyle = *selectedStyle;
                    }
                }
                setModelStyleDefaults(combinedStyle, false);
            }
            else
            {
                PowerlineLayer::ModelOptions modelOptions = evalTowerModel(f, context);
                if (modelOptions.uri().isSet())
                {
                    setModelStyleDefaults(combinedStyle, modelOptions.uri().get(), _powerlineOptions.referrer, true);
                }
            }

            if (!combinedStyle.empty())
            {
                osg::ref_ptr<osg::Node> towerNode;
                osg::ref_ptr<Feature> featureRef(f);
                FeatureList flist = {featureRef};
                osg::ref_ptr<FeatureListCursor> towerCursor = new FeatureListCursor(flist);
                // See comment below
                GeomFeatureNodeFactory::createOrUpdateNode(towerCursor.get(), combinedStyle, localCX, towerNode, query);
                towersNode->addChild(towerNode.get());
            }
        }
    }
    else
    {
        PowerlineLayer::ModelOptions modelOptions = _powerlineOptions.towerModels().front();
        const Style* sessionTowerStyle
            = context.getSession()->styles()->getStyle("towers", false);
        if (sessionTowerStyle)
        {
            combinedStyle = *sessionTowerStyle;
        }
        if (modelOptions.uri().isSet())
        {
            setModelStyleDefaults(combinedStyle, modelOptions.uri().get(), _powerlineOptions.referrer, true);
        }
        else
        {
            setModelStyleDefaults(combinedStyle);
        }
        // This has the side effect of updating the elevations of the point features according to the
        // model style sheet. We rely on this in makeCableFeatures().
        GeomFeatureNodeFactory::createOrUpdateNode(listCursor.get(), combinedStyle, localCX, pointsNode, query);
    }

    osg::ref_ptr<osg::Group> results(new osg::Group);
    results->addChild(pointsNode.get());
    FeatureList cableFeatures =  makeCableFeatures(workingSet, pointSet, localCX, query,
                                                   cableStyle);

    GeometryCompiler compiler;
    if (_cableExpr.isSet())
    {
        for (FeatureList::iterator i = cableFeatures.begin(); i != cableFeatures.end(); ++i)
        {
            Style localStyle;
            evalStyle(i->get(), localCX, _cableExpr.get(), localStyle);
            setCableStyleDefaults(localStyle);
            osg::Node* cable = compiler.compile(i->get(), localStyle, localCX);
            results->addChild(cable);
        }
    }
    else
    {
        osg::Node* cables = compiler.compile(cableFeatures, cableStyle, localCX);
        results->addChild(cables);
    }
    node = results;
    return true;
}
