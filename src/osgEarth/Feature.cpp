/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Feature>
#include <osgEarth/FilterContext>
#include <osgEarth/GeometryUtils>
#include <osgEarth/ScriptEngine>

#include <osgEarth/StringUtils>
#include <osgEarth/JsonUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[Feature] "

//----------------------------------------------------------------------------

FeatureProfile::FeatureProfile(const GeoExtent& extent) :
    _extent(extent)
{
    //nop
}

FeatureProfile::FeatureProfile(const Profile* tilingProfile) :
    _tilingProfile(tilingProfile),
    _extent(tilingProfile ? tilingProfile->getExtent() : GeoExtent::INVALID)
{
    //nop
}

bool
FeatureProfile::isTiled() const
{
    return getTilingProfile() != 0L;
}

int
FeatureProfile::getFirstLevel() const
{
    return _firstLevel;
}

void
FeatureProfile::setFirstLevel(int firstLevel )
{
    _firstLevel = firstLevel;
}

int
FeatureProfile::getMaxLevel() const
{
    return _maxLevel;
}

void
FeatureProfile::setMaxLevel(int maxLevel)
{
    _maxLevel = maxLevel;
}

const osgEarth::Profile*
FeatureProfile::getTilingProfile() const
{
    return _tilingProfile.get();
}

void
FeatureProfile::setTilingProfile( const osgEarth::Profile* profile )
{
    _tilingProfile = profile;
}

//----------------------------------------------------------------------------

Feature::Feature(FeatureID fid) :
    _fid(fid)
{
    //NOP
}

Feature::Feature(Geometry* geom, const SpatialReference* srs, const Style& style, FeatureID fid) :
    _geom(geom),
    _srs(srs),
    _fid(fid)
{
    if (!style.empty())
    {
        _style = std::make_shared<Style>(style);
    }
}

Feature::Feature(const Feature& rhs) :
    _fid(rhs._fid),
    _attrs(rhs._attrs),
    _style(rhs._style),
    _geoInterp(rhs._geoInterp),
    _srs(rhs._srs.get())
{
    OE_SOFT_ASSERT(rhs._geom.valid());

    if (rhs._geom.valid())
        _geom = rhs._geom->clone();
}

void
Feature::setFID(FeatureID fid)
{
    _fid = fid;
}

GeoExtent
Feature::getExtent() const
{
    if (!getSRS() || !getGeometry())
    {
        return GeoExtent::INVALID;
    }
    return GeoExtent(getSRS(), getGeometry()->getBounds());
}

void
Feature::setSRS( const SpatialReference* srs )
{
    _srs = srs;
}

void
Feature::setGeometry( Geometry* geom )
{
    OE_HARD_ASSERT(geom != nullptr);
    _geom = geom;
}

void
Feature::setStyle(const Style& style)
{
    if (style.empty())
        _style = nullptr;
    else
        _style = std::make_shared<Style>(style);
}

Style&
Feature::getOrCreateStyle()
{
    if (!_style)
    {
        _style = std::make_shared<Style>();
    }
    return *_style.get();
}

void
Feature::set(const std::string& name, const std::string& value)
{
    _attrs[toLower(name)].emplace<std::string>(value);
}

void
Feature::set(const std::string& name, const char* value)
{
    _attrs[toLower(name)].emplace<std::string>(std::string(value));
}

void
Feature::set(const std::string& name, double value)
{
    _attrs[toLower(name)].emplace<double>(value);
}

void
Feature::set(const std::string& name, long long value)
{
    _attrs[toLower(name)].emplace<long long>(value);
}

void
Feature::set(const std::string& name, int value)
{
    _attrs[toLower(name)].emplace<long long>(static_cast<long long>(value));
}

void
Feature::set(const std::string& name, bool value)
{
    _attrs[toLower(name)].emplace<bool>(value);
}

void
Feature::set(const std::string& name, const AttributeValue& value)
{
    _attrs[toLower(name)] = value;
}

void
Feature::setNull(const std::string& name)
{
    _attrs[toLower(name)].emplace<std::monostate>();
}

void
Feature::removeAttribute(const std::string& name)
{
    _attrs.erase(toLower(name));
}

bool
Feature::hasAttr( const std::string& name ) const
{
    return _attrs.find(toLower(name)) != _attrs.end();
}

std::string
Feature::getString( const std::string& name ) const
{
    auto i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getString() : EMPTY_STRING;
}

double
Feature::getDouble( const std::string& name, double defaultValue ) const
{
    auto i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getDouble(defaultValue) : defaultValue;
}

long long
Feature::getInt( const std::string& name, long long defaultValue ) const
{
    auto i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getInt(defaultValue) : defaultValue;
}

bool
Feature::getBool( const std::string& name, bool defaultValue ) const
{
    auto i = _attrs.find(toLower(name));
    return i != _attrs.end()? i->second.getBool(defaultValue) : defaultValue;
}

bool
Feature::isSet(const std::string& name) const
{
    auto i = _attrs.find(toLower(name));
    return i != _attrs.end() ? i->second.getType() != ATTRTYPE_UNSPECIFIED : false;
}

double
Feature::eval(const NumericExpression& expr, FilterContext const* context)
{
    NumericExpression temp(expr);
    return eval(temp, context);
}

double
Feature::eval( NumericExpression& expr, FilterContext const* context )
{
    const NumericExpression::Variables& vars = expr.variables();
    for (NumericExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
    {
        double val = 0.0;
        AttributeTable::const_iterator ai = _attrs.find(toLower(i->first));
        if (ai != _attrs.end())
        {
            val = ai->second.getDouble(0.0);
        }
        else if (context && context->getSession())
        {
            //No attr found, look for script
            ScriptEngine* engine = context->getSession()->getScriptEngine();
            if (engine)
            {
                ScriptResult result = engine->run(i->first, this, context);
                if (result.success())
                    val = result.asDouble();
                //else {
                //    OE_WARN << LC << "Feature Script error on '" << expr.expr() << "': " << result.message() << std::endl;
                //}
            }
        }

        expr.set(*i, val);
    }

    return expr.eval();
}

double
Feature::eval(const NumericExpression& expr, Session* session)
{
    NumericExpression temp(expr);
    return eval(temp, session);
}

double
Feature::eval(NumericExpression& expr, Session* session)
{
    const NumericExpression::Variables& vars = expr.variables();
    for( NumericExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i )
    {
        double val = 0.0;
        AttributeTable::const_iterator ai = _attrs.find(toLower(i->first));
        if (ai != _attrs.end())
        {
            val = ai->second.getDouble(0.0);
        }
        else if (session)
        {
            //No attr found, look for script
            ScriptEngine* engine = session->getScriptEngine();
            if (engine)
            {
                ScriptResult result = engine->run(i->first, this);
                if (result.success())
                    val = result.asDouble();
                //else {
                //    OE_WARN << LC << "Feature Script error on '" << expr.expr() << "': " << result.message() << std::endl;
                //}
            }
        }

        expr.set( *i, val );
    }

    return expr.eval();
}

std::string
Feature::eval(const StringExpression& expr, FilterContext const* context)
{
    StringExpression temp(expr);
    return eval(temp, context);
}

const std::string&
Feature::eval(StringExpression& expr, FilterContext const* context)
{
    const StringExpression::Variables& vars = expr.variables();
    for (StringExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
    {
        std::string val = "";
        AttributeTable::const_iterator ai = _attrs.find(toLower(i->first));
        if (ai != _attrs.end())
        {
            val = ai->second.getString();
        }
        else if (context && context->getSession())
        {
            //No attr found, look for script
            ScriptEngine* engine = context->getSession()->getScriptEngine();
            if (engine)
            {
                ScriptResult result = engine->run(i->first, this, context);
                if (result.success())
                    val = result.asString();
                else
                {
                    // Couldn't execute it as code, just take it as a string literal.
                    val = i->first;
                    //OE_DEBUG << LC << "Feature Script error on '" << expr.expr() << "': " << result.message() << std::endl;
                }
            }
        }

        expr.set(*i, val);
    }

    return expr.eval();
}

std::string
Feature::eval(const StringExpression& expr, Session* session)
{
    StringExpression temp(expr);
    return eval(temp, session);
}

const std::string&
Feature::eval(StringExpression& expr, Session* session)
{
    const StringExpression::Variables& vars = expr.variables();
    for( StringExpression::Variables::const_iterator i = vars.begin(); i != vars.end(); ++i )
    {
        std::string val = "";
        AttributeTable::const_iterator ai = _attrs.find(toLower(i->first));
        if (ai != _attrs.end())
        {
            val = ai->second.getString();
        }
        else if (session)
        {
            //No attr found, look for script
            ScriptEngine* engine = session->getScriptEngine();
            if (engine)
            {
                ScriptResult result = engine->run(i->first, this);
                if (result.success())
                    val = result.asString();
                else
                {
                    // Couldn't execute it as code, just take it as a string literal.
                    val = i->first;
                    //OE_DEBUG << LC << "Feature Script error on '" << expr.expr() << "': " << result.message() << std::endl;
                }
            }
        }

        expr.set( *i, val );
    }

    return expr.eval();
}


bool
Feature::getWorldBound(const SpatialReference* srs,
                       osg::BoundingSphered&   out_bound) const
{
    if ( srs && getSRS() && getGeometry() )
    {
        osg::BoundingBoxd box;
        //out_bound.init();

        ConstGeometryIterator i( getGeometry(), false);
        while( i.hasMore() )
        {
            const Geometry* g = i.next();
            for( Geometry::const_iterator p = g->begin(); p != g->end(); ++p )
            {
                GeoPoint point( getSRS(), *p, ALTMODE_ABSOLUTE );
                GeoPoint srs_point;
                if ( point.transform( srs, srs_point ) )
                {
                    osg::Vec3d world;
                    srs_point.toWorld(world);
                    box.expandBy(world);
                    //out_bound.expandBy( world );
                }
            }
        }
        out_bound = osg::BoundingSphered(box);

        if ( out_bound.valid() && out_bound.radius() == 0.0 )
        {
            out_bound.radius() = 1.0;
        }
        return true;
    }
    return false;
}


bool
Feature::getWorldBoundingPolytope(const SpatialReference* srs,
                                  osg::Polytope&          out_polytope) const
{
    osg::BoundingSphered bs;
    if ( getWorldBound(srs, bs) && bs.valid() )
    {
        return getWorldBoundingPolytope( bs, srs, out_polytope );
    }
    return false;
}

bool Feature::getWorldBoundingPolytope( const osg::BoundingSphered& bs, const SpatialReference* srs, osg::Polytope& out_polytope )
{
    if ( bs.valid() )
    {
        out_polytope.getMaskStack().clear();
        out_polytope.clear();

        // add planes for the four sides of the BS. Normals point inwards.
        out_polytope.add( osg::Plane(osg::Vec3d( 1, 0,0), osg::Vec3d(-bs.radius(),0,0)) );
        out_polytope.add( osg::Plane(osg::Vec3d(-1, 0,0), osg::Vec3d( bs.radius(),0,0)) );
        out_polytope.add( osg::Plane(osg::Vec3d( 0, 1,0), osg::Vec3d(0, -bs.radius(),0)) );
        out_polytope.add( osg::Plane(osg::Vec3d( 0,-1,0), osg::Vec3d(0,  bs.radius(),0)) );

        // for a projected feature, we're done. For a geocentric one, transform the polytope
        // into world (ECEF) space.
        if ( srs->isGeographic() )
        {
            const Ellipsoid& e = srs->getEllipsoid();

            // add a bottom cap, unless the bounds are sufficiently large.
            double minRad = std::min(e.getRadiusPolar(), e.getRadiusEquator());
            double maxRad = std::max(e.getRadiusPolar(), e.getRadiusEquator());
            double zeroOffset = bs.center().length();
            if ( zeroOffset > minRad * 0.1 )
            {
                out_polytope.add( osg::Plane(osg::Vec3d(0,0,1), osg::Vec3d(0,0,-maxRad+zeroOffset)) );
            }
        }

        // transform the clipping planes ito ECEF space
        GeoPoint refPoint;
        refPoint.fromWorld( srs, bs.center() );

        osg::Matrix local2world;
        refPoint.createLocalToWorld( local2world );

        out_polytope.transform( local2world );

        return true;
    }
    return false;
}

GeoExtent
Feature::calculateExtent() const
{
    GeoExtent e(getSRS());
    ConstGeometryIterator gi(getGeometry(), false);
    while (gi.hasMore()) {
        const Geometry* part = gi.next();
        for (Geometry::const_iterator v = part->begin(); v != part->end(); ++v)
            e.expandToInclude(*v);
    }
    return e;
}

std::string
Feature::getGeoJSON(bool includeNulls) const
{
    std::string geometry = GeometryUtils::geometryToGeoJSON( getGeometry(), getSRS() );

    Json::Value root(Json::objectValue);
    root["type"] = "Feature";
    root["id"] = (double)getFID(); //TODO:  Update JSON to use unsigned longs

    Json::Reader reader;
    Json::Value geometryValue( Json::objectValue );
    if ( reader.parse( geometry, geometryValue ) )
    {
        root["geometry"] = geometryValue;
    }

    //Write out all the properties
    Json::Value props(Json::objectValue);

    for (auto& attr : getAttrs())
    {
        auto& value = attr.second;
        if (value.is<std::string>())
            props[attr.first] = value.get<std::string>();
        else if (value.is<double>())
            props[attr.first] = value.get<double>();
        else if (value.is<long long>())
            props[attr.first] = static_cast<int>(value.get<long long>());
        else if (value.is<bool>())
            props[attr.first] = value.get<bool>();
        else if (includeNulls)
            props[attr.first] = Json::nullValue;
    }

    root["properties"] = props;
    return Json::FastWriter().write( root );
}

std::string Feature::featuresToGeoJSON(const FeatureList& features, bool includeNulls)
{
    std::stringstream buf;

    buf << "{\"type\": \"FeatureCollection\", \"features\": [";


    auto last = features.end();
    last--;

    for (FeatureList::const_iterator i = features.begin(); i != features.end(); i++)
    {
        buf << i->get()->getGeoJSON(includeNulls);
        if (i != last)
        {
            buf << ",";
        }
    }

    buf << "]}";

    return buf.str();

}

void Feature::transform( const SpatialReference* srs )
{
    if (!getGeometry())
        return;

    if (!getSRS() || !srs)
        return;

    if (getSRS()->isEquivalentTo( srs ))
        return;

    // iterate over the feature geometry.
    GeometryIterator iter( getGeometry() );
    while( iter.hasMore() )
    {
        Geometry* geom = iter.next();
        getSRS()->transform( geom->asVector(), srs );
    }
    setSRS( srs );
}

void
Feature::splitAcrossAntimeridian()
{
    // If the feature is geodetic, try to split it across the dateline.
    if (getSRS() && getSRS()->isGeodetic() && getGeometry())
    {
        auto* split_geom = getGeometry()->splitAcrossAntimeridian();
        setGeometry(split_geom);
    }
}



std::string
osgEarth::evaluateExpression(const std::string& expr, Feature* feature, const FilterContext& context)
{
    OE_SOFT_ASSERT_AND_RETURN(feature, {});
    OE_SOFT_ASSERT_AND_RETURN(context.getSession(), {});

    auto* engine = context.getSession()->getScriptEngine();
    OE_SOFT_ASSERT_AND_RETURN(engine, {});

    auto result = engine->run(expr, feature, &context);

    if (result.success())
        return result.asString();

    OE_WARN << LC << "Feature Script error on '" << expr << "': " << result.message() << std::endl;

    return {};
}
