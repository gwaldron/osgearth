/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/PolygonSymbol>
#include <osgEarth/Style>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(polygon, PolygonSymbol);

PolygonSymbol::PolygonSymbol(const PolygonSymbol& rhs, const osg::CopyOp& copyop) :
    Symbol(rhs, copyop),
    _fill(rhs._fill),
    _outline(rhs._outline)
{
    //nop
}

PolygonSymbol::PolygonSymbol(const Config& conf) :
    Symbol(conf)
{
    mergeConfig(conf);
}

Config
PolygonSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "polygon";
    conf.set("fill", fill());
    conf.set("outline", outline());
    conf.set("material", material());
    return conf;
}

void
PolygonSymbol::mergeConfig(const Config& conf)
{
    conf.get("fill", fill());
    conf.get("outline", outline());
    conf.get("material", material());
}

void
PolygonSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "library")) {
        if (!c.value().empty())
            style.getOrCreate<SkinSymbol>()->library() = Strings::unquote(c.value());
    }
    else
    if (match(c.key(), "fill")) {
        style.getOrCreate<PolygonSymbol>()->fill().mutable_value().color() = Color(c.value());
    }
    else if (match(c.key(), "fill-opacity")) {
        style.getOrCreate<PolygonSymbol>()->fill().mutable_value().color().a() = as<float>(c.value(), 1.0f);
    }
    else if (match(c.key(), "fill-script")) {
        style.getOrCreate<PolygonSymbol>()->script() = StringExpression(c.value());
    }
    else if (match(c.key(), "fill-outline")) {
        style.getOrCreate<PolygonSymbol>()->outline() = as<bool>(c.value(), true);
    }
    else if (match(c.key(), "fill-material")) {
        style.getOrCreate<PolygonSymbol>()->material() = URI(c.value(), c.referrer());
    }
}
