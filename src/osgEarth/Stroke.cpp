/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Stroke>

using namespace osgEarth;

//------------------------------------------------------------------------


Stroke::Stroke( float r, float g, float b, float a ) :
    Stroke(Color(r,g,b,a))
{
    //nop
}

Stroke::Stroke(const Color& color)
{
    _color = color;
}

Stroke::Stroke(const Config& conf)
{
    mergeConfig( conf );
}

Stroke::Stroke(const Stroke& rhs)
{
    mergeConfig( rhs.getConfig() );
}

Config
Stroke::getConfig() const {
    Config conf("stroke");
    conf.set("color", color());
    conf.set("linecap", "flat", _lineCap, LINECAP_FLAT);
    conf.set("linecap", "square", _lineCap, LINECAP_SQUARE);
    conf.set("linecap", "round", _lineCap, LINECAP_ROUND);
    conf.set("linejoin", "mitre", _lineJoin, LINEJOIN_MITRE);
    conf.set("linejoin", "round", _lineJoin, LINEJOIN_ROUND);
    conf.set("width", _width);
    conf.set("stipple_factor", _stippleFactor);
    conf.set("stipple_pattern", _stipplePattern);
    conf.set("rounding_ratio", _roundingRatio);
    if (_widthUnits.isSet())
        conf.set("width_units", _widthUnits->getAbbr());
    conf.set("min_pixels", _minPixels);
    conf.set("smooth", _smooth);
    conf.set("outline_color", outlineColor());
    conf.set("outline_width", outlineWidth());
    return conf;
}

void
Stroke::mergeConfig(const Config& conf) {
    conf.get("color", color());
    conf.get("linecap", "flat", _lineCap, LINECAP_FLAT);
    conf.get("linecap", "square", _lineCap, LINECAP_SQUARE);
    conf.get("linecap", "round", _lineCap, LINECAP_ROUND);
    conf.get("linejoin", "mitre", _lineJoin, LINEJOIN_MITRE);
    conf.get("linejoin", "miter", _lineJoin, LINEJOIN_MITRE); // alternate spelling
    conf.get("linejoin", "round", _lineJoin, LINEJOIN_ROUND);
    conf.get("width", _width);
    conf.get("stipple", _stipplePattern); // back compat
    conf.get("stipple_factor", _stippleFactor);
    conf.get("stipple_pattern", _stipplePattern);
    conf.get("rounding_ratio", _roundingRatio);
    if (conf.hasValue("width_units"))
        Units::parse(conf.value("width_units"), _widthUnits.mutable_value());
    conf.get("min_pixels", _minPixels);
    conf.get("smooth", _smooth);
    conf.get("outline_color", outlineColor());
    conf.get("outline_width", outlineWidth());
}
