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
#include <osgEarth/ExtrusionSymbol>
#include <osgEarth/Style>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(extrusion, ExtrusionSymbol);

ExtrusionSymbol::ExtrusionSymbol(const ExtrusionSymbol& rhs, const osg::CopyOp& copyop) :
    Symbol(rhs, copyop)
{
    _height = rhs._height;
    _flatten = rhs._flatten;
    _heightExpression = rhs._heightExpression;
    _wallStyleName = rhs._wallStyleName;
    _roofStyleName = rhs._roofStyleName;
    _wallGradientPercentage = rhs._wallGradientPercentage;
    _wallShadePercentage = rhs._wallShadePercentage;
    _wallSkinName = rhs._wallSkinName;
    _roofSkinName = rhs._roofSkinName;
    _direction = rhs._direction;
}

ExtrusionSymbol::ExtrusionSymbol(const Config& conf) :
    Symbol(conf)
{
    if (!conf.empty())
        mergeConfig(conf);
}

Config
ExtrusionSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "extrusion";
    conf.set("height", _height);
    conf.set("flatten", _flatten);
    conf.set("height_expression", heightExpression());
    conf.set("wall_style", _wallStyleName);
    conf.set("roof_style", _roofStyleName);
    conf.set("wall_gradient", _wallGradientPercentage);
    conf.set("wall_shade", _wallShadePercentage);
    conf.set("wall_skin", _wallSkinName);
    conf.set("roof_skin", _roofSkinName);
    conf.set("direction", "up", direction(), DIRECTION_UP);
    conf.set("direction", "down", direction(), DIRECTION_DOWN);
    return conf;
}

void
ExtrusionSymbol::mergeConfig(const Config& conf)
{
    conf.get("height", _height);
    conf.get("flatten", _flatten);
    conf.get("height_expression", heightExpression());
    conf.get("wall_style", _wallStyleName);
    conf.get("roof_style", _roofStyleName);
    conf.get("wall_gradient", _wallGradientPercentage);
    conf.get("wall_shade", _wallShadePercentage);
    conf.get("wall_skin", _wallSkinName);
    conf.get("roof_skin", _roofSkinName);
    conf.get("direction", "up", direction(), DIRECTION_UP);
    conf.get("direction", "down", direction(), DIRECTION_DOWN);
}

void
ExtrusionSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "library")) {
        if (!c.value().empty())
            style.getOrCreate<SkinSymbol>()->library() = Strings::unquote(c.value());
    }
    else
    if ( match(c.key(), "extrusion-height") ) {
        style.getOrCreate<ExtrusionSymbol>()->heightExpression() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "extrusion-flatten") ) {
        style.getOrCreate<ExtrusionSymbol>()->flatten() = as<bool>(c.value(), true);
    }
    else if ( match(c.key(), "extrusion-wall-style") ) {
        style.getOrCreate<ExtrusionSymbol>()->wallStyleName() = c.value();
    }
    else if ( match(c.key(), "extrusion-roof-style") ) {
        style.getOrCreate<ExtrusionSymbol>()->roofStyleName() = c.value();
    }
    else if ( match(c.key(), "extrusion-wall-gradient") ) {
        style.getOrCreate<ExtrusionSymbol>()->wallGradientPercentage() = as<float>(c.value(), 0.0f);
    }
    else if (match(c.key(), "extrusion-wall-shade")) {
        style.getOrCreate<ExtrusionSymbol>()->wallShadePercentage() = as<float>(c.value(), 0.0f);
    }
    else if ( match(c.key(), "extrusion-script") ) {
        style.getOrCreate<ExtrusionSymbol>()->script() = StringExpression(c.value());
    }
    else if ( match(c.key(), "extrusion-wall-skin") ) {
        style.getOrCreate<ExtrusionSymbol>()->wallSkinName() = c.value();
    }
    else if ( match(c.key(), "extrusion-roof-skin") ) {
        style.getOrCreate<ExtrusionSymbol>()->roofSkinName() = c.value();
    }
    else if (match(c.key(), "extrusion-direction")) {
        if (ci_equals(c.value(), "up"))
            style.getOrCreate<ExtrusionSymbol>()->direction() = ExtrusionSymbol::DIRECTION_UP;
        else if (ci_equals(c.value(), "down"))
            style.getOrCreate<ExtrusionSymbol>()->direction() = ExtrusionSymbol::DIRECTION_DOWN;
    }
}
