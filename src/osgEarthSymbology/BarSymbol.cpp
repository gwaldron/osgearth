/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthSymbology/BarSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(bar, BarSymbol)

BarSymbol::BarSymbol(const BarSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_value(rhs._value),
_width(rhs._width),
_minimumValue(rhs._minimumValue),
_maximumValue(rhs._maximumValue),
_minimumColor(rhs._minimumColor),
_maximumColor(rhs._maximumColor)
{
}

BarSymbol::BarSymbol( const Config& conf ) :
    Symbol( conf ),
    _value( 0 ),
    _width    ( 4.0),
    _minimumValue(0),
    _maximumValue(100),
    _minimumColor(Fill(1, 0, 0, 1)),
    _maximumColor(Fill(1, 0, 0, 1))
{
    if ( !conf.empty() )
        mergeConfig(conf);
}

Config 
BarSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "bar";
    conf.set( "value", _value);
    conf.set("min_value", _minimumValue);
    conf.set("max_value", _maximumValue);
    conf.set( "width", _width);
    return conf;
}

void 
BarSymbol::mergeConfig( const Config& conf )
{
    conf.get( "value", _value);
    conf.get("min_value", _minimumValue);
    conf.get("max_value", _maximumValue);
    conf.get( "width", _width);
}

void
BarSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "bar-value") ) {
        style.getOrCreate<BarSymbol>()->value() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "bar-width") ) {
        style.getOrCreate<BarSymbol>()->width() = NumericExpression(c.value());
    }
    else if (match(c.key(), "bar-min-value")) {
        style.getOrCreate<BarSymbol>()->minimumValue() = NumericExpression(c.value());
    }
    else if (match(c.key(), "bar-max-value")) {
        style.getOrCreate<BarSymbol>()->maximumValue() = NumericExpression(c.value());
    }
    else if (match(c.key(), "bar-color")) {
        style.getOrCreate<BarSymbol>()->minimumColor()->color() = Color(c.value());
        style.getOrCreate<BarSymbol>()->maximumColor()->color() = Color(c.value());
    }
    else if (match(c.key(), "bar-min-color")) {
        style.getOrCreate<BarSymbol>()->minimumColor()->color() = Color(c.value());
    }
    else if (match(c.key(), "bar-max-color")) {
        style.getOrCreate<BarSymbol>()->maximumColor()->color() = Color(c.value());
    }

}
