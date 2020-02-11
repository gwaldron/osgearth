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

ExtrusionSymbol::ExtrusionSymbol(const ExtrusionSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop)
{
    _height = rhs._height;
    _flatten = rhs._flatten;
    _heightExpr = rhs._heightExpr;
    _wallStyleName = rhs._wallStyleName;
    _roofStyleName = rhs._roofStyleName;
    _wallGradientPercentage = rhs._wallGradientPercentage;
}

ExtrusionSymbol::ExtrusionSymbol( const Config& conf ) :
Symbol    ( conf ),
_height   ( 10.0 ),
_flatten  ( true ),
_wallGradientPercentage( 0.0f )
{
    if ( !conf.empty() )
        mergeConfig(conf);
}

Config 
ExtrusionSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "extrusion";
    conf.set( "height", _height );
    conf.set( "flatten", _flatten );
    conf.set( "height_expression", _heightExpr );
    conf.set( "wall_style", _wallStyleName );
    conf.set( "roof_style", _roofStyleName );
    conf.set( "wall_gradient", _wallGradientPercentage );
    return conf;
}

void 
ExtrusionSymbol::mergeConfig( const Config& conf )
{
    conf.get( "height",  _height );
    conf.get( "flatten", _flatten );
    conf.get( "height_expression", _heightExpr );
    conf.get( "wall_style", _wallStyleName );
    conf.get( "roof_style", _roofStyleName );
    conf.get( "wall_gradient", _wallGradientPercentage );
}

void
ExtrusionSymbol::parseSLD(const Config& c, Style& style)
{
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
    else if ( match(c.key(), "extrusion-script") ) {
        style.getOrCreate<ExtrusionSymbol>()->script() = StringExpression(c.value());
    }
}
