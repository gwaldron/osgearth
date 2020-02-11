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
#include <osgEarth/CoverageSymbol>
#include <osgEarth/Style>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(coverage, CoverageSymbol);

CoverageSymbol::CoverageSymbol(const CoverageSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_valueExpr( rhs._valueExpr )
{
    //nop
}

CoverageSymbol::CoverageSymbol( const Config& conf ) :
Symbol( conf )
{
    mergeConfig(conf);
}

Config 
CoverageSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "coverage";
    conf.set( "value", _valueExpr );
    return conf;
}

void 
CoverageSymbol::mergeConfig( const Config& conf )
{
    conf.get( "value", _valueExpr );
}


void
CoverageSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "coverage-value") ) {
        style.getOrCreate<CoverageSymbol>()->valueExpression() = NumericExpression(c.value());
    }
}

