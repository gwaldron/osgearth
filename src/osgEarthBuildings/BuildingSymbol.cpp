/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2016 Pelican Mapping
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
#include "BuildingSymbol"

using namespace osgEarth;
using namespace osgEarth;
using namespace osgEarth::Buildings;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(building, BuildingSymbol);

BuildingSymbol::BuildingSymbol(const Config& conf) :
Symbol      ( conf ),
_floorHeight( 3.5f )
{
    mergeConfig(conf);
}

BuildingSymbol::BuildingSymbol(const BuildingSymbol& rhs,const osg::CopyOp& copyop) :
Symbol       ( rhs, copyop ),
_floorHeight ( rhs._floorHeight ),
_heightExpr  ( rhs._heightExpr ),
_modelURIExpr( rhs._modelURIExpr ),
_tagsExpr    ( rhs._tagsExpr ),
_libraryName ( rhs._libraryName )
{
    //nop
}

Config 
BuildingSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "building";
    conf.set( "floor_height", _floorHeight );
    conf.set( "height",       _heightExpr );
    conf.set( "tags",         _tagsExpr );
    conf.set( "model",        _modelURIExpr );
    conf.set( "library_name", _libraryName );
    return conf;
}

void 
BuildingSymbol::mergeConfig( const Config& conf )
{
    conf.get( "floor_height", _floorHeight );
    conf.get( "height",       _heightExpr );
    conf.get( "tags",         _tagsExpr );
    conf.get( "model",        _modelURIExpr );
    conf.get( "library_name", _libraryName );
}

void
BuildingSymbol::parseSLD(const Config& c, Style& style)
{
    BuildingSymbol defaults;

    if ( match(c.key(), "building-floor-height") ) {
        style.getOrCreate<BuildingSymbol>()->floorHeight() = as<float>(c.value(), *defaults.floorHeight());
    }
    else if ( match(c.key(), "building-height") ) {
        style.getOrCreate<BuildingSymbol>()->height() = !c.value().empty() ? NumericExpression(c.value()) : *defaults.height();
    }
    else if ( match(c.key(), "building-tags") ) {
        style.getOrCreate<BuildingSymbol>()->tags() = !c.value().empty() ? StringExpression(c.value()) : *defaults.tags();
    }
    else if ( match(c.key(), "building-model") ) {
        style.getOrCreate<BuildingSymbol>()->modelURI() = !c.value().empty() ? StringExpression(c.value()) : *defaults.modelURI();
    }
    else if ( match(c.key(), "building-library") ) {
        style.getOrCreate<BuildingSymbol>()->library() = !c.value().empty() ? StringExpression(c.value()) : *defaults.library();
    }
}
