/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthSymbology/ModelSymbol>
#include <osgEarthSymbology/ModelResource>

using namespace osgEarth;
using namespace osgEarth::Symbology;

ModelSymbol::ModelSymbol( const Config& conf ) :
InstanceSymbol( conf ),
_heading( NumericExpression(0.0) ),
_pitch  ( NumericExpression(0.0) ),
_roll   ( NumericExpression(0.0) )
{
    mergeConfig( conf );
}

Config 
ModelSymbol::getConfig() const
{
    Config conf = InstanceSymbol::getConfig();
    conf.key() = "model";
    conf.addObjIfSet( "heading",   _heading );
    conf.addObjIfSet( "pitch",     _pitch );
    conf.addObjIfSet( "roll",      _roll );

    conf.addIfSet   ( "alias_map", _uriAliasMap );

    conf.addNonSerializable( "ModelSymbol::node", _node.get() );
    return conf;
}

void 
ModelSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "heading", _heading );
    conf.getObjIfSet( "pitch",   _pitch );
    conf.getObjIfSet( "roll",    _roll );

    conf.getIfSet   ( "alias_map", _uriAliasMap );

    _node = conf.getNonSerializable<osg::Node>( "ModelSymbol::node" );
}

InstanceResource*
ModelSymbol::createResource() const
{
    return new ModelResource();
}
