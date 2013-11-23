/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthSymbology/InstanceSymbol>
#include <osgEarthSymbology/IconSymbol>
#include <osgEarthSymbology/ModelSymbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

InstanceSymbol::InstanceSymbol( const Config& conf ) :
Symbol     ( conf ),
_placement ( PLACEMENT_CENTROID ),
_density   ( 25.0f ),
_randomSeed( 0 ),
_scale     ( NumericExpression(1.0) )
{
    mergeConfig( conf );
}

Config 
InstanceSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "instance";
    conf.addObjIfSet( "url", _url );
    conf.addObjIfSet( "library", _libraryName );
    conf.addObjIfSet( "scale", _scale );
    conf.addIfSet   ( "placement", "vertex",   _placement, PLACEMENT_VERTEX );
    conf.addIfSet   ( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.addIfSet   ( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.addIfSet   ( "density", _density );
    conf.addIfSet   ( "random_seed", _randomSeed );
    return conf;
}

void 
InstanceSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "url", _url );
    conf.getObjIfSet( "library", _libraryName );
    conf.getObjIfSet( "scale", _scale );
    conf.getIfSet   ( "placement", "vertex",   _placement, PLACEMENT_VERTEX );
    conf.getIfSet   ( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.getIfSet   ( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.getIfSet   ( "density", _density );
    conf.getIfSet   ( "random_seed", _randomSeed );
}


const IconSymbol*
InstanceSymbol::asIcon() const
{
    return dynamic_cast<const IconSymbol*>( this );
}

const ModelSymbol*
InstanceSymbol::asModel() const
{
    return dynamic_cast<const ModelSymbol*>( this );
}
