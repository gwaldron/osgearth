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
#include <osgEarthSymbology/InstanceSymbol>
#include <osgEarthSymbology/IconSymbol>
#include <osgEarthSymbology/ModelSymbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

InstanceSymbol::InstanceSymbol( const Config& conf ) :
_placement ( PLACEMENT_VERTEX ),
_density   ( 25.0f ),
_randomSeed( 0 ),
_scale     ( NumericExpression(1.0) )
{
    mergeConfig( conf );
}

InstanceSymbol::InstanceSymbol(const InstanceSymbol& rhs,const osg::CopyOp& copyop):
Taggable<Symbol>(rhs, copyop),
_url(rhs._url),
_library(rhs._library),
_scale(rhs._scale),
_placement(rhs._placement),
_density(rhs._density),
_randomSeed(rhs._randomSeed),
_uriAliasMap(rhs._uriAliasMap),
_script(rhs._script)
{
}

Config 
InstanceSymbol::getConfig() const
{
    Config conf = Taggable<Symbol>::getConfig();
    conf.key() = "instance";
    conf.set( "url", _url );
    conf.set( "library", _library );
    conf.set( "scale", _scale );
    conf.set( "script", _script );
    conf.set( "placement", "vertex",    _placement, PLACEMENT_VERTEX );
    conf.set( "placement", "interval",  _placement, PLACEMENT_INTERVAL );
    conf.set( "placement", "random",    _placement, PLACEMENT_RANDOM );
    conf.set( "placement", "centroid",  _placement, PLACEMENT_CENTROID );
    conf.set( "density", _density );
    conf.set( "random_seed", _randomSeed );

    std::string tagstring = this->tagString();
    if ( !tagstring.empty() )
        conf.set("tags", tagstring);

    return conf;
}

void 
InstanceSymbol::mergeConfig( const Config& conf )
{
    conf.get( "url", _url );
    conf.get( "library", _library );
    conf.get( "scale", _scale );
    conf.get( "script", _script );
    conf.get( "placement", "vertex",   _placement, PLACEMENT_VERTEX );
    conf.get( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.get( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.get( "placement", "centroid", _placement, PLACEMENT_CENTROID );
    conf.get( "density", _density );
    conf.get( "random_seed", _randomSeed );
    
    addTags( conf.value("tags") );
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
