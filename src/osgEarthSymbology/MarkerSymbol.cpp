/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthSymbology/MarkerSymbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

MarkerSymbol::MarkerSymbol( const Config& conf ) :
_placement( PLACEMENT_CENTROID ),
_density( 25.0f ),
_scale( osg::Vec3f(1,1,1) ),
_randomSeed( 0 )
{
    mergeConfig( conf );
}

Config 
MarkerSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "marker";
    conf.addIfSet( "url", _url );
    conf.addIfSet( "placement", "centroid", _placement, PLACEMENT_CENTROID );
    conf.addIfSet( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.addIfSet( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.addIfSet( "density", _density );
    conf.addIfSet( "scale", _scale );
    conf.addIfSet( "random_seed", _randomSeed );
    conf.addNonSerializable( "MarkerSymbol::image", _image.get() );
    conf.addNonSerializable( "MarkerSymbol::node", _node.get() );
    return conf;
}

void 
MarkerSymbol::mergeConfig( const Config& conf )
{
    conf.getIfSet( "url", _url );
    conf.getIfSet( "placement", "centroid", _placement, PLACEMENT_CENTROID );
    conf.getIfSet( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.getIfSet( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.getIfSet( "density", _density );
    conf.getIfSet( "scale", _scale );
    conf.getIfSet( "random_seed", _randomSeed );
    _image = conf.getNonSerializable<osg::Image>( "MarkerSymbol::image" );
    _node = conf.getNonSerializable<osg::Node>( "MarkerSymbol::node" );
}

