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
#include <osgEarthSymbology/ModelSymbol>
#include <osgEarthSymbology/ModelResource>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(model, ModelSymbol);

ModelSymbol::ModelSymbol( const Config& conf ) :
InstanceSymbol( conf ),
_heading  ( NumericExpression(0.0) ),
_pitch    ( NumericExpression(0.0) ),
_roll     ( NumericExpression(0.0) ),
_autoScale( false )
{
    mergeConfig( conf );
}

Config 
ModelSymbol::getConfig() const
{
    Config conf = InstanceSymbol::getConfig();
    conf.key() = "model";
    conf.addObjIfSet( "heading",    _heading );
    conf.addObjIfSet( "pitch",      _pitch );
    conf.addObjIfSet( "roll",       _roll );
    
    conf.addIfSet( "auto_scale", _autoScale );
    conf.addIfSet( "alias_map", _uriAliasMap );

    conf.addNonSerializable( "ModelSymbol::node", _node.get() );
    return conf;
}

void 
ModelSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "heading", _heading );
    conf.getObjIfSet( "pitch",   _pitch );
    conf.getObjIfSet( "roll",    _roll );

    conf.getIfSet( "auto_scale", _autoScale );
    conf.getIfSet( "alias_map", _uriAliasMap );

    _node = conf.getNonSerializable<osg::Node>( "ModelSymbol::node" );
}

InstanceResource*
ModelSymbol::createResource() const
{
    return new ModelResource();
}

void
ModelSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "model") ) {
        style.getOrCreate<ModelSymbol>()->url() = c.value();
        style.getOrCreate<ModelSymbol>()->url()->setURIContext( c.referrer() );
    }
    else if ( match(c.key(), "model-placement") ) {
        if      ( match(c.value(), "vertex") )   
            style.getOrCreate<ModelSymbol>()->placement() = ModelSymbol::PLACEMENT_VERTEX;
        else if ( match(c.value(), "interval") ) 
            style.getOrCreate<ModelSymbol>()->placement() = ModelSymbol::PLACEMENT_INTERVAL;
        else if ( match(c.value(), "random") )   
            style.getOrCreate<ModelSymbol>()->placement() = ModelSymbol::PLACEMENT_RANDOM;
        else if ( match(c.value(), "centroid") ) 
            style.getOrCreate<ModelSymbol>()->placement() = ModelSymbol::PLACEMENT_CENTROID;
    }
    else if ( match(c.key(), "model-density") ) {
        style.getOrCreate<ModelSymbol>()->density() = as<float>(c.value(), 1.0f);
    }
    else if ( match(c.key(), "model-random-seed") ) {
        style.getOrCreate<ModelSymbol>()->randomSeed() = as<unsigned>(c.value(), 0);
    }
    else if ( match(c.key(), "model-scale") ) {
        if ( match(c.value(), "auto") )
            style.getOrCreate<ModelSymbol>()->autoScale() = true;
        else
            style.getOrCreate<ModelSymbol>()->scale() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "model-heading") ) {
        style.getOrCreate<ModelSymbol>()->heading() = NumericExpression(c.value());
    }

}