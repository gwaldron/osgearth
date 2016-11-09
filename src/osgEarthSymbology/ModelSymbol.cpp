/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

ModelSymbol::ModelSymbol(const ModelSymbol& rhs,const osg::CopyOp& copyop):
InstanceSymbol(rhs, copyop),
_heading(rhs._heading),
_pitch(rhs._pitch),
_roll(rhs._roll),
_autoScale(rhs._autoScale),
_minAutoScale(rhs._minAutoScale),
_maxAutoScale(rhs._maxAutoScale),
_name(rhs._name),
_node(rhs._node),
_maxSizeX(rhs._maxSizeX),
_maxSizeY(rhs._maxSizeY),
_scaleX( rhs._scaleX ),
_scaleY( rhs._scaleY ),
_scaleZ( rhs._scaleZ )
{
    // nop
}

ModelSymbol::ModelSymbol( const Config& conf ) :
InstanceSymbol( conf ),
_heading  ( NumericExpression(0.0) ),
_pitch    ( NumericExpression(0.0) ),
_roll     ( NumericExpression(0.0) ),
_autoScale( false ),
_minAutoScale	( 0.0 ),
_maxAutoScale	( DBL_MAX ),
_maxSizeX ( FLT_MAX ),
_maxSizeY ( FLT_MAX ),
_scaleX    ( NumericExpression(1.0) ),
_scaleY    ( NumericExpression(1.0) ),
_scaleZ    ( NumericExpression(1.0) )
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
    conf.addObjIfSet( "name",       _name );
    
    conf.addIfSet( "auto_scale", _autoScale );
	conf.addIfSet( "min_auto_scale", _minAutoScale );
	conf.addIfSet( "max_auto_scale", _maxAutoScale );
    conf.addIfSet( "alias_map", _uriAliasMap );

    conf.addIfSet( "max_size_x", _maxSizeX );
    conf.addIfSet( "max_size_y", _maxSizeY );
    
    conf.addObjIfSet( "scale_x", _scaleX );
    conf.addObjIfSet( "scale_y", _scaleY );
    conf.addObjIfSet( "scale_z", _scaleZ );

    conf.addNonSerializable( "ModelSymbol::node", _node.get() );
    return conf;
}

void 
ModelSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "heading", _heading );
    conf.getObjIfSet( "pitch",   _pitch );
    conf.getObjIfSet( "roll",    _roll );
    conf.getObjIfSet( "name",    _name );

    conf.getIfSet( "max_size_x", _maxSizeX );
    conf.getIfSet( "max_size_y", _maxSizeY );

    conf.getIfSet( "auto_scale", _autoScale );
	conf.getIfSet( "min_auto_scale", _minAutoScale);
	conf.getIfSet( "max_auto_scale", _maxAutoScale);
    conf.getIfSet( "alias_map", _uriAliasMap );
    
    conf.getObjIfSet( "scale_x", _scaleX );
    conf.getObjIfSet( "scale_y", _scaleY );
    conf.getObjIfSet( "scale_z", _scaleZ );

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
    else if ( match(c.key(),"model-library") ) {
        style.getOrCreate<ModelSymbol>()->library() = StringExpression(c.value());
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
	else if (match(c.key(), "model-min-auto-scale")) {
		style.getOrCreate<ModelSymbol>()->minAutoScale() = as<double>(c.value(), 0.0f);
	}
	else if (match(c.key(), "model-max-auto-scale")) {
		style.getOrCreate<ModelSymbol>()->maxAutoScale() = as<double>(c.value(), DBL_MAX);
	}
    else if ( match(c.key(), "model-scale-x") ) {
        style.getOrCreate<ModelSymbol>()->scaleX() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "model-scale-y") ) {
        style.getOrCreate<ModelSymbol>()->scaleY() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "model-scale-z") ) {
        style.getOrCreate<ModelSymbol>()->scaleZ() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "model-heading") ) {
        style.getOrCreate<ModelSymbol>()->heading() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "model-script") ) {
        style.getOrCreate<ModelSymbol>()->script() = StringExpression(c.value());
    }
    else if ( match(c.key(), "model-name") ) {
        style.getOrCreate<ModelSymbol>()->name() = StringExpression(c.value());
    }
    else if ( match(c.key(), "model-max-size-x") ) {
        style.getOrCreate<ModelSymbol>()->maxSizeX() = as<float>(c.value(), FLT_MAX);
    }
    else if ( match(c.key(), "model-max-size-y") ) {
        style.getOrCreate<ModelSymbol>()->maxSizeY() = as<float>(c.value(), FLT_MAX);
    }
}

