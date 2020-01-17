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
#include <osgEarthSymbology/AltitudeSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(altitude, AltitudeSymbol);

AltitudeSymbol::AltitudeSymbol( const Config& conf ) :
Symbol             ( conf ),
_clamping          ( CLAMP_NONE ),
_technique         ( TECHNIQUE_MAP ),
_binding           ( BINDING_VERTEX ),
_resolution        ( 0.0 ), //0.001f ),
_verticalScale     ( NumericExpression(1.0) ),
_verticalOffset    ( NumericExpression(0.0) )
{
    mergeConfig( conf );
}

AltitudeSymbol::AltitudeSymbol(const AltitudeSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_clamping(rhs._clamping),
_technique(rhs._technique),
_binding(rhs._binding),
_resolution(rhs._resolution),
_verticalOffset(rhs._verticalOffset),
_verticalScale(rhs._verticalScale)
{

}

Config 
AltitudeSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();

    conf.key() = "altitude";
    conf.set( "clamping", "none",     _clamping, CLAMP_NONE );
    conf.set( "clamping", "terrain",  _clamping, CLAMP_TO_TERRAIN );
    conf.set( "clamping", "absolute", _clamping, CLAMP_ABSOLUTE );
    conf.set( "clamping", "relative", _clamping, CLAMP_RELATIVE_TO_TERRAIN );

    conf.set( "technique", "map",   _technique, TECHNIQUE_MAP );
    conf.set( "technique", "scene", _technique, TECHNIQUE_SCENE );
    conf.set( "technique", "gpu",   _technique, TECHNIQUE_GPU );
    conf.set( "technique", "drape", _technique, TECHNIQUE_DRAPE );

    conf.set( "binding", "vertex",   _binding, BINDING_VERTEX );
    conf.set( "binding", "centroid", _binding, BINDING_CENTROID );

    conf.set( "clamping_resolution", _resolution );
    conf.set( "vertical_offset",     _verticalOffset );
    conf.set( "vertical_scale",      _verticalScale );
    return conf;
}

void 
AltitudeSymbol::mergeConfig( const Config& conf )
{
    conf.get( "clamping",  "none",     _clamping, CLAMP_NONE );
    conf.get( "clamping",  "terrain",  _clamping, CLAMP_TO_TERRAIN );
    conf.get( "clamping",  "absolute", _clamping, CLAMP_ABSOLUTE );
    conf.get( "clamping",  "relative", _clamping, CLAMP_RELATIVE_TO_TERRAIN );

    conf.get( "technique", "map",   _technique, TECHNIQUE_MAP );
    conf.get( "technique", "scene", _technique, TECHNIQUE_SCENE );
    conf.get( "technique", "gpu",   _technique, TECHNIQUE_GPU );
    conf.get( "technique", "drape", _technique, TECHNIQUE_DRAPE );

    conf.get( "binding", "vertex",   _binding, BINDING_VERTEX );
    conf.get( "binding", "centroid", _binding, BINDING_CENTROID );

    conf.get( "clamping_resolution", _resolution );
    conf.get( "vertical_offset",     _verticalOffset );
    conf.get( "vertical_scale",      _verticalScale );
}

void
AltitudeSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "altitude-clamping") ) {
        if      ( match(c.value(), "none") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping() = CLAMP_NONE;
        }
        else if ( match(c.value(), "terrain") )  {
            style.getOrCreate<AltitudeSymbol>()->clamping() = CLAMP_TO_TERRAIN;
        }
        else if ( match(c.value(), "absolute") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping() = CLAMP_ABSOLUTE;
        }
        else if ( match(c.value(), "relative") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping() = CLAMP_RELATIVE_TO_TERRAIN;
        }
        else if ( match(c.value(), "relative-gpu") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping() = CLAMP_RELATIVE_TO_TERRAIN;
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_GPU;
        }
        else if ( match(c.value(), "terrain-drape") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping()  = CLAMP_TO_TERRAIN;
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_DRAPE;
        }
        else if ( match(c.value(), "terrain-gpu") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping()  = CLAMP_TO_TERRAIN;
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_GPU;
        }
        else if ( match(c.value(), "terrain-scene") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping()  = CLAMP_TO_TERRAIN;
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_SCENE;
        }
        else if ( match(c.value(), "relative-scene") ) {
            style.getOrCreate<AltitudeSymbol>()->clamping()  = CLAMP_RELATIVE_TO_TERRAIN;
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_SCENE;
        }
    }
    else if ( match(c.key(), "altitude-technique") ) {
        if      ( match(c.value(), "map") )
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_MAP;
        else if ( match(c.value(), "scene") )
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_SCENE;
        else if ( match(c.value(), "gpu") )
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_GPU;
        else if ( match(c.value(), "drape") )
            style.getOrCreate<AltitudeSymbol>()->technique() = TECHNIQUE_DRAPE;
    }
    else if ( match(c.key(), "altitude-binding") ) {
        if      ( match(c.value(), "vertex") )
            style.getOrCreate<AltitudeSymbol>()->binding() = BINDING_VERTEX;
        else if ( match(c.value(), "centroid") )
            style.getOrCreate<AltitudeSymbol>()->binding() = BINDING_CENTROID;
    }
    else if ( match(c.key(), "altitude-resolution") ) {
        style.getOrCreate<AltitudeSymbol>()->clampingResolution() = as<float>( c.value(), 0.0f );
    }
    else if ( match(c.key(), "altitude-offset") ) {
        style.getOrCreate<AltitudeSymbol>()->verticalOffset() = NumericExpression( c.value() );
    }
    else if ( match(c.key(), "altitude-scale") ) {
        style.getOrCreate<AltitudeSymbol>()->verticalScale() = NumericExpression( c.value() );
    }
    else if ( match(c.key(), "altitude-script") ) {
        style.getOrCreate<AltitudeSymbol>()->script() = StringExpression(c.value());
    }
}
