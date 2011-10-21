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
#include <osgEarthSymbology/AltitudeSymbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

AltitudeSymbol::AltitudeSymbol( const Config& conf ) :
Symbol             ( conf ),
_clamping          ( CLAMP_NONE ),
_clampingResolution( 0.0f )
{
    mergeConfig( conf );
}

Config 
AltitudeSymbol::getConfig() const
{
    Config conf;
    conf.key() = "altitude";
    conf.addIfSet   ( "clamping",  "none",     _clamping, CLAMP_NONE );
    conf.addIfSet   ( "clamping",  "terrain",  _clamping, CLAMP_TO_TERRAIN );
    conf.addIfSet   ( "clamping",  "absolute", _clamping, CLAMP_ABSOLUTE );
    conf.addIfSet   ( "clamping",  "relative", _clamping, CLAMP_RELATIVE_TO_TERRAIN );
    conf.addIfSet   ( "clamping_resolution",   _clampingResolution );
    conf.addObjIfSet( "vertical_offset",       _verticalOffset );
    conf.addObjIfSet( "vertical_scale",        _verticalScale );
    return conf;
}

void 
AltitudeSymbol::mergeConfig( const Config& conf )
{
    conf.getIfSet   ( "clamping",  "none",     _clamping, CLAMP_NONE );
    conf.getIfSet   ( "clamping",  "terrain",  _clamping, CLAMP_TO_TERRAIN );
    conf.getIfSet   ( "clamping",  "absolute", _clamping, CLAMP_ABSOLUTE );
    conf.getIfSet   ( "clamping",  "relative", _clamping, CLAMP_RELATIVE_TO_TERRAIN );
    conf.getIfSet   ( "clamping_resolution",   _clampingResolution );
    conf.getObjIfSet( "vertical_offset",       _verticalOffset );
    conf.getObjIfSet( "vertical_scale",        _verticalScale );
}
