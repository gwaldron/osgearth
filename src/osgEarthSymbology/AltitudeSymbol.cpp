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
Symbol(),
_clamping( CLAMP_NONE ),
_verticalOffset( 0 )
{
    mergeConfig( conf );
}

Config 
AltitudeSymbol::getConfig() const
{
    Config conf;
    conf.addIfSet( "clamping",  "none",     _clamping, CLAMP_NONE );
    conf.addIfSet( "clamping",  "terrain",  _clamping, CLAMP_TO_TERRAIN );
    conf.addIfSet( "clamping",  "relative", _clamping, CLAMP_RELATIVE_TO_TERRAIN );
    conf.addIfSet( "vertical_offset", _verticalOffset );
    return conf;
}

void 
AltitudeSymbol::mergeConfig( const Config& conf )
{
    conf.getIfSet( "clamping",  "none",     _clamping, CLAMP_NONE );
    conf.getIfSet( "clamping",  "terrain",  _clamping, CLAMP_TO_TERRAIN );
    conf.getIfSet( "clamping",  "relative", _clamping, CLAMP_RELATIVE_TO_TERRAIN );
    conf.getIfSet( "vertical_offset", _verticalOffset );
}
