/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthSymbology/Symbolizer>

using namespace osgEarth::Symbology;

Symbolizer::Symbolizer()
{
    //nop
}

/*************************************************************************/

Symbolizer::State::State() :
_styleRevision( -1 ),
_dataSetRevision( -1 )
{
    //nop
}

bool
Symbolizer::State::outOfSync( const Style* style ) const
{
    return style && style->outOfSync( _styleRevision );
}

bool
Symbolizer::State::outOfSync( const SymbolizerInput* dataSet ) const
{
    return dataSet && dataSet->outOfSync( _dataSetRevision );
}

/*************************************************************************/

//SymbolizerInput::SymbolizerInput()
//{
//}

//int
//SymbolizerInput::getRevision() const {
//    int rev = _revision;
//    if ( _alwaysDirty ) dirty();
//    return rev;
//}

//void
//SymbolizerInput::dirty() {
//    _revision++;
//}

//void 
//SymbolizerInput::inSyncWith( int otherRevision ) {
//    return revision == _revision;
//}

//void
//Symbolizer::sync( int& revision ) {
//    if ( !_alwaysDirty )
//        revision = _revision;
//}
//
//void SymbolizerInput::setAlwaysDirty( bool value ) {
//    _alwaysDirty = value;
//    if ( value ) dirty();
//}

