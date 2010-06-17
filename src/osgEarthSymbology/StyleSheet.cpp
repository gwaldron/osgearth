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
#include <osgEarthSymbology/StyleSheet>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

StyleSheet::StyleSheet()
{
    //NOP
}

void 
StyleSheet::addStyle( const std::string& name, Style* style )
{
    _styles[name] = style;
}

Style* 
StyleSheet::getStyle( const std::string& name )
{
    NameStyleMap::iterator i = _styles.find( name );
    return i != _styles.end() ? i->second.get() : 0L;
}

const Style* 
StyleSheet::getStyle( const std::string& name ) const
{
    NameStyleMap::const_iterator i = _styles.find( name );
    return i != _styles.end() ? i->second.get() : 0L;
}
