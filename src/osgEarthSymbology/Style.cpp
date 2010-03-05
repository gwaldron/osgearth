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
#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/Geometry>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Symbology;

Style::Style()
{
    //NOP
}

void Style::setSymbol( Symbol* symbol )
{
    if (symbol)
        _symbols[symbol->getType()] = symbol;
}

Symbol* Style::getSymbol(Features::Geometry::Type type) const
{
    switch (type)
    {
    case Features::Geometry::TYPE_POINTSET:
        if (_symbols.find(Symbol::POINT) != _symbols.end())
            return _symbols.find(Symbol::POINT)->second.get();
    case Features::Geometry::TYPE_LINESTRING:
        if (_symbols.find(Symbol::LINESTRING) != _symbols.end())
            return _symbols.find(Symbol::LINESTRING)->second.get();
    case Features::Geometry::TYPE_RING:
        if (_symbols.find(Symbol::RING) != _symbols.end())
            return _symbols.find(Symbol::RING)->second.get();
    case Features::Geometry::TYPE_POLYGON:
        if (_symbols.find(Symbol::POLYGON) != _symbols.end())
            return _symbols.find(Symbol::POLYGON)->second.get();
    }
    return 0;
}
