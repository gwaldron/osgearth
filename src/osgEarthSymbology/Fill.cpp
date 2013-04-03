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
#include <osgEarthSymbology/Fill>

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

Fill::Fill()
{
    init();
}

Fill::Fill( float r, float g, float b, float a )
{
    init();
    _color.set( r, g, b, a );
}

Fill::Fill(const Color& color)
{
    init();
    _color = color;
}

Fill::Fill(const Config& conf )
{
    init();
    mergeConfig(conf);
}

Fill::Fill(const Fill& rhs)
{
    init();
    mergeConfig(rhs.getConfig());
}

void
Fill::init()
{
    _color.set( 1.0f, 1.0f, 1.0f, 1.0f );
}

Config
Fill::getConfig() const
{
    Config conf("fill");
    conf.add("color", _color.toHTML() );
    return conf;
}

void
Fill::mergeConfig( const Config& conf )
{
    _color = Color( conf.value("color") );
}
