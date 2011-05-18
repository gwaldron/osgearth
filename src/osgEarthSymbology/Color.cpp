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
#include <osgEarthSymbology/Color>
#include <algorithm>
#include <osg/Vec4ub>
#include <sstream>
#include <iomanip>
#include <cctype>

using namespace osgEarth::Symbology;

Color Color::White    ( 0xffffffff );
Color Color::Silver   ( 0xc0c0c0ff );
Color Color::Gray     ( 0x808080ff );
Color Color::Black    ( 0x000000ff );
Color Color::Red      ( 0xff0000ff );
Color Color::Maroon   ( 0x800000ff );
Color Color::Yellow   ( 0xffff00ff );
Color Color::Olive    ( 0x808000ff );
Color Color::Lime     ( 0x00ff00ff );
Color Color::Green    ( 0x008000ff );
Color Color::Aqua     ( 0x00ffffff );
Color Color::Teal     ( 0x008080ff );
Color Color::Blue     ( 0x0000ffff );
Color Color::Navy     ( 0x000080ff );
Color Color::Fuchsia  ( 0xff00ffff );
Color Color::Purple   ( 0x800080ff );
Color Color::Orange   ( 0xffa500ff );

Color Color::DarkGray ( 0x404040ff );

Color::Color( unsigned rgba )
{
    set(
        (float)(rgba>>24)/255.0f, 
        (float)((rgba&0xFF0000)>>16)/255.0f, 
        (float)((rgba&0xFF00)>>8)/255.0f,
        (float)(rgba&0xFF)/255.0f );
}

Color::Color( const Color& rhs, float a ) :
osg::Vec4f( rhs )
{
    (*this)[3] = a;
}

/** Parses an HTML color ("#rrggbb" or "#rrggbbaa") into an OSG color. */
Color::Color( const std::string& html )
{
    std::string t = html;
    std::transform( t.begin(), t.end(), t.begin(), std::tolower );
    osg::Vec4ub c(0,0,0,255);
    if ( t.length() >= 7 ) {
        c.r() |= t[1]<='9' ? (t[1]-'0')<<4 : (10+(t[1]-'a'))<<4;
        c.r() |= t[2]<='9' ? (t[2]-'0')    : (10+(t[2]-'a'));
        c.g() |= t[3]<='9' ? (t[3]-'0')<<4 : (10+(t[3]-'a'))<<4;
        c.g() |= t[4]<='9' ? (t[4]-'0')    : (10+(t[4]-'a'));
        c.b() |= t[5]<='9' ? (t[5]-'0')<<4 : (10+(t[5]-'a'))<<4;
        c.b() |= t[6]<='9' ? (t[6]-'0')    : (10+(t[6]-'a'));
        if ( t.length() == 9 ) {
            c.a() = 0;
            c.a() |= t[7]<='9' ? (t[7]-'0')<<4 : (10+(t[7]-'a'))<<4;
            c.a() |= t[8]<='9' ? (t[8]-'0')    : (10+(t[8]-'a'));
        }
    }
    set( ((float)c.r())/255.0f, ((float)c.g())/255.0f, ((float)c.b())/255.0f, ((float)c.a())/255.0f );
}

/** Makes an HTML color ("#rrggbb" or "#rrggbbaa") from an OSG color. */
std::string
Color::toHTML() const
{
    std::stringstream buf;
    buf << "#";
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(r()*255.0f);
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(g()*255.0f);
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(b()*255.0f);
    if ( a() < 1.0f )
        buf << std::hex << std::setw(2) << std::setfill('0') << (int)(a()*255.0f);
    std::string ssStr = buf.str();
    return ssStr;
}