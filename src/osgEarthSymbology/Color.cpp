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
#include <osgEarthSymbology/Color>
#include <algorithm>
#include <osg/Vec4ub>
#include <sstream>
#include <iomanip>
#include <ctype.h>

using namespace osgEarth::Symbology;

namespace
{
    void rgb2hsv( osg::Vec4f& c )
    {
        float minval = std::min( c.r(), std::min( c.g(), c.b() ) );
        float maxval = std::max( c.r(), std::max( c.g(), c.b() ) );
        float delta = maxval - minval;
        float h = 0.0f, s = 0.0, v = maxval;
        if ( delta != 0.0f )
        {
            s = delta / maxval;
            float dr = (((maxval-c.r())/6.0f)+(delta/2.0f))/delta;
            float dg = (((maxval-c.g())/6.0f)+(delta/2.0f))/delta;
            float db = (((maxval-c.b())/6.0f)+(delta/2.0f))/delta;
            if ( c.r() == maxval ) h = db - dg;
            else if ( c.g() == maxval ) h = (1.0f/3.0f)+dr-db;
            else if ( c.b() == maxval ) h = (2.0f/3.0f)+dg-dr;
            if ( h < 0.0f ) h += 1.0f;
            if ( h > 1.0f ) h -= 1.0f;
        }
        c.set( h, s, v, c.a() );
    }

    void hsv2rgb( osg::Vec4f& c )
    {
        float h = c[0], s = c[1], v = c[2];
        if ( s == 0.0f ) {
            c.r() = c.g() = c.b() = 1.0f;
        }
        else {
            float vh = h*6.0f;
            float vi = floor(vh);
            float v1 = v * (1.0f - s);
            float v2 = v * (1.0f - s * (vh-vi));
            float v3 = v * (1.0f - s * (1.0f - (vh-vi)));
            float vr, vg, vb;
            if ( vi == 0.0f )      { vr = v,  vg = v3, vb = v1; }
            else if ( vi == 1.0f ) { vr = v2, vg = v,  vb = v1; }
            else if ( vi == 2.0f ) { vr = v1, vg = v,  vb = v3; }
            else if ( vi == 3.0f ) { vr = v1, vg = v2, vb = v; }
            else if ( vi == 4.0f ) { vr = v3, vg = v1, vb = v; }
            else                   { vr = v,  vg = v1, vb = v2; }
            c.set( vr, vg, vb, c.a() );
        }
    }
}

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
Color Color::Magenta  ( 0xc000c0ff );
Color Color::Cyan     ( 0x00ffffff );
Color Color::Brown    ( 0xaa5500ff );

Color::Color( unsigned v, Format format )
{
    if ( format == RGBA )
    {
        set(
            (float)(v>>24)/255.0f, 
            (float)((v&0xFF0000)>>16)/255.0f, 
            (float)((v&0xFF00)>>8)/255.0f,
            (float)(v&0xFF)/255.0f );
    }
    else // format == ABGR
    {
        set(
            (float)(v&0xFF)/255.0f,
            (float)((v&0xFF00)>>8)/255.0f,
            (float)((v&0xFF0000)>>16)/255.0f, 
            (float)(v>>24)/255.0f );
    }
}

Color::Color( const Color& rhs, float a ) :
osg::Vec4f( rhs )
{
    (*this)[3] = a;
}

/** Parses a hex color string ("#rrggbb", "#rrggbbaa", "0xrrggbb", etc.) into an OSG color. */
Color::Color( const std::string& input, Format format )
{
    std::string t = input;
    std::transform( t.begin(), t.end(), t.begin(), ::tolower );
    osg::Vec4ub c(0,0,0,255);

    unsigned e = 
        t.size() >= 2 && t[0] == '0' && t[1] == 'x' ? 2 :
        t.size() >= 1 && t[0] == '#' ? 1 :
        0;
    unsigned len = t.length() - e;

    if ( len >= 6 ) {
        c.r() |= t[e+0]<='9' ? (t[e+0]-'0')<<4 : (10+(t[e+0]-'a'))<<4;
        c.r() |= t[e+1]<='9' ? (t[e+1]-'0')    : (10+(t[e+1]-'a'));
        c.g() |= t[e+2]<='9' ? (t[e+2]-'0')<<4 : (10+(t[e+2]-'a'))<<4;
        c.g() |= t[e+3]<='9' ? (t[e+3]-'0')    : (10+(t[e+3]-'a'));
        c.b() |= t[e+4]<='9' ? (t[e+4]-'0')<<4 : (10+(t[e+4]-'a'))<<4;
        c.b() |= t[e+5]<='9' ? (t[e+5]-'0')    : (10+(t[e+5]-'a'));
        if ( t.length() >= 8 ) {
            c.a() = 0;
            c.a() |= t[e+6]<='9' ? (t[e+6]-'0')<<4 : (10+(t[e+6]-'a'))<<4;
            c.a() |= t[e+7]<='9' ? (t[e+7]-'0')    : (10+(t[e+7]-'a'));
        }
    }
    float w = ((float)c.r())/255.0f;
    float x = ((float)c.g())/255.0f;
    float y = ((float)c.b())/255.0f;
    float z = ((float)c.a())/255.0f;

    if ( format == RGBA )
        set( w, x, y, z );
    else // ABGR
        set( z, y, x, w );
}

/** Makes an HTML color ("#rrggbb" or "#rrggbbaa") from an OSG color. */
std::string
Color::toHTML( Format format ) const
{
    float w, x, y, z;
    if ( format == RGBA ) {
        w = r(), x = g(), y = b(), z = a();
    }
    else { // ABGR
        w = a(), x = b(), y = g(), z = r();
    }

    return Stringify()
        << "#"
        << std::hex << std::setw(2) << std::setfill('0') << (int)(w*255.0f)
        << std::hex << std::setw(2) << std::setfill('0') << (int)(x*255.0f)
        << std::hex << std::setw(2) << std::setfill('0') << (int)(y*255.0f)
        << std::hex << std::setw(2) << std::setfill('0') << (int)(z*255.0f);
}

Color
Color::brightness( float perc ) const
{
    return Color(r()*perc, g()*perc, b()*perc, a());
}

unsigned
Color::as( Format format ) const
{
    if ( format == RGBA )
    {
        return 
            (((unsigned)(r()*255.0)) << 24) |
            (((unsigned)(g()*255.0)) << 16) |
            (((unsigned)(b()*255.0)) << 8 ) |
            (((unsigned)(a()*255.0)));
    }
    else // format == ABGR
    {
        return 
            (((unsigned)(a()*255.0)) << 24) |
            (((unsigned)(b()*255.0)) << 16) |
            (((unsigned)(g()*255.0)) << 8 ) |
            (((unsigned)(r()*255.0)));
    }
}
