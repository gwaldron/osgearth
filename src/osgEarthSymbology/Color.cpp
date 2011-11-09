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
            else if ( vi == 3.0f ) { vr = v1, vb = v2, vb = v; }
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
Color Color::Cyan     ( 0x00ffffff );

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
Color::Color( const std::string& html, Format format )
{
    std::string t = html;
    std::transform( t.begin(), t.end(), t.begin(), ::tolower );
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

    std::stringstream buf;
    buf << "#";
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(w*255.0f);
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(x*255.0f);
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(y*255.0f);
    buf << std::hex << std::setw(2) << std::setfill('0') << (int)(z*255.0f);
    std::string ssStr = buf.str();
    return ssStr;
}

Color
Color::brightness( float perc ) const
{
    Color c( *this );
    rgb2hsv( c );
    c.b() = osg::clampBetween( perc * c.b(), 0.0f, 1.0f );
    hsv2rgb( c );
    return c;
}
