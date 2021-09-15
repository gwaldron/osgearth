/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/Color>
#include <osgEarth/Math>
#include "Random"
#include <algorithm>
#include <osg/Vec4ub>
#include <osgEarth/StringUtils>
#include <sstream>
#include <iomanip>
#include <ctype.h>

using namespace osgEarth;

namespace
{
    void rgb2hsv( osg::Vec4f& c )
    {
        float minval = osg::minimum( c.r(), osg::minimum( c.g(), c.b() ) );
        float maxval = osg::maximum( c.r(), osg::maximum( c.g(), c.b() ) );
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

    float hue2rgb(float v1, float v2, float vH)
    {
        if (vH < 0.0f) vH += 1.0f;
        if (vH > 1.0f) vH -= 1.0f;
        if ((6.0f * vH) < 1.0f) return (v1 + (v2 - v1) * 6.0f * vH);
        if ((2.0f * vH) < 1.0f) return (v2);
        if ((3.0f * vH) < 2.0f) return (v1 + (v2 - v1) * ((2.0f / 3.0f) - vH) * 6.0f);
        return (v1);
    }

    void hsl2rgb(osg::Vec4f& c)
    {
        float H = c.x();
        float S = c.y();
        float L = c.z();

        float R, G, B;
        if (S == 0)                       //HSL values = 0 - 1
        {
            R = L;                      //RGB results = 0 - 1
            G = L;
            B = L;
        }
        else
        {
            float var_2, var_1;
            if (L < 0.5)
                var_2 = L * (1 + S);
            else
                var_2 = (L + S) - (S * L);

            var_1 = 2 * L - var_2;

            R = hue2rgb(var_1, var_2, H + (1.0f / 3.0f));
            G = hue2rgb(var_1, var_2, H);
            B = hue2rgb(var_1, var_2, H - (1.0f / 3.0f));
        }
        c.r() = R;
        c.g() = G;
        c.b() = B;
    }
}

Color Color::White    ( 0xffffffff, Color::RGBA );
Color Color::Silver   ( 0xc0c0c0ff, Color::RGBA );
Color Color::Gray     ( 0x808080ff, Color::RGBA );
Color Color::Black    ( 0x000000ff, Color::RGBA );
Color Color::Red      ( 0xff0000ff, Color::RGBA );
Color Color::Maroon   ( 0x800000ff, Color::RGBA );
Color Color::Yellow   ( 0xffff00ff, Color::RGBA );
Color Color::Olive    ( 0x808000ff, Color::RGBA );
Color Color::Lime     ( 0x00ff00ff, Color::RGBA );
Color Color::Green    ( 0x008000ff, Color::RGBA );
Color Color::Aqua     ( 0x00ffffff, Color::RGBA );
Color Color::Teal     ( 0x008080ff, Color::RGBA );
Color Color::Blue     ( 0x0000ffff, Color::RGBA );
Color Color::Navy     ( 0x000080ff, Color::RGBA );
Color Color::Fuchsia  ( 0xff00ffff, Color::RGBA );
Color Color::Purple   ( 0x800080ff, Color::RGBA );
Color Color::Orange   ( 0xffa500ff, Color::RGBA );

Color Color::DarkGray ( 0x404040ff, Color::RGBA );
Color Color::Magenta  ( 0xc000c0ff, Color::RGBA );
Color Color::Cyan     ( 0x00ffffff, Color::RGBA );
Color Color::Brown    ( 0xaa5500ff, Color::RGBA );
Color Color::Transparent(0x00000000,Color::RGBA);

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
    std::string t = osgEarth::toLower(input);
    osgEarth::trim2(t);

    if (osgEarth::startsWith(t, "rgb("))
    {
        std::string sub = t.substr(4, t.size() - 5);
        StringTokenizer tok(",");
        StringVector components;
        tok.tokenize(sub, components);
        if (components.size() == 3)
        {
            unsigned int r = osgEarth::as<unsigned int>(components[0], 0u);
            unsigned int g = osgEarth::as<unsigned int>(components[1], 0u);
            unsigned int b = osgEarth::as<unsigned int>(components[2], 0u);
            set((float)r / 255.0f, (float)g / 255.0f, (float)b / 255.0f, 1.0f);
        }
    }
    else if (osgEarth::startsWith(t, "rgba("))
    {
        std::string sub = t.substr(5, t.size() - 6);
        StringTokenizer tok(",");
        StringVector components;
        tok.tokenize(sub, components);
        if (components.size() == 4)
        {
            unsigned int r = osgEarth::as<unsigned int>(components[0], 0u);
            unsigned int g = osgEarth::as<unsigned int>(components[1], 0u);
            unsigned int b = osgEarth::as<unsigned int>(components[2], 0u);
            unsigned int a = osgEarth::as<unsigned int>(components[3], 0u);
            set((float)r / 255.0f, (float)g / 255.0f, (float)b / 255.0f, (float)a);
        }
    }
    else if (osgEarth::startsWith(t, "hsl("))
    {
        std::string sub = t.substr(4, t.size() - 5);
        StringTokenizer tok(",");
        StringVector components;
        tok.tokenize(sub, components);
        if (components.size() == 3)
        {
            float H = osgEarth::as<float>(components[0], 0.0f);
            float S = 0.0f;
            if (osgEarth::endsWith(components[1], "%"))
            {
                std::string sub = components[1].substr(0, components[1].size() - 1);
                S = osgEarth::as<float>(sub, 0.0f);
            }
            else
            {
                S = osgEarth::as<float>(components[1], 0.0f);
            }
            float L = 0.0f;
            if (osgEarth::endsWith(components[2], "%"))
            {
                std::string sub = components[2].substr(0, components[2].size() - 1);
                L = osgEarth::as<float>(sub, 0.0f);
            }
            else
            {
                L = osgEarth::as<float>(components[2], 0.0f);
            }
            osg::Vec4 c(H / 255.0f, S / 100.0f, L / 100.0f, 1.0f);
            hsl2rgb(c);
            set(c.r(), c.g(), c.b(), c.a());
        }
    }
    else if (osgEarth::startsWith(t, "hsla("))
    {
        std::string sub = t.substr(5, t.size() - 6);
        StringTokenizer tok(",");
        StringVector components;
        tok.tokenize(sub, components);
        if (components.size() == 4)
        {
            float H = osgEarth::as<float>(components[0], 0.0f);
            float S = 0.0f;
            if (osgEarth::endsWith(components[1], "%"))
            {
                std::string sub = components[1].substr(0, components[1].size() - 1);
                S = osgEarth::as<float>(sub, 0.0f);
            }
            else
            {
                S = osgEarth::as<float>(components[1], 0.0f);
            }
            float L = 0.0f;
            if (osgEarth::endsWith(components[2], "%"))
            {
                std::string sub = components[2].substr(0, components[2].size() - 1);
                L = osgEarth::as<float>(sub, 0.0f);
            }
            else
            {
                L = osgEarth::as<float>(components[2], 0.0f);
            }
            float A = osgEarth::as<float>(components[3], 1.0f);
            osg::Vec4 c(H / 255.0f, S / 100.0f, L / 100.0f, A);
            hsl2rgb(c);
            set(c.r(), c.g(), c.b(), c.a());
        }
    }
    else
    {
        osg::Vec4ub c(0, 0, 0, 255);

        unsigned e =
            t.size() >= 2 && t[0] == '0' && t[1] == 'x' ? 2 :
            t.size() >= 1 && t[0] == '#' ? 1 :
            0;
        unsigned len = t.length() - e;
        if (len == 3)
        {
            // This is a 3 digit hex code, so turn it into a 6 digit hex code
            std::stringstream buf;
            buf << t[e + 0] << t[e + 0] << t[e + 1] << t[e + 1] << t[e + 2] << t[e + 2];
            t = buf.str();
            len = 6;
            e = 0;
        }

        if (len >= 6) {
            c.r() |= t[e + 0] <= '9' ? (t[e + 0] - '0') << 4 : (10 + (t[e + 0] - 'a')) << 4;
            c.r() |= t[e + 1] <= '9' ? (t[e + 1] - '0') : (10 + (t[e + 1] - 'a'));
            c.g() |= t[e + 2] <= '9' ? (t[e + 2] - '0') << 4 : (10 + (t[e + 2] - 'a')) << 4;
            c.g() |= t[e + 3] <= '9' ? (t[e + 3] - '0') : (10 + (t[e + 3] - 'a'));
            c.b() |= t[e + 4] <= '9' ? (t[e + 4] - '0') << 4 : (10 + (t[e + 4] - 'a')) << 4;
            c.b() |= t[e + 5] <= '9' ? (t[e + 5] - '0') : (10 + (t[e + 5] - 'a'));
            if (len >= 8) {
                c.a() = 0;
                c.a() |= t[e + 6] <= '9' ? (t[e + 6] - '0') << 4 : (10 + (t[e + 6] - 'a')) << 4;
                c.a() |= t[e + 7] <= '9' ? (t[e + 7] - '0') : (10 + (t[e + 7] - 'a'));
            }
        }

        float w = ((float)c.r()) / 255.0f;
        float x = ((float)c.g()) / 255.0f;
        float y = ((float)c.b()) / 255.0f;
        float z = ((float)c.a()) / 255.0f;

        if (format == RGBA)
            set(w, x, y, z);
        else // ABGR
            set(z, y, x, w);
    }
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

osg::Vec4f
Color::asHSL() const
{
    static const osg::Vec4f K(0.0f, -1.0f / 3.0f, 2.0f / 3.0f, -1.0f);
    osg::Vec4f A(b(), g(), K.w(), K.z());
    osg::Vec4f B(g(), b(), K.x(), K.y());
    osg::Vec4f p = mix(A, B, step(b(), g()));
    A.set(p.x(), p.y(), p.w(), r());
    B.set(r(), p.y(), p.z(), p.x());
    osg::Vec4f q = mix(A, B, step(p.x(), r()));
    float d = q.x() - std::min(q.w(), q.y());
    const float e = 1.0e-10;
    return osg::Vec4f(
        fabs(q.z() + (q.w() - q.y()) / (6.0f*d + e)),
        d / (q.x() + e),
        q.x(),
        a());
}

void
Color::fromHSL(const osg::Vec4f& hsl)
{
    set(hsl[0], hsl[1], hsl[2], a());
    float h = x(), s = y(), v = z();
    if (s == 0.0f) {
        set(1.0f, 1.0f, 1.0f, a());
    }
    else {
        float vh = h * 6.0f;
        float vi = floor(vh);
        float v1 = v * (1.0f - s);
        float v2 = v * (1.0f - s * (vh - vi));
        float v3 = v * (1.0f - s * (1.0f - (vh - vi)));
        float vr, vg, vb;
        if (vi == 0.0f) { vr = v, vg = v3, vb = v1; }
        else if (vi == 1.0f) { vr = v2, vg = v, vb = v1; }
        else if (vi == 2.0f) { vr = v1, vg = v, vb = v3; }
        else if (vi == 3.0f) { vr = v1, vg = v2, vb = v; }
        else if (vi == 4.0f) { vr = v3, vg = v1, vb = v; }
        else { vr = v, vg = v1, vb = v2; }
        set(vr, vg, vb, a());
    }
}

void
Color::createRandomColorRamp(
    unsigned count,
    std::vector<Color>& output,
    int seed)
{
    // Code is adapted from QGIS random color ramp feature,
    // which found the idea here (http://basecase.org/env/on-rainbows)
    // of adding the "golden ratio" to the hue angle in order to
    // minimize hue overlap and repetition.

    constexpr int hueMin = 0;
    constexpr int hueMax = 360;
    constexpr float satMin = 0.5f;
    constexpr float satMax = 1.0f;
    constexpr float valMin = 0.5f;
    constexpr float valMax = 1.0f;

    Random prng;
    if (seed >= 0)
        prng.seed(seed);
    
    double hueAngle = (double)prng.next(360);
    osg::Vec4f hsv(0, 0, 0, 1);

    for (unsigned i = 0; i < count; ++i)
    {
        hueAngle = fmod(hueAngle + 137.50776, 360.0);
        hsv[0] = hueAngle / 360.0;
        hsv[1] = satMin + (float)prng.next()*(satMax - satMin);
        hsv[2] = valMin + (float)prng.next()*(valMax - valMin);        
        hsv2rgb(hsv);
        output.push_back(Color(hsv));
    }
}
