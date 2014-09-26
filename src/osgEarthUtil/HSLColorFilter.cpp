/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthUtil/HSLColorFilter>
#include <osgEarth/VirtualProgram>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osg/Program>
#include <OpenThreads/Atomic>

using namespace osgEarth;
using namespace osgEarth::Util;

//---------------------------------------------------------------------------

namespace
{
    static OpenThreads::Atomic s_uniformNameGen;

    static const char* s_commonShaderSource =

        "#version 110 \n"

        "void oe_hsl_RGB_2_HSL(in float r, in float g, in float b, out float h, out float s, out float l)\n"
        "{ \n"
        "    float var_Min = min( r, min(g, b) );    //Min. value of RGB\n"
        "    float var_Max = max( r, max(g, b) );    //Max. value of RGB\n"
        "    float del_Max = var_Max - var_Min;      //Delta RGB value\n"
        "\n"
        "    l = ( var_Max + var_Min ) / 2.0;\n"
        "\n"
        "    if ( del_Max == 0.0 )                     //This is a gray, no chroma...\n"
        "    {\n"
        "        h = 0.0;                              //HSL results from 0 to 1\n"
        "        s = 0.0;\n"
        "    }\n"
        "    else                                      //Chromatic data...\n"
        "    {\n"
        "        if ( l < 0.5 ) s = del_Max / ( var_Max + var_Min );\n"
        "        else           s = del_Max / ( 2.0 - var_Max - var_Min );\n"
        "\n"
        "        float del_R = ( ( ( var_Max - r ) / 6.0 ) + ( del_Max / 2.0 ) ) / del_Max;\n"
        "        float del_G = ( ( ( var_Max - g ) / 6.0 ) + ( del_Max / 2.0 ) ) / del_Max;\n"
        "        float del_B = ( ( ( var_Max - b ) / 6.0 ) + ( del_Max / 2.0 ) ) / del_Max;\n"
        "        if      ( r == var_Max ) h = del_B - del_G;\n"
        "        else if ( g == var_Max ) h = ( 1.0 / 3.0 ) + del_R - del_B;\n"
        "        else if ( b == var_Max ) h = ( 2.0 / 3.0 ) + del_G - del_R;\n"
        "        if ( h < 0.0 ) h += 1.0;\n"
        "        if ( h > 1.0 ) h -= 1.0;\n"
        "    }\n"
        "}\n"

        "float oe_hsl_Hue_2_RGB(float v1, float v2, float vH )\n"
        "{\n"
        "    float ret;\n"
        "    if ( vH < 0.0 )\n"
        "        vH += 1.0;\n"
        "    if ( vH > 1.0 )\n"
        "        vH -= 1.0;\n"
        "    if ( ( 6.0 * vH ) < 1.0 )\n"
        "      ret = ( v1 + ( v2 - v1 ) * 6.0 * vH );\n"
        "    else if ( ( 2.0 * vH ) < 1.0 )\n"
        "        ret = ( v2 );\n"
        "    else if ( ( 3.0 * vH ) < 2.0 )\n"
        "        ret = ( v1 + ( v2 - v1 ) * ( ( 2.0 / 3.0 ) - vH ) * 6.0 );\n"
        "    else\n"
        "        ret = v1;\n"
        "    return ret;\n"
        "}\n"

        "void oe_hsl_HSL_2_RGB(in float h, in float s, in float l, out float r, out float g, out float b)\n"
        "{\n"
        "  float var_2, var_1;\n"
        "  if (s == 0.0)\n"
        "  {\n"
        "    r = l;\n"
        "    g = l;\n"
        "    b = l;\n"
        "  }\n"
        "  else\n"
        "  {\n"
        "    if ( l < 0.5 )\n"
        "    {\n"
        "      var_2 = l * ( 1.0 + s );\n"
        "    }\n"
        "    else\n"
        "    {\n"
        "      var_2 = ( l + s ) - ( s * l );\n"
        "    }\n"

        "    var_1 = 2.0 * l - var_2;\n"
        
        "    r = oe_hsl_Hue_2_RGB( var_1, var_2, h + ( 1.0 / 3.0 ) );\n"
        "    g = oe_hsl_Hue_2_RGB( var_1, var_2, h );\n"
        "    b = oe_hsl_Hue_2_RGB( var_1, var_2, h - ( 1.0 / 3.0 ) );\n"
        "  }\n"
        "}\n";


    static const char* s_localShaderSource =

        "#version 110\n"
        "void oe_hsl_RGB_2_HSL(in float r, in float g, in float b, out float h, out float s, out float l);\n"
        "void oe_hsl_HSL_2_RGB(in float h, in float s, in float l, out float r, out float g, out float b);\n"
        "uniform vec3 __UNIFORM_NAME__;\n"

        "void __ENTRY_POINT__(inout vec4 color)\n"
        "{ \n"
        "    if (__UNIFORM_NAME__.x != 0.0 || __UNIFORM_NAME__.y != 0.0 || __UNIFORM_NAME__.z != 0.0) \n"
        "    { \n"
        "        float h, s, l;\n"
        "        oe_hsl_RGB_2_HSL( color.r, color.g, color.b, h, s, l);\n"
        "        h += __UNIFORM_NAME__.x;\n"
        "        s += __UNIFORM_NAME__.y;\n"
        "        l += __UNIFORM_NAME__.z;\n"

        // clamp H,S and L to [0..1]

        "        h = clamp(h, 0.0, 1.0);\n"
        "        s = clamp(s, 0.0, 1.0);\n"
        "        l = clamp(l, 0.0, 1.0);\n"


        "        float r, g, b;\n"
        "        oe_hsl_HSL_2_RGB( h, s, l, r, g, b);\n"
        "        color.r = r;\n"
        "        color.g = g;\n"
        "        color.b = b;\n"
        "    }\n"
        "} \n";
}

//---------------------------------------------------------------------------

#define FUNCTION_PREFIX "osgearthutil_hslColorFilter_"
#define UNIFORM_PREFIX  "osgearthutil_u_hsl_"

namespace
{
    static Threading::Mutex          s_commonShaderMutex;
    static osg::ref_ptr<osg::Shader> s_commonShader;
}

//---------------------------------------------------------------------------

HSLColorFilter::HSLColorFilter()
{
    init();
}


void
HSLColorFilter::init()
{
    // Generate a unique name for this filter's uniform. This is necessary
    // so that each layer can have a unique uniform and entry point.
    _instanceId = (++s_uniformNameGen)-1;
    _hsl = new osg::Uniform( osg::Uniform::FLOAT_VEC3, Stringify() << UNIFORM_PREFIX << _instanceId );
    _hsl->set( osg::Vec3f(0.0f, 0.0f, 0.0f) );

    // Build the "common" shader code-- this is shader code that will be shared by multiple instances
    // of this filter (if there are multiple instances)
    if ( !s_commonShader.valid() )
    {
        Threading::ScopedMutexLock lock(s_commonShaderMutex);
        if ( !s_commonShader.valid() )
        {
            s_commonShader = new osg::Shader(osg::Shader::FRAGMENT, s_commonShaderSource);
        }
    }
}


void
HSLColorFilter::setHSLOffset( const osg::Vec3f& value )
{
    _hsl->set( value );
}


osg::Vec3f
HSLColorFilter::getHSLOffset() const
{
    osg::Vec3f value;
    _hsl->get( value );
    return value;
}


std::string
HSLColorFilter::getEntryPointFunctionName() const
{
    return Stringify() << FUNCTION_PREFIX << _instanceId;
}


void
HSLColorFilter::install( osg::StateSet* stateSet ) const
{
    // safe: won't add twice.
    stateSet->addUniform( _hsl.get() );

    VirtualProgram* vp = dynamic_cast<VirtualProgram*>(stateSet->getAttribute(VirtualProgram::SA_TYPE));
    if ( vp )
    {
        // safe: won't add the same pointer twice.
        vp->setShader( "osgEarthUtil::HSLColorFilter_common", s_commonShader.get() );

        // build the local shader (unique per instance). We'll use a template with search
        // and replace for this one.
        std::string entryPoint = Stringify() << FUNCTION_PREFIX << _instanceId;
        std::string code       = s_localShaderSource;
        replaceIn( code, "__UNIFORM_NAME__", _hsl->getName() );
        replaceIn( code, "__ENTRY_POINT__",  entryPoint );

        osg::Shader* main = new osg::Shader(osg::Shader::FRAGMENT, code);
        //main->setName(entryPoint);
        vp->setShader(entryPoint, main);
    }
}

//---------------------------------------------------------------------------

OSGEARTH_REGISTER_COLORFILTER( hsl, osgEarth::Util::HSLColorFilter );


HSLColorFilter::HSLColorFilter(const Config& conf)
{
    init();

    osg::Vec3f hsl;
    hsl[0] = conf.value("h", 0.0);
    hsl[1] = conf.value("s", 0.0);
    hsl[2] = conf.value("l", 0.0);
    setHSLOffset( hsl );
}

Config
HSLColorFilter::getConfig() const
{
    osg::Vec3f hsl = getHSLOffset();
    Config conf("hsl");
    conf.add( "h", hsl[0] );
    conf.add( "s", hsl[1] );
    conf.add( "l", hsl[2] );
    return conf;
}
