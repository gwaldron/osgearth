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
#include <osgEarth/ShaderFactory>

#include <osgEarth/ShaderUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osg/Shader>
#include <osg/Program>
#include <osg/State>
#include <osg/Notify>
#include <sstream>

#define LC "[ShaderFactory] "

#ifdef OSG_GLES2_AVAILABLE
    static bool s_GLES_SHADERS = true;
#else
    static bool s_GLES_SHADERS = false;
#endif

#define INDENT "    "


using namespace osgEarth;
using namespace osgEarth::ShaderComp;


std::string
ShaderFactory::getSamplerName( unsigned unit ) const
{
    return Stringify() << "osgearth_tex" << unit;
}


osg::Shader*
ShaderFactory::createVertexShaderMain(const FunctionLocationMap& functions) const
{
    // collect the "model" stage functions:
    FunctionLocationMap::const_iterator i = functions.find( LOCATION_VERTEX_MODEL );
    const OrderedFunctionMap* modelStage = i != functions.end() ? &i->second : 0L;

    // collect the "view" stage functions:
    FunctionLocationMap::const_iterator j = functions.find( LOCATION_VERTEX_VIEW );
    const OrderedFunctionMap* viewStage = j != functions.end() ? &j->second : 0L;

    // collect the "clip" stage functions:
    FunctionLocationMap::const_iterator k = functions.find( LOCATION_VERTEX_CLIP );
    const OrderedFunctionMap* clipStage = k != functions.end() ? &k->second : 0L;

    // header:
    std::stringstream buf;
    buf << 
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n";

    // prototypes for model stage methods:
    if ( modelStage )
    {
        for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
            buf << "void " << i->second << "(inout vec4 VertexMODEL); \n";
    }

    // prototypes for view stage methods:
    if ( viewStage )
    {
        for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            buf << "void " << i->second << "(inout vec4 VertexVIEW); \n";
    }

    // prototypes for clip stage methods:
    if ( clipStage )
    {
        for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            buf << "void " << i->second << "(inout vec4 VertexCLIP); \n";
    }

    // main:
    buf <<
        "varying vec4 osg_FrontColor; \n"
        "varying vec3 oe_Normal; \n"
        "void main(void) \n"
        "{ \n"
        INDENT "osg_FrontColor = gl_Color; \n"
        INDENT "vec4 vertex = gl_Vertex; \n";

    // call Model stage methods.
    if ( modelStage )
    {
        buf << INDENT "oe_Normal = gl_Normal; \n";

        for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
        {
            buf << INDENT << i->second << "(vertex); \n";
        }

        buf << INDENT << "oe_Normal = normalize(gl_NormalMatrix * oe_Normal); \n";
    }
    else
    {
        buf << INDENT << "oe_Normal = normalize(gl_NormalMatrix * gl_Normal); \n";
    }

    // call View stage methods.
    if ( viewStage )
    {
        buf << INDENT "vertex = gl_ModelViewMatrix * vertex; \n";

        for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
        {
            buf << INDENT << i->second << "(vertex); \n";
        }
    }

    // call Clip stage methods.
    if ( clipStage )
    {
        if ( viewStage )
        {
            buf << INDENT "vertex = gl_ProjectionMatrix * vertex; \n";
        }
        else
        {
            buf << INDENT "vertex = gl_ModelViewProjectionMatrix * vertex; \n";
        }

        for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
        {
            buf << INDENT << i->second << "(vertex); \n";
        }
    }

    // finally, emit the position vertex.
    if ( clipStage )
    {
        buf << INDENT "gl_Position = vertex; \n";
    }
    else if ( viewStage )
    {
        buf << INDENT "gl_Position = gl_ProjectionMatrix * vertex; \n";
    }
    else
    {
        buf << INDENT "gl_Position = gl_ModelViewProjectionMatrix * vertex; \n";
    }

    buf << "} \n";

    std::string str;
    str = buf.str();
    osg::Shader* shader = new osg::Shader( osg::Shader::VERTEX, str );
    shader->setName( "main(vert)" );
    return shader;
}


osg::Shader*
ShaderFactory::createFragmentShaderMain(const FunctionLocationMap& functions) const
{
    FunctionLocationMap::const_iterator i = functions.find( LOCATION_FRAGMENT_COLORING );
    const OrderedFunctionMap* coloring = i != functions.end() ? &i->second : 0L;

    FunctionLocationMap::const_iterator j = functions.find( LOCATION_FRAGMENT_LIGHTING );
    const OrderedFunctionMap* lighting = j != functions.end() ? &j->second : 0L;

    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n"
        << GLSL_DEFAULT_PRECISION_FLOAT << "\n";

    if ( coloring )
    {
        for( OrderedFunctionMap::const_iterator i = coloring->begin(); i != coloring->end(); ++i )
            buf << "void " << i->second << "( inout vec4 color ); \n";
    }

    if ( lighting )
    {
        for( OrderedFunctionMap::const_iterator i = lighting->begin(); i != lighting->end(); ++i )
            buf << "void " << i->second << "( inout vec4 color ); \n";
    }

    buf << 
        "varying vec4 osg_FrontColor; \n"
        "void main(void) \n"
        "{ \n"
        INDENT "vec4 color = osg_FrontColor; \n";

    if ( coloring )
    {
        for( OrderedFunctionMap::const_iterator i = coloring->begin(); i != coloring->end(); ++i )
            buf << INDENT << i->second << "( color ); \n";
    }

    if ( lighting )
    {
        for( OrderedFunctionMap::const_iterator i = lighting->begin(); i != lighting->end(); ++i )
            buf << INDENT << i->second << "( color ); \n";
    }

    buf << 
        INDENT "gl_FragColor = color; \n"
        "} \n";  

    std::string str;
    str = buf.str();
    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
    shader->setName( "main(frag)" );
    return shader;
}


void
ShaderFactory::installLightingShaders(VirtualProgram* vp) const
{
    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec4 oe_lighting_adjustment; \n"
        "varying vec4 oe_lighting_zero_vec; \n"
        "varying vec3 oe_Normal; \n"

        "void oe_lighting_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    oe_lighting_adjustment = vec4(1.0); \n"
        "    if (oe_mode_GL_LIGHTING) \n"
        "    { \n"
        "        vec3 N = oe_Normal; \n" //normalize(gl_NormalMatrix * gl_Normal); \n"
        "        float NdotL = dot( N, normalize(gl_LightSource[0].position.xyz) ); \n"
        "        NdotL = max( 0.0, NdotL ); \n"

        // NOTE: See comment in the fragment shader below for an explanation of
        //       this oe_zero_vec value.
        "        oe_lighting_zero_vec = vec4(0.0); \n"

        "        vec4 adj = \n"
        "            gl_FrontLightProduct[0].ambient + \n"
        "            gl_FrontLightProduct[0].diffuse * NdotL; \n"
        "        oe_lighting_adjustment = clamp( adj, 0.0, 1.0 ); \n"
        "    } \n"
        "} \n";

    const char* fs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "varying vec4 oe_lighting_adjustment; \n"
        "varying vec4 oe_lighting_zero_vec; \n"

         "uniform bool oe_mode_GL_LIGHTING; \n"
         "void oe_lighting_fragment( inout vec4 color ) \n"
         "{ \n"
         //NOTE: The follow was changed from the single line
         //      "color *= oe_lighting_adjustment" to the current code to fix
         //      an issue on iOS devices.  Adding a varying vec4 value set to
         //      (0.0,0.0,0.0,0.0) to the color should not make a difference,
         //      but it is part of the solution to the issue we were seeing.
         //      Without it and the additional lines of code, the globe was
         //      rendering textureless (just a white surface with lighting).
         "    if ( oe_mode_GL_LIGHTING ) \n"
         "    { \n"
         "        float alpha = color.a; \n"
         "        color = color * oe_lighting_adjustment + oe_lighting_zero_vec; \n"
         "        color.a = alpha; \n"
         "    } \n"
        "} \n";

    vp->setFunction( "oe_lighting_vertex",   vs, ShaderComp::LOCATION_VERTEX_VIEW, 0.0 );
    vp->setFunction( "oe_lighting_fragment", fs, ShaderComp::LOCATION_FRAGMENT_LIGHTING, 0.0 );
}


osg::Shader*
ShaderFactory::createColorFilterChainFragmentShader(const std::string&      function, 
                                                    const ColorFilterChain& chain ) const
{
    std::stringstream buf;
    buf << 
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n";

    // write out the shader function prototypes:
    for( ColorFilterChain::const_iterator i = chain.begin(); i != chain.end(); ++i )
    {
        ColorFilter* filter = i->get();
        buf << "void " << filter->getEntryPointFunctionName() << "(inout vec4 color);\n";
    }

    // write out the main function:
    buf << "void " << function << "(inout vec4 color) \n"
        << "{ \n";

    // write out the function calls. if there are none, it's a NOP.
    for( ColorFilterChain::const_iterator i = chain.begin(); i != chain.end(); ++i )
    {
        ColorFilter* filter = i->get();
        buf << INDENT << filter->getEntryPointFunctionName() << "(color);\n";
    }
        
    buf << "} \n";

    std::string bufstr;
    bufstr = buf.str();
    return new osg::Shader(osg::Shader::FRAGMENT, bufstr);
}


osg::Uniform*
ShaderFactory::createUniformForGLMode(osg::StateAttribute::GLMode      mode,
                                      osg::StateAttribute::GLModeValue value) const
{
    osg::Uniform* u = 0L;

    if ( mode == GL_LIGHTING )
    {
        u = new osg::Uniform(osg::Uniform::BOOL, "oe_mode_GL_LIGHTING");
        u->set( (value & osg::StateAttribute::ON) != 0 );
    }

    return u;
}
