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
#define RANGE  osgEarth::Registry::instance()->shaderFactory()->getRangeUniformName()

using namespace osgEarth;
using namespace osgEarth::ShaderComp;


namespace
{
    void insertRangeConditionals(const Function& f, std::ostream& buf)
    {
        if ( f._minRange.isSet() && !f._maxRange.isSet() )
        {
            buf << INDENT << "if (" << RANGE << " >= float(" << f._minRange.value() << "))\n" << INDENT;
        }
        else if ( !f._minRange.isSet() && f._maxRange.isSet() )
        {
            buf << INDENT << "if (" << RANGE << " <= float(" << f._maxRange.value() << "))\n" << INDENT;
        }
        else if ( f._minRange.isSet() && f._maxRange.isSet() )
        {
            buf << INDENT << "if (" << RANGE << " >= float(" << f._minRange.value() << ") && " << RANGE << " <= float(" << f._maxRange.value() << "))\n" << INDENT;
        }
    }
}


ShaderFactory::ShaderFactory()
{
    _fragStageOrder = FRAGMENT_STAGE_ORDER_COLORING_LIGHTING;
}



bool
ShaderFactory::createMains(const FunctionLocationMap&                functions,
                           std::vector< osg::ref_ptr<osg::Shader> >& out_shaders) const
{
    FunctionLocationMap::const_iterator f;

    // collect the "model" stage vertex functions:
    f = functions.find( LOCATION_VERTEX_MODEL );
    const OrderedFunctionMap* modelStage = f != functions.end() ? &f->second : 0L;

    // collect the "view" stage vertex functions:
    f = functions.find( LOCATION_VERTEX_VIEW );
    const OrderedFunctionMap* viewStage = f != functions.end() ? &f->second : 0L;

    // geometry shader functions:
    f = functions.find( LOCATION_VERTEX_GEOMETRY );
    const OrderedFunctionMap* geomStage = f != functions.end() ? &f->second : 0L;

    // geometry shader functions:
    f = functions.find( LOCATION_VERTEX_TESSELLATION_CONTROL );
    const OrderedFunctionMap* tessStage = f != functions.end() ? &f->second : 0L;

    // geometry shader functions:
    f = functions.find( LOCATION_VERTEX_TESSELLATION_EVAL );
    const OrderedFunctionMap* tessEvalStage = f != functions.end() ? &f->second : 0L;

    // collect the "clip" stage functions:
    f = functions.find( LOCATION_VERTEX_CLIP );
    const OrderedFunctionMap* clipStage = f != functions.end() ? &f->second : 0L;

    // fragment shader coloring functions:
    f = functions.find( LOCATION_FRAGMENT_COLORING );
    const OrderedFunctionMap* coloringStage = f != functions.end() ? &f->second : 0L;

    // fragment shader lighting functions:
    f = functions.find( LOCATION_FRAGMENT_LIGHTING );
    const OrderedFunctionMap* lightingStage = f != functions.end() ? &f->second : 0L;

    // what do we need to build?
    bool hasVertShader     = modelStage || viewStage || (clipStage && !geomStage && !tessEvalStage);
    bool hasGeomShader     = geomStage;
    bool hasTessShader     = tessStage;
    bool hasTessEvalShader = tessEvalStage;
    bool hasFragShader     = coloringStage || lightingStage;
    
    // where to insert the clip stage functions:
    bool clipStageInVertexShader   = !geomStage && !tessEvalStage;
    bool clipStageInGeomShader     = geomStage && !tessEvalStage;
    bool clipStageInTessEvalShader = tessEvalStage && !geomStage;

    bool useInterfaceBlocks = false;

    std::string versionHeader =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT;

    std::string vertGlobals =
        "// osgEarth vertex stage globals:\n"
        "vec4 oe_VertexModel, oe_VertexModel, oe_VertexClip; \n"
        "vec3 oe_NormalModel, oe_NormalView; \n"
        "vec4 oe_Color;";


    // Build the vertex shader.
    if ( hasVertShader )
    {
        std::stringstream buf;

        buf << versionHeader << "\n"
            << vertGlobals   << "\n";

        // prototype functions:
        if ( modelStage )
        {
            for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4 VertexMODEL); \n";
            }
        }

        // prototypes for view stage methods:
        if ( viewStage )
        {
            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4 VertexVIEW); \n";
            }
        }

        // prototypes for clip stage methods:
        if ( clipStage && clipStageInVertexShader )
        {
            for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4 VertexCLIP); \n";
            }
        }

        if ( hasGeomShader || hasTessShader || hasFragShader )
        {
            buf << "out vec4 oe_FrontColor_vertOut; \n"
        }

        if ( hasGeomShader || hasTessShader )
        {
            buf << "out vec3 oe_NormalView_vertOut; \n";
        }


        buf <<
            "void main(void) \n"
            "{ \n"
            INDENT "oe_FrontColor_vertOut = gl_Color; \n"
            INDENT "oe_VertexModel = gl_Vertex; \n"
            INDENT "oe_NormalModel = gl_Normal; \n";

        if ( modelStage )
        {
            buf <<
                INDENT "oe_Normal = oe_NormalModel; // backwards compatibility \n";

            for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
            {
                //insertRangeConditionals( i->second, buf );
                buf << INDENT << i->second._name << "(oe_VertexModel); \n"
            }

            buf << INDENT
                << "oe_NormalView = normalize(gl_NormalMatrix * oe_NormalModel; \n";
        }
        else
        {
            buf <<
                INDENT << "oe_VertexView = gl_ModelViewMatrix * oe_VertexModel; \n"
                INDENT << "oe_NormalView = gl_NormalMatrix * oe_NormalModel; \n"
        }

        // call View stage methods.
        if ( viewStage )
        {
            buf << INDENT "vertex = gl_ModelViewMatrix * vertex; \n";

            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(oe_VertexView); \n";
            }
        }


        ///left off here........



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
                insertRangeConditionals( i->second, buf );
                buf << INDENT << i->second._name << "(vertex); \n";
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
        osg::Shader* vertexShader = new osg::Shader( osg::Shader::VERTEX, str );
        vertexShader->setName( "main(vertex)" );
        out_shaders.push_back( vertexShader );
    }
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
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float " << RANGE << ";\n";

    // prototypes for model stage methods:
    if ( modelStage )
    {
        for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
        {
            buf << "void " << i->second._name << "(inout vec4 VertexMODEL); \n";
        }
    }

    // prototypes for view stage methods:
    if ( viewStage )
    {
        for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
        {
            buf << "void " << i->second._name << "(inout vec4 VertexVIEW); \n";
        }
    }

    // prototypes for clip stage methods:
    if ( clipStage )
    {
        for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
        {
            buf << "void " << i->second._name << "(inout vec4 VertexCLIP); \n";
        }
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
            insertRangeConditionals( i->second, buf );
            buf << INDENT << i->second._name << "(vertex); \n";
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
            insertRangeConditionals( i->second, buf );
            buf << INDENT << i->second._name << "(vertex); \n";
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
            insertRangeConditionals( i->second, buf );
            buf << INDENT << i->second._name << "(vertex); \n";
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

    FunctionLocationMap::const_iterator k = functions.find( LOCATION_FRAGMENT_OUTPUT );
    const OrderedFunctionMap* output = k != functions.end() ? &k->second : 0L;

    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n"
        << GLSL_DEFAULT_PRECISION_FLOAT << "\n"
        << "uniform float " << RANGE << ";\n";

    if ( coloring )
    {
        for( OrderedFunctionMap::const_iterator i = coloring->begin(); i != coloring->end(); ++i )
        {
            buf << "void " << i->second._name << "( inout vec4 color ); \n";
        }
    }

    if ( lighting )
    {
        for( OrderedFunctionMap::const_iterator i = lighting->begin(); i != lighting->end(); ++i )
        {
            buf << "void " << i->second._name << "( inout vec4 color ); \n";
        }
    }

    if ( output )
    {
        for( OrderedFunctionMap::const_iterator i = output->begin(); i != output->end(); ++i )
        {
            buf << "void " << i->second._name << "( inout vec4 color ); \n";
        }
    }

    buf << 
        "varying vec4 osg_FrontColor; \n"
        "varying vec3 oe_Normal; \n"
        "vec3 oe_global_Normal; \n" // stage-global
        "void main(void) \n"
        "{ \n"
        INDENT "vec4 color = osg_FrontColor; \n"
        INDENT "oe_global_Normal = normalize(oe_Normal); \n";

    int coloringPass = _fragStageOrder == FRAGMENT_STAGE_ORDER_COLORING_LIGHTING ? 0 : 1;
    int lightingPass = 1-coloringPass;

    for(int pass=0; pass<2; ++pass)
    {
        if ( coloring && (pass == coloringPass) )
        {
            for( OrderedFunctionMap::const_iterator i = coloring->begin(); i != coloring->end(); ++i )
            {
                insertRangeConditionals( i->second, buf );
                buf << INDENT << i->second._name << "( color ); \n";
            }
        }

        if ( lighting && (pass == lightingPass) )
        {
            for( OrderedFunctionMap::const_iterator i = lighting->begin(); i != lighting->end(); ++i )
            {
                insertRangeConditionals( i->second, buf );
                buf << INDENT << i->second._name << "( color ); \n";
            }
        }
    }

    if ( output )
    {
        for( OrderedFunctionMap::const_iterator i = output->begin(); i != output->end(); ++i )
        {
            insertRangeConditionals( i->second, buf );
            buf << INDENT << i->second._name << "( color ); \n";
        }
    }
    else
    {
        // in the absense of any output functions, generate a default output statement
        // that simply writes to gl_FragColor.
        buf << INDENT "gl_FragColor = color;\n";
    }
    buf << "}\n";

    std::string str;
    str = buf.str();
    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
    shader->setName( "main(frag)" );
    return shader;
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

std::string
ShaderFactory::getRangeUniformName() const
{
    return "oe_range_to_bs";
}

osg::Uniform*
ShaderFactory::createRangeUniform() const
{
    return new osg::Uniform(osg::Uniform::FLOAT, getRangeUniformName());
}
