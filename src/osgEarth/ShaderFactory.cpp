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
#include <osgEarth/ShaderLoader>
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


#define SPACE_MODEL 0
#define SPACE_VIEW  1
#define SPACE_CLIP  2


ShaderComp::StageMask
ShaderFactory::createMains(const ShaderComp::FunctionLocationMap&    functions,
                           const VirtualProgram::ShaderMap&          in_shaders,
                           std::vector< osg::ref_ptr<osg::Shader> >& out_shaders) const
{
    StageMask stages =
        ShaderComp::STAGE_VERTEX |
        ShaderComp::STAGE_FRAGMENT;

    FunctionLocationMap::const_iterator f;

    // collect the "model" stage vertex functions:
    f = functions.find( LOCATION_VERTEX_MODEL );
    const OrderedFunctionMap* modelStage = f != functions.end() ? &f->second : 0L;

    // collect the "view" stage vertex functions:
    f = functions.find( LOCATION_VERTEX_VIEW );
    const OrderedFunctionMap* viewStage = f != functions.end() ? &f->second : 0L;

    // geometry shader functions:
    f = functions.find( LOCATION_TESS_CONTROL );
    const OrderedFunctionMap* tessStage = f != functions.end() ? &f->second : 0L;

    // geometry shader functions:
    f = functions.find( LOCATION_TESS_EVALUATION );
    const OrderedFunctionMap* tessEvalStage = f != functions.end() ? &f->second : 0L;

    // geometry shader functions:
    f = functions.find( LOCATION_GEOMETRY );
    const OrderedFunctionMap* geomStage = f != functions.end() ? &f->second : 0L;

    // collect the "clip" stage functions:
    f = functions.find( LOCATION_VERTEX_CLIP );
    const OrderedFunctionMap* clipStage = f != functions.end() ? &f->second : 0L;

    // fragment shader coloring functions:
    f = functions.find( LOCATION_FRAGMENT_COLORING );
    const OrderedFunctionMap* coloringStage = f != functions.end() ? &f->second : 0L;

    // fragment shader lighting functions:
    f = functions.find( LOCATION_FRAGMENT_LIGHTING );
    const OrderedFunctionMap* lightingStage = f != functions.end() ? &f->second : 0L;

    // fragment shader lighting functions:
    f = functions.find( LOCATION_FRAGMENT_OUTPUT );
    const OrderedFunctionMap* outputStage = f != functions.end() ? &f->second : 0L;

    // what do we need to build?
    bool hasGeomShader     = geomStage     && !geomStage->empty();
    bool hasTessShader     = tessStage     && !tessStage->empty();
    bool hasTessEvalShader = tessEvalStage && !tessEvalStage->empty();
    bool hasFragShader     = true;
    bool hasVertShader     = true;
    
    // where to insert the view/clip stage vertex functions:
    bool viewStageInGeomShader     = hasGeomShader;
    bool viewStageInTessEvalShader = !hasGeomShader && hasTessEvalShader;
    bool viewStageInVertexShader   = !viewStageInTessEvalShader && !viewStageInGeomShader;

    OE_DEBUG << "hasGeomShader = " << hasGeomShader << "; viewStageInVertexShader = " << viewStageInVertexShader << "\n";
    
    bool clipStageInTessEvalShader = hasTessEvalShader;
    bool clipStageInVertexShader   = !hasGeomShader && !hasTessEvalShader;
    bool clipStageInGeomShader     = hasGeomShader && !hasTessEvalShader;

    // search for pragma varyings and build up our interface block definitions.
    typedef std::set<std::string> VaryingDefs;
    VaryingDefs varyingDefs;

    // built-ins:
    varyingDefs.insert( "vec4 vp_Color" );
    varyingDefs.insert( "vec3 vp_Normal" );

    for(VirtualProgram::ShaderMap::const_iterator s = in_shaders.begin(); s != in_shaders.end(); ++s )
    {
        osg::Shader* shader = s->second._shader->getNominalShader();
        if ( shader )
        {
            ShaderLoader::getAllQuotedPragmaValues(shader->getShaderSource(), "vp_varying", varyingDefs);
        }
    }

    typedef std::set< std::pair<std::string, std::string> > Varyings;
    Varyings varyings;
    for(VaryingDefs::iterator i = varyingDefs.begin(); i != varyingDefs.end(); ++i) 
    {
        std::vector<std::string> tokens;        
        StringTokenizer(*i, tokens, " \t", "", false, true);
        if ( tokens.size() == 2 )
            varyings.insert( std::make_pair(tokens[0], tokens[1]) );
    }

    std::string
        gl_Color                     = "gl_Color",
        gl_Vertex                    = "gl_Vertex",
        gl_Normal                    = "gl_Normal",
        gl_Position                  = "gl_Position",
        gl_ModelViewMatrix           = "gl_ModelViewMatrix",
        gl_ProjectionMatrix          = "gl_ProjectionMatrix",
        gl_ModelViewProjectionMatrix = "gl_ModelViewProjectionMatrix",
        gl_NormalMatrix              = "gl_NormalMatrix",
        gl_FrontColor                = "gl_FrontColor",
        gl_FragColor                 = "gl_FragColor";

    int version = 120;

    std::string
        varying_in  = version <= 110 ? "varying" : version <= 120? "varying in"  : "in",
        varying_out = version <= 110 ? "varying" : version <= 120? "varying out" : "out";
    
    // build the vertex data interface block definition:
    std::string vertdata;
    {
        std::stringstream buf;
        buf << "VP_Transit { \n";
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
        {
            buf << INDENT << i->first << " " << i->second << "; \n";
        }
        buf << "}";
        vertdata = buf.str();
    }


    std::string fragdata = vertdata;
        //"VP_FragData { \n"
        //"    vec4 vp_Color; \n"
        //"    vec3 vp_Normal; \n"
        //"}";

    // Build the vertex shader.
    if ( hasVertShader )
    {
        stages |= ShaderComp::STAGE_VERTEX;

        std::stringstream buf;

        buf <<
            "#version 120\n"
            "#pragma name \"VP Vertex Shader Main\" \n"
            "#extension GL_ARB_gpu_shader5 : enable \n";

        buf <<
            "\n// Vertex stage globals:\n"
            "vec4 vp_Vertex; \n";

        // Declare stage globals.
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << i->first << " " << i->second << "; \n";
        
        buf << "\n// Vertex stage outputs:\n";
        if ( hasGeomShader || hasTessShader )
            buf << "out " << vertdata << " vp_out; \n";
        else
            buf << "out " << fragdata << " vp_out; \n";

        // prototype functions:
        if ( modelStage || (viewStage && viewStageInVertexShader) || (clipStage && clipStageInVertexShader) )
        {
            buf << "\n// Function declarations:\n";
        }

        if ( modelStage )
        {
            for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4); \n";
            }
        }

        // prototypes for view stage methods:
        if ( viewStage != 0L && viewStageInVertexShader )
        {
            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4); \n";
            }
        }

        // prototypes for clip stage methods:
        if ( clipStage != 0L && clipStageInVertexShader )
        {
            for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4); \n";
            }
        }

        if ( hasGeomShader || hasTessShader || hasFragShader )
        {
        }

        if ( hasGeomShader || hasTessShader )
        {
        }

        buf <<
            "\nvoid main(void) \n"
            "{ \n"
            INDENT "vp_Vertex = " << gl_Vertex << "; \n"
            INDENT "vp_Normal = " << gl_Normal << "; \n"
            INDENT "vp_Color  = " << gl_Color  << "; \n";

        if ( modelStage )
        {
            for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
            {
                //insertRangeConditionals( i->second, buf );
                buf << INDENT << i->second._name << "(vp_Vertex); \n";
            }
        }

        if ( viewStageInVertexShader )
        {
            if ( viewStage )
            {
                buf <<
                    INDENT << "vp_Vertex = " << gl_ModelViewMatrix << " * vp_Vertex; \n"
                    INDENT << "vp_Normal = " << gl_NormalMatrix    << " * vp_Normal; \n";

                for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
                {
                    buf << INDENT << i->second._name << "(vp_Vertex); \n";
                }
            }

            if ( clipStageInVertexShader )
            {
                if ( clipStage )
                {
                    if ( viewStage )
                    {
                        buf << INDENT << "vp_Vertex = " << gl_ProjectionMatrix << " * vp_Vertex; \n";
                    }
                    else
                    {
                        buf <<
                            INDENT << "vp_Vertex = " << gl_ModelViewProjectionMatrix << " * vp_Vertex; \n"
                            INDENT << "vp_Normal = " << gl_NormalMatrix              << " * vp_Normal; \n";
                    }

                    for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
                    {
                        buf << INDENT << i->second._name << "(vp_Vertex); \n";
                    }
                }
            }
        }

        // if there are no further vertex-processing stages, transform the position into clip coordinates
        // for the fragment shader now:
        if ( !hasGeomShader && !hasTessShader )
        {
            if ( clipStage )
                buf << INDENT "gl_Position = vp_Vertex; \n";
            else if ( viewStage )
                buf << INDENT "gl_Position = " << gl_ProjectionMatrix << " * vp_Vertex; \n";
            else
                buf << INDENT "gl_Position = " << gl_ModelViewProjectionMatrix << " * vp_Vertex; \n";
        }

        // otherwise, pass it along as-is.
        else
        {
            buf << INDENT "gl_Position = vp_Vertex; \n";
        }


        if ( hasTessShader || hasGeomShader || hasFragShader )
        {
            // Copy stage globals to output block:
            for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
                buf << INDENT << "vp_out." << i->second << " = " << i->second << "; \n";
        }

        if ( hasTessShader || hasGeomShader )
        {
        }

        buf << "} \n";

        std::string str;
        str = buf.str();
        osg::Shader* vertexShader = new osg::Shader( osg::Shader::VERTEX, str );
        vertexShader->setName( "main(vertex)" );
        out_shaders.push_back( vertexShader );
    }


    //.................................................................................


    if ( hasTessShader )
    {
        stages |= ShaderComp::STAGE_TESSCONTROL;
        //todo
    }


    //.................................................................................


    if ( hasTessEvalShader )
    {
        stages |= ShaderComp::STAGE_TESSEVALULATION;
        //todo
    }


    //.................................................................................


    // Build the geometry shader.
    if ( hasGeomShader )
    {
        stages |= ShaderComp::STAGE_GEOMETRY;

        std::stringstream buf;

        buf << "#version 330\n"
            << "#pragma name \"VP Geometry Shader Main\" \n";

        if ( hasVertShader )
        {
              buf << "\n// Geometry stage inputs:\n"
                 << "in " << vertdata << " vp_in []; \n";
        }
        
        buf << "\n// Geometry stage globals: \n";

        // Declare stage globals.
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << i->first << " " << i->second << "; \n";
        
        buf << "\n// Geometry stage outputs: \n";
        if ( hasTessShader )
            buf << "out " << vertdata << " vp_out; \n";
        else
            buf << "out " << fragdata << " vp_out; \n";

        if ( geomStage || (viewStage && viewStageInGeomShader) || (clipStage && clipStageInGeomShader) )
        {
            buf << "\n// Function declarations:\n";
            if ( geomStage )
            {
                for( OrderedFunctionMap::const_iterator i = geomStage->begin(); i != geomStage->end(); ++i )
                {
                    buf << "void " << i->second._name << "(); \n";
                }
            }
        }

        buf << "\n"
            << "void main(void) \n"
            << "{ \n"
            << INDENT "// copy default outputs: \n";
        
        // Copy default input block to output block (auto passthrough on first vert)
        // NOT SURE WE NEED THIS
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << INDENT << "vp_out." << i->second << " = vp_in[0]." << i->second << "; \n";

        if ( geomStage )
        {
            for( OrderedFunctionMap::const_iterator i = geomStage->begin(); i != geomStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(); \n";
            }
        }

        buf << "} \n";

        std::string str;
        str = buf.str();
        osg::Shader* geomShader = new osg::Shader( osg::Shader::GEOMETRY, str );
        geomShader->setName( "main(geometry)" );
        out_shaders.push_back( geomShader );


        // Construct the GS Helper Functions.
        buf.str("");
        buf << "#version 330 compatibility\n"
            << "#pragma name \"VP Geometry Shader Helper Functions\"\n\n";

        buf << "in " << vertdata << " vp_in []; \n\n";       
        
        if ( hasTessShader )
            buf << "out " << vertdata << " vp_out; \n";
        else
            buf << "out " << fragdata << " vp_out; \n";
        
        // Declare stage globals.
        buf << "\n// Geometry stage globals\n";        
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << i->first << " " << i->second << "; \n";
        

        if ( viewStage && viewStageInGeomShader )
        {
            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4); \n";
            }
        }

        if ( clipStage && clipStageInGeomShader )
        {
            for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4); \n";
            }
        }

        buf << "\nvoid VP_LoadVertex(in int index) \n"
            << "{ \n";
        
        // Copy input block to stage globals:
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << INDENT << i->second << " = vp_in[index]." << i->second << "; \n";

        buf << "} \n";

        buf << "\nvoid VP_EmitVertex(in vec4 vertex) \n"
            << "{ \n"
            << "    vec4 v = vertex; \n";

        int space = SPACE_MODEL;

        if ( viewStage && viewStageInGeomShader )
        {
            buf << INDENT << "v = " << gl_ModelViewMatrix << " * v; \n";
            space = SPACE_VIEW;

            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(v); \n";
            }
        }

        if ( clipStage && clipStageInGeomShader )
        {
            if ( space == SPACE_MODEL )
                buf << INDENT << "v = " << gl_ModelViewProjectionMatrix << " * v; \n";
            else if ( space == SPACE_VIEW )
                buf << INDENT << "v = " << gl_ProjectionMatrix << " * v; \n";

            space = SPACE_CLIP;

            for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(v); \n";
            }
        }
        
        // Copy globals to output block:
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << INDENT << "vp_out." << i->second << " = " << i->second << "; \n";

        // resolve vertex to its next space:
        if ( space == SPACE_MODEL )
            buf << INDENT << "v = " << gl_ModelViewProjectionMatrix << " * v; \n";
        else if ( space == SPACE_VIEW )
            buf << INDENT << "v = " << gl_ProjectionMatrix << " * v; \n";

        buf << INDENT << "gl_Position = v; \n"
            << INDENT << "EmitVertex(); \n"
            << "} \n";
        
        str = buf.str();
        osg::Shader* helperShader = new osg::Shader(osg::Shader::GEOMETRY, str);
        helperShader->setName("vp helpers(geometry)");
        out_shaders.push_back( helperShader );
    }
    

    //.................................................................................


    // Build the Fragment shader.
    if ( hasFragShader )
    {
        stages |= ShaderComp::STAGE_FRAGMENT;

        std::stringstream buf;

        buf << "#version 120\n"
            << "#pragma name \"VP Fragment Shader Main\" \n"
            << "#extension GL_ARB_gpu_shader5 : enable \n";

        buf << "\n// Fragment stage inputs:\n";
        buf << "in " << fragdata << " vp_in; \n";
        
        buf <<
            "\n// Fragment stage globals:\n";

        // Declare stage globals.
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << i->first << " " << i->second << "; \n";

        if ( coloringStage || lightingStage || outputStage )
        {
            buf << "\n// Function declarations:\n";
        }

        if ( coloringStage )
        {
            for( OrderedFunctionMap::const_iterator i = coloringStage->begin(); i != coloringStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4 color); \n";
            }
        }

        if ( lightingStage )
        {
            for( OrderedFunctionMap::const_iterator i = lightingStage->begin(); i != lightingStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4 color); \n";
            }
        }

        if ( outputStage )
        {
            for( OrderedFunctionMap::const_iterator i = outputStage->begin(); i != outputStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4 color); \n";
            }
        }

        buf << 
            "\nvoid main(void) \n"
            "{ \n";
        
        // Copy input block to stage globals:
        for(Varyings::const_iterator i = varyings.begin(); i != varyings.end(); ++i)
            buf << INDENT << i->second << " = vp_in." << i->second << "; \n";

        int coloringPass = _fragStageOrder == FRAGMENT_STAGE_ORDER_COLORING_LIGHTING ? 0 : 1;
        int lightingPass = 1-coloringPass;

        for(int pass=0; pass<2; ++pass)
        {
            if ( coloringStage && (pass == coloringPass) )
            {
                for( OrderedFunctionMap::const_iterator i = coloringStage->begin(); i != coloringStage->end(); ++i )
                {
                    buf << INDENT << i->second._name << "( vp_Color ); \n";
                }
            }

            if ( lightingStage && (pass == lightingPass) )
            {
                for( OrderedFunctionMap::const_iterator i = lightingStage->begin(); i != lightingStage->end(); ++i )
                {
                    buf << INDENT << i->second._name << "( vp_Color ); \n";
                }
            }
        }

        if ( outputStage )
        {
            for( OrderedFunctionMap::const_iterator i = outputStage->begin(); i != outputStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "( vp_Color ); \n";
            }
        }
        else
        {
            // in the absense of any output functions, generate a default output statement
            // that simply writes to gl_FragColor.
            buf << INDENT << gl_FragColor << " = vp_Color;\n";
        }
        buf << "}\n";

        std::string str;
        str = buf.str();
        osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
        shader->setName( "main(fragment)" );
        out_shaders.push_back( shader );
    }

    return stages;
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
            buf << "void " << i->second._name << "(inout vec4); \n";
        }
    }

    // prototypes for view stage methods:
    if ( viewStage )
    {
        for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
        {
            buf << "void " << i->second._name << "(inout vec4); \n";
        }
    }

    // prototypes for clip stage methods:
    if ( clipStage )
    {
        for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
        {
            buf << "void " << i->second._name << "(inout vec4); \n";
        }
    }

    // main:
    buf <<
        "varying vec4 osg_FrontColor; \n"
        "varying vec3 vp_Normal; \n"
        "void main(void) \n"
        "{ \n"
        INDENT "osg_FrontColor = gl_Color; \n"
        INDENT "vec4 vertex = gl_Vertex; \n";

    // call Model stage methods.
    if ( modelStage )
    {
        buf << INDENT "vp_Normal = gl_Normal; \n";

        for( OrderedFunctionMap::const_iterator i = modelStage->begin(); i != modelStage->end(); ++i )
        {
            insertRangeConditionals( i->second, buf );
            buf << INDENT << i->second._name << "(vertex); \n";
        }

        buf << INDENT << "vp_Normal = normalize(gl_NormalMatrix * vp_Normal); \n";
    }
    else
    {
        buf << INDENT << "vp_Normal = normalize(gl_NormalMatrix * gl_Normal); \n";
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
        "varying vec3 vp_Normal; \n"
        "vec3 oe_global_Normal; \n" // stage-global
        "void main(void) \n"
        "{ \n"
        INDENT "vec4 color = osg_FrontColor; \n"
        INDENT "oe_global_Normal = normalize(vp_Normal); \n";

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
