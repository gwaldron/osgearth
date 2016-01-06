/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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

namespace
{
    struct Variable
    {
        std::string interp;      // interpolation qualifer (flat, etc.)
        std::string type;        // float, vec4, etc.
        std::string name;        // name without any array specifiers, etc.
        std::string declaration; // name including array specifiers (for decl)
        int         arraySize;   // 0 if not an array; else array size.
    };

    typedef std::vector<Variable> Variables;
}


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
    const OrderedFunctionMap* tessControlStage = f != functions.end() ? &f->second : 0L;

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
    bool hasGS  = geomStage        && !geomStage->empty();
    bool hasTCS = tessControlStage && !tessControlStage->empty();
    bool hasTES = tessEvalStage    && !tessEvalStage->empty();
    bool hasFS  = true;
    bool hasVS  = true;
    
    // where to insert the view/clip stage vertex functions:
    bool viewStageInGS  = hasGS;
    bool viewStageInTES = !viewStageInGS && hasTES;
    bool viewStageInVS  = !viewStageInTES && !viewStageInGS;
    
    bool clipStageInGS  = hasGS;
    bool clipStageInTES = hasTES && !hasGS;
    bool clipStageInVS  = !clipStageInGS && !clipStageInTES;

    // search for pragma varyings and build up our interface block definitions.
    typedef std::set<std::string> VarDefs;
    VarDefs varDefs;

    // built-ins:
    varDefs.insert( "vec4 vp_Color" );
    varDefs.insert( "vec3 vp_Normal" );
    varDefs.insert( "vec4 vp_Vertex" );

    // parse the vp_varyings (which were injected by the ShaderLoader)
    for(VirtualProgram::ShaderMap::const_iterator s = in_shaders.begin(); s != in_shaders.end(); ++s )
    {
        osg::Shader* shader = s->data()._shader->getNominalShader();
        if ( shader )
        {
            ShaderLoader::getAllPragmaValues(shader->getShaderSource(), "vp_varying", varDefs);
        }
    }

    Variables vars;
    for(VarDefs::iterator i = varDefs.begin(); i != varDefs.end(); ++i) 
    {
        std::vector<std::string> tokens;        
        StringTokenizer st;
        st.addDelims( " \t", false );
        st.addDelims( "[]", true );
        st.tokenize( *i, tokens ); //(*i, tokens, " \t", "", false, true);
        if ( tokens.size() >= 2 )
        {
            int p=0;
            Variable v;
            if ( tokens[p] == "flat" || tokens[p] == "nonperspective" || tokens[p] == "smooth" )
            {
                v.interp = tokens[p++];
            }

            if ( p+1 < tokens.size() )
            {
                v.type = tokens[p++];
                v.name = tokens[p++];

                // check for array
                if ( p+2 < tokens.size() && tokens[p] == "[" && tokens[p+2] == "]" )
                {
                    v.declaration = Stringify() << v.type << " " << v.name << tokens[p] << tokens[p+1] << tokens[p+2];
                    v.arraySize = as<int>(tokens[p+1], 0);
                }
                else
                {
                    v.declaration = Stringify() << v.type << " " << v.name;
                    v.arraySize = 0;
                }
            }

            if ( !v.type.empty() && !v.name.empty() && !v.declaration.empty() )
            {
                vars.push_back( v );
            }
        }
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

    #define VS_GLSL_VERSION  "330 compatibility"
    #define FS_GLSL_VERSION  "330 compatibility"
    #define GS_GLSL_VERSION  "330 compatibility"
    #define TCS_GLSL_VERSION "400 compatibility"
    #define TES_GLSL_VERSION "400 compatibility"

    // build the vertex data interface block definition:
    std::string vertdata;
    {
        std::stringstream buf;
        buf << "VP_PerVertex { \n";
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << i->interp << (i->interp.empty()?"":" ") << i->declaration << "; \n";
        buf << "}";
        vertdata = buf.str();
    }

    // TODO: perhaps optimize later to not include things we don't need in the FS
    std::string fragdata = vertdata;

    // Build the vertex shader.
    if ( hasVS )
    {
        stages |= ShaderComp::STAGE_VERTEX;

        std::stringstream buf;

        buf <<
            "#version " VS_GLSL_VERSION "\n"
            "#pragma vp_name VP Vertex Shader Main\n"
            "#extension GL_ARB_gpu_shader5 : enable \n";

        buf << "\n// Vertex stage globals:\n";
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << i->declaration << "; \n";
        
        buf << "\n// Vertex stage outputs:\n";
        if ( hasGS || hasTCS )
            buf << "out " << vertdata << " vp_out; \n";
        else
            buf << "out " << fragdata << " vp_out; \n";

        // prototype functions:
        if ( modelStage || (viewStage && viewStageInVS) || (clipStage && clipStageInVS) )
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
        if ( viewStage != 0L && viewStageInVS )
        {
            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4); \n";
            }
        }

        // prototypes for clip stage methods:
        if ( clipStage != 0L && clipStageInVS )
        {
            for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            {
                buf << "void " << i->second._name << "(inout vec4); \n";
            }
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

        if ( viewStageInVS )
        {
            if ( viewStage )
            {
                buf <<
                    INDENT << "vp_Vertex = " << gl_ModelViewMatrix << " * vp_Vertex; \n"
                    INDENT << "vp_Normal = normalize(" << gl_NormalMatrix    << " * vp_Normal); \n";

                for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
                {
                    buf << INDENT << i->second._name << "(vp_Vertex); \n";
                }
            }

            if ( clipStageInVS )
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
                            INDENT << "vp_Normal = normalize(" << gl_NormalMatrix << " * vp_Normal); \n";
                    }

                    for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
                    {
                        buf << INDENT << i->second._name << "(vp_Vertex); \n";
                    }
                }
            }
        }

        // if there are no further vertex-processing stages, transform the position into clip coordinates
        // for the fragment shader now:
        if ( !hasGS && !hasTCS )
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


        if ( hasTCS || hasGS || hasFS )
        {
            // Copy stage globals to output block:
            for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
                buf << INDENT << "vp_out." << i->name << " = " << i->name << "; \n";
        }

        buf << "} \n";

        osg::Shader* vertexShader = new osg::Shader( osg::Shader::VERTEX, buf.str() );
        vertexShader->setName( "main(vertex)" );
        out_shaders.push_back( vertexShader );
    }


    //.................................................................................


    if ( hasTCS )
    {
        stages |= ShaderComp::STAGE_TESSCONTROL;
        std::stringstream buf;

        buf << "#version " TCS_GLSL_VERSION "\n"
            << "#pragma vp_name VP Tessellation Control Shader (TCS) Main\n";

        if ( hasVS )
        {
              buf << "\n// TCS stage inputs:\n"
                 << "in " << vertdata << " vp_in [gl_MaxPatchVertices]; \n";
        }

        // The TES is mandatory.
        buf << "\n// TCS stage outputs to TES: \n"
            << "out " << vertdata << " vp_out [gl_MaxPatchVertices]; \n";

        // Stage globals.
        buf << "\n// TCS stage globals \n";
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << i->declaration << "; \n";

        // Helper functions:
        // TODO: move this into its own osg::Shader so it can be shared.
        buf << "\nvoid VP_LoadVertex(in int index) \n"
            << "{ \n";
        
        // Copy input block to stage globals:
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << i->name << " = vp_in[index]." << i->name << "; \n";

        buf << "} \n";
        
        // Function declares
        if ( tessControlStage )
        {
            buf << "\n// Function declarations:\n";
            for( OrderedFunctionMap::const_iterator i = tessControlStage->begin(); i != tessControlStage->end(); ++i )
                buf << "void " << i->second._name << "(); \n";
        }

        // Main
        buf << "\nvoid main(void) \n"
            << "{ \n"
            << INDENT "// copy default outputs: \n";
                
        // Copy in to globals
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << i->name << " = vp_in[gl_InvocationID]." << i->name << "; \n";

        // Invoke functions
        if ( tessControlStage )
        {
            for( OrderedFunctionMap::const_iterator i = tessControlStage->begin(); i != tessControlStage->end(); ++i )
                buf << INDENT << i->second._name << "(); \n";
        }
                
        // Copy globals to out.
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << "vp_out[gl_InvocationID]." << i->name << " = " << i->name << "; \n";

        buf << "} \n";
        
        std::string str = buf.str();
        osg::Shader* tcsShader = new osg::Shader(osg::Shader::TESSCONTROL, str);
        tcsShader->setName("VP TCS");
        out_shaders.push_back( tcsShader );
    }


    //.................................................................................


    if ( hasTES )
    {
        stages |= ShaderComp::STAGE_TESSEVALUATION;

        std::stringstream buf;

        buf << "#version " TES_GLSL_VERSION "\n"
            << "#pragma vp_name VP Tessellation Evaluation (TES) Shader MAIN\n";

        buf << "\n// TES stage inputs (required):\n"
            << "in " << vertdata << " vp_in []; \n";
        
        // Declare stage globals.
        buf << "\n// TES stage globals: \n";
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << i->declaration << "; \n";
        
        buf << "\n// TES stage outputs: \n";
        if ( hasGS )
            buf << "out " << vertdata << " vp_out; \n";
        else
            buf << "out " << fragdata << " vp_out; \n";

        std::set<std::string> types;
        for(Variables::const_iterator i=vars.begin(); i != vars.end(); ++i)
            types.insert(i->type);

        for(std::set<std::string>::const_iterator i = types.begin(); i != types.end(); ++i)
        {
            buf << *i << " VP_Interpolate3(" << *i << "," << *i << "," << *i << ");\n";
        }       

        //buf <<
        //    "\n// TES user-supplied interpolators: \n"
        //    "float VP_Interpolate3(float,float,float); \n"
        //    "vec2  VP_Interpolate3(vec2,vec2,vec2); \n"
        //    "vec3  VP_Interpolate3(vec3,vec3,vec3); \n"
        //    "vec4  VP_Interpolate3(vec4,vec4,vec4); \n";

#if 0
        buf <<
            "\n// TES user-supplied interpolators: \n"
            "float VP_Interpolate16(float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float); \n"
            "vec2  VP_Interpolate16(vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2,vec2); \n"
            "vec3  VP_Interpolate16(vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3,vec3); \n"
            "vec4  VP_Interpolate16(vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4,vec4); \n";
#endif

        if ( tessEvalStage || (viewStage && viewStageInTES) || (clipStage && clipStageInTES) )
        {
            buf << "\n// Function declarations:\n";
            if ( tessEvalStage )
            {
                for( OrderedFunctionMap::const_iterator i = tessEvalStage->begin(); i != tessEvalStage->end(); ++i )
                {
                    buf << "void " << i->second._name << "(); \n";
                }
            }

            if (viewStage && viewStageInTES)
            {
                for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
                {
                    buf << "void " << i->second._name << "(inout vec4); \n";
                }
            }

            if (clipStage && clipStageInTES) 
            {
                for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
                {
                    buf << "void " << i->second._name << "(inout vec4); \n";
                }
            }

            // Helper functions:
            buf << "\nvoid VP_LoadVertex(in int index) \n"
                << "{ \n";
        
            // Copy input block to stage globals:
            for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
                buf << INDENT << i->name << " = vp_in[index]." << i->name << "; \n";

            buf << "} \n";

            buf << "\nvoid VP_Interpolate3() \n"
                << "{ \n";            
            for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            {
                if ( i->interp != "flat" )
                {
                    if ( i->arraySize == 0 )
                    {
                        buf << INDENT << i->name << " = VP_Interpolate3"
                            << "( vp_in[0]." << i->name
                            << ", vp_in[1]." << i->name
                            << ", vp_in[2]." << i->name << " ); \n";
                    }
                    else
                    {
                        for(int n=0; n<i->arraySize; ++n)
                        {
                            buf << INDENT << i->name << "[" << n << "] = VP_Interpolate3"
                                << "( vp_in[0]." << i->name << "[" << n << "]"
                                << ", vp_in[1]." << i->name << "[" << n << "]"
                                << ", vp_in[2]." << i->name << "[" << n << "] ); \n";
                        }
                    }
                }
                else
                {
                    buf << INDENT << i->name << " = vp_in[0]." << i->name << "; \n";
                }
            }
            buf << "} \n";

            buf << "\nvoid VP_EmitVertex() \n"
                << "{ \n";

            int space = SPACE_MODEL;

            if ( viewStage && viewStageInTES )
            {
                buf << INDENT << "vp_Vertex = " << gl_ModelViewMatrix << " * vp_Vertex; \n"
                    << INDENT << "vp_Normal = normalize(" << gl_NormalMatrix << " * vp_Normal); \n";
                space = SPACE_VIEW;

                for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
                {
                    buf << INDENT << i->second._name << "(vp_Vertex); \n";
                }
            }

            if ( clipStage && clipStageInTES )
            {
                if ( space == SPACE_MODEL )
                    buf << INDENT << "vp_Vertex = " << gl_ModelViewProjectionMatrix << " * vp_Vertex; \n"
                    << INDENT << "vp_Normal = normalize(" << gl_NormalMatrix << " * vp_Normal); \n";
                else if ( space == SPACE_VIEW )
                    buf << INDENT << "vp_Vertex = " << gl_ProjectionMatrix << " * vp_Vertex; \n";

                space = SPACE_CLIP;

                for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
                {
                    buf << INDENT << i->second._name << "(vp_Vertex); \n";
                }
            }

            // resolve vertex to its next space, but ONLY if this is the final Vertex Processing stage.
            if ( !hasGS )
            {
                if ( space == SPACE_MODEL )
                    buf << INDENT << "vp_Vertex = " << gl_ModelViewProjectionMatrix << " * vp_Vertex; \n"
                        << INDENT << "vp_Normal = normalize(" << gl_NormalMatrix << " * vp_Normal); \n";
                else if ( space == SPACE_VIEW )
                    buf << INDENT << "vp_Vertex = " << gl_ProjectionMatrix << " * vp_Vertex; \n";
            }
        
            // Copy globals to output block:
            for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
                buf << INDENT << "vp_out." << i->name << " = " << i->name << "; \n";

            buf << INDENT << "gl_Position = vp_Vertex; \n"
                << "} \n";
        }

        buf << "\n"
            << "void main(void) \n"
            << "{ \n"
            << INDENT "// copy default outputs: \n";
        
        if ( !tessEvalStage )
        {
            // Copy default input block to output block (auto passthrough on first vert)
            // NOT SURE WE NEED THIS
            for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
                buf << INDENT << "vp_out." << i->name << " = vp_in[0]." << i->name << "; \n";
        }

        if ( tessEvalStage )
        {
            for( OrderedFunctionMap::const_iterator i = tessEvalStage->begin(); i != tessEvalStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(); \n";
            }
        }

        buf << "} \n";
        
        std::string str = buf.str();
        osg::Shader* tesShader = new osg::Shader(osg::Shader::TESSEVALUATION, str);
        tesShader->setName("VP TES");
        out_shaders.push_back( tesShader );
    }


    //.................................................................................


    // Build the geometry shader.
    if ( hasGS )
    {
        stages |= ShaderComp::STAGE_GEOMETRY;

        std::stringstream buf;

        buf << "#version " GS_GLSL_VERSION "\n"
            << "#pragma vp_name VP Geometry Shader Main\n";

        if ( hasVS || hasTCS || hasTES )
        {
            buf << "\n// Geometry stage inputs:\n"
                << "in " << vertdata << " vp_in []; \n";
        }        

        // Declare stage globals.
        buf << "\n// Geometry stage globals: \n";
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << i->declaration << "; \n";
        
        buf << "\n// Geometry stage outputs: \n"
            << "out " << fragdata << " vp_out; \n";

        if ( geomStage || (viewStage && viewStageInGS) || (clipStage && clipStageInGS) )
        {
            buf << "\n// Injected function declarations:\n";
            if ( geomStage )
            {
                for( OrderedFunctionMap::const_iterator i = geomStage->begin(); i != geomStage->end(); ++i )
                {
                    buf << "void " << i->second._name << "(); \n";
                }
            }       

            if ( viewStage && viewStageInGS )
            {
                for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
                {
                    buf << "void " << i->second._name << "(inout vec4); \n";
                }
            }

            if ( clipStage && clipStageInGS )
            {
                for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
                {
                    buf << "void " << i->second._name << "(inout vec4); \n";
                }
            }
        }
        
        // Build-in helper functions:
        buf << "\nvoid VP_LoadVertex(in int index) \n"
            << "{ \n";
        
        // Copy input block to stage globals:
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << i->name << " = vp_in[index]." << i->name << "; \n";

        buf << "} \n";

        buf << "\nvoid VP_EmitModelVertex() \n"
            << "{ \n";
        
        buf << INDENT << "vp_Vertex = gl_Position; \n";
        int space = SPACE_MODEL;
        if ( viewStage && viewStageInGS )
        {
            buf << INDENT << "vp_Vertex = " << gl_ModelViewMatrix << " * vp_Vertex;\n"
                << INDENT << "vp_Normal = normalize(" << gl_NormalMatrix << " * vp_Normal); \n";

            space = SPACE_VIEW;

            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(vp_Vertex); \n";
            }
        }

        if ( clipStage && clipStageInGS )
        {
            if ( space == SPACE_MODEL )
            {
                buf << INDENT << "vp_Vertex = " << gl_ModelViewProjectionMatrix << " * vp_Vertex; \n"
                    << INDENT << "vp_Normal = normalize(" << gl_NormalMatrix << " * vp_Normal); \n";
            }
            else if ( space == SPACE_VIEW )
            {
                buf << INDENT << "vp_Vertex = " << gl_ProjectionMatrix << " * vp_Vertex; \n";
            }            

            space = SPACE_CLIP;

            for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(vp_Vertex); \n";
            }
        }

        // resolve vertex to its next space:
        if ( space == SPACE_MODEL )
        {
            buf << INDENT << "vp_Vertex = " << gl_ModelViewProjectionMatrix << " * vp_Vertex; \n"
                << INDENT << "vp_Normal = normalize(" << gl_NormalMatrix << " * vp_Normal); \n";
        }
        else if ( space == SPACE_VIEW )
        {
            buf << INDENT << "vp_Vertex = " << gl_ProjectionMatrix << " * vp_Vertex; \n";
        }
                
        // Copy globals to output block:
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << "vp_out." << i->name << " = " << i->name << "; \n";

        buf << INDENT << "gl_Position = vp_Vertex; \n";

        buf << INDENT << "EmitVertex(); \n"
            << "} \n";
        

        buf << "\nvoid VP_EmitViewVertex() \n"
            << "{ \n";
        
        buf << INDENT << "vp_Vertex = gl_Position; \n";
        space = SPACE_VIEW;
        if ( viewStage && viewStageInGS )
        {
            for( OrderedFunctionMap::const_iterator i = viewStage->begin(); i != viewStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(vp_Vertex); \n";
            }
        }

        if ( clipStage && clipStageInGS )
        {
            buf << INDENT << "vp_Vertex = " << gl_ProjectionMatrix << " * vp_Vertex; \n";
            space = SPACE_CLIP;
            for( OrderedFunctionMap::const_iterator i = clipStage->begin(); i != clipStage->end(); ++i )
            {
                buf << INDENT << i->second._name << "(vp_Vertex); \n";
            }
        }

        // resolve vertex to its next space:
        if ( space == SPACE_VIEW )
            buf << INDENT << "vp_Vertex = " << gl_ProjectionMatrix << " * vp_Vertex; \n";
                
        // Copy globals to output block:
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << "vp_out." << i->name << " = " << i->name << "; \n";

        buf << INDENT << "gl_Position = vp_Vertex; \n";

        buf << INDENT << "EmitVertex(); \n"
            << "} \n";

        buf << "\n"
            << "void main(void) \n"
            << "{ \n"
            << INDENT "// copy default outputs: \n";
        
        // Copy default input block to output block (auto passthrough on first vert)
        // NOT SURE WE NEED THIS
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << "vp_out." << i->name << " = vp_in[0]." << i->name << "; \n";

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
    }
    

    //.................................................................................


    // Build the Fragment shader.
    if ( hasFS )
    {
        stages |= ShaderComp::STAGE_FRAGMENT;

        std::stringstream buf;

        buf << "#version " FS_GLSL_VERSION "\n"
            << "#pragma vp_name VP Fragment Shader Main\n"
            << "#extension GL_ARB_gpu_shader5 : enable \n";

        buf << "\n// Fragment stage inputs:\n";
        buf << "in " << fragdata << " vp_in; \n";
        
        buf <<
            "\n// Fragment stage globals:\n";

        // Declare stage globals.
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << i->declaration << ";\n";

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
        for(Variables::const_iterator i = vars.begin(); i != vars.end(); ++i)
            buf << INDENT << i->name << " = vp_in." << i->name << "; \n";

        buf << INDENT << "vp_Normal = normalize(vp_Normal); \n";

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
