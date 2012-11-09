/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

#define VERTEX_SETUP_COLORING   "osgearth_vert_setupColoring"
#define VERTEX_SETUP_LIGHTING   "osgearth_vert_setupLighting"
#define FRAGMENT_APPLY_COLORING "osgearth_frag_applyColoring"
#define FRAGMENT_APPLY_LIGHTING "osgearth_frag_applyLighting"

#ifdef OSG_GLES2_AVAILABLE
#   define PRECISION_MEDIUMP_FLOAT "precision mediump float;"
    static bool s_GLES_SHADERS = true;
#   define GLENNS_PER_VERTEX_LIGHTING 1
#else
#   define PRECISION_MEDIUMP_FLOAT ""
    static bool s_GLES_SHADERS = false;
#   define GLENNS_PER_VERTEX_LIGHTING 1
#endif


using namespace osgEarth;
using namespace osgEarth::ShaderComp;


std::string
ShaderFactory::getSamplerName( unsigned unit ) const
{
    return Stringify() << "osgearth_tex" << unit;
}


osg::Shader*
ShaderFactory::createVertexShaderMain(const FunctionLocationMap& functions,
                                      bool  useLightingShaders ) const
{
    FunctionLocationMap::const_iterator i = functions.find( LOCATION_VERTEX_PRE_TEXTURING );
    const OrderedFunctionMap* preTexture = i != functions.end() ? &i->second : 0L;

    FunctionLocationMap::const_iterator j = functions.find( LOCATION_VERTEX_PRE_LIGHTING );
    const OrderedFunctionMap* preLighting = j != functions.end() ? &j->second : 0L;

    FunctionLocationMap::const_iterator k = functions.find( LOCATION_VERTEX_POST_LIGHTING );
    const OrderedFunctionMap* postLighting = k != functions.end() ? &k->second : 0L;

    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n"
        << PRECISION_MEDIUMP_FLOAT "\n"
        << "void osgearth_vert_setupColoring(); \n";

    if ( useLightingShaders )
        buf << "void osgearth_vert_setupLighting(); \n";

    if ( preTexture )
        for( OrderedFunctionMap::const_iterator i = preTexture->begin(); i != preTexture->end(); ++i )
            buf << "void " << i->second << "(); \n";

    if ( preLighting )
        for( OrderedFunctionMap::const_iterator i = preLighting->begin(); i != preLighting->end(); ++i )
            buf << "void " << i->second << "(); \n";

    if ( postLighting )
        for( OrderedFunctionMap::const_iterator i = postLighting->begin(); i != postLighting->end(); ++i )
            buf << "void " << i->second << "(); \n";

    buf << "void main(void) \n"
        << "{ \n"
        << "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n";

    if ( preTexture )
        for( OrderedFunctionMap::const_iterator i = preTexture->begin(); i != preTexture->end(); ++i )
            buf << "    " << i->second << "(); \n";

    buf << "    osgearth_vert_setupColoring(); \n";
    
    if ( preLighting )
        for( OrderedFunctionMap::const_iterator i = preLighting->begin(); i != preLighting->end(); ++i )
            buf << "    " << i->second << "(); \n";

    if ( useLightingShaders )
        buf << "    osgearth_vert_setupLighting(); \n";
    
    if ( postLighting )
        for( OrderedFunctionMap::const_iterator i = postLighting->begin(); i != postLighting->end(); ++i )
            buf << "    " << i->second << "(); \n";

    buf << "} \n";

    std::string str;
    str = buf.str();
    osg::Shader* shader = new osg::Shader( osg::Shader::VERTEX, str );
    shader->setName( "main(vert)" );
    return shader;
}


osg::Shader*
ShaderFactory::createFragmentShaderMain(const FunctionLocationMap& functions,
                                        bool  useLightingShaders ) const
{
    FunctionLocationMap::const_iterator i = functions.find( LOCATION_FRAGMENT_PRE_TEXTURING );
    const OrderedFunctionMap* preTexture = i != functions.end() ? &i->second : 0L;

    FunctionLocationMap::const_iterator j = functions.find( LOCATION_FRAGMENT_PRE_LIGHTING );
    const OrderedFunctionMap* preLighting = j != functions.end() ? &j->second : 0L;

    FunctionLocationMap::const_iterator k = functions.find( LOCATION_FRAGMENT_POST_LIGHTING );
    const OrderedFunctionMap* postLighting = k != functions.end() ? &k->second : 0L;

    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n"
        << PRECISION_MEDIUMP_FLOAT << "\n"
        << "void osgearth_frag_applyColoring( inout vec4 color ); \n";

    if ( useLightingShaders )
        buf << "void osgearth_frag_applyLighting( inout vec4 color ); \n";

    if ( preTexture )
        for( OrderedFunctionMap::const_iterator i = preTexture->begin(); i != preTexture->end(); ++i )
            buf << "void " << i->second << "( inout vec4 color ); \n";

    if ( preLighting )
        for( OrderedFunctionMap::const_iterator i = preLighting->begin(); i != preLighting->end(); ++i )
            buf << "void " << i->second << "( inout vec4 color ); \n";

    if ( postLighting )
        for( OrderedFunctionMap::const_iterator i = postLighting->begin(); i != postLighting->end(); ++i )
            buf << "void " << i->second << "( inout vec4 color ); \n";

    buf << "void main(void) \n"
        << "{ \n"
        << "    vec4 color = vec4(1,1,1,1); \n"; //gl_Color; \n";

    if ( preTexture )
        for( OrderedFunctionMap::const_iterator i = preTexture->begin(); i != preTexture->end(); ++i )
            buf << "    " << i->second << "( color ); \n";

    buf << "    osgearth_frag_applyColoring( color ); \n";

    if ( preLighting )
        for( OrderedFunctionMap::const_iterator i = preLighting->begin(); i != preLighting->end(); ++i )
            buf << "    " << i->second << "( color ); \n";
    
    if ( useLightingShaders )
        buf << "    osgearth_frag_applyLighting( color ); \n";

    if ( postLighting )
        for( OrderedFunctionMap::const_iterator i = postLighting->begin(); i != postLighting->end(); ++i )
            buf << "    " << i->second << "( color ); \n";

    buf << "    gl_FragColor = color; \n"

#if 0 // GW: testing logarithmic depth buffer remapping
        << "    float A = gl_ProjectionMatrix[2].z; \n"
        << "    float B = gl_ProjectionMatrix[3].z; \n"
        << "    float n = -B/(1.0-A); \n"
        << "    float f =  B/(1.0+A); \n"
        << "    float C = 1; \n"
        << "    gl_FragDepth = log(C*gl_FragCoord.z+1) / log(C*f+1); \n"
#endif
        << "} \n";  

    std::string str;
    str = buf.str();
    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
    shader->setName( "main(frag)" );
    return shader;
}
 

osg::Shader*
ShaderFactory::createDefaultColoringVertexShader( unsigned numTexCoordSets ) const
{
    std::stringstream buf;

    buf << 
        "#version " << GLSL_VERSION_STR << "\n"
        PRECISION_MEDIUMP_FLOAT "\n";

    buf << "varying vec4 osg_TexCoord[" << Registry::capabilities().getMaxGPUTextureCoordSets() << "]; \n";

    buf
        << "varying vec4 osg_FrontColor; \n"
        << "varying vec4 osg_FrontSecondaryColor; \n"
    
        << "void osgearth_vert_setupColoring() \n"
        << "{ \n"
        << "    osg_FrontColor = gl_Color; \n"
        << "    osg_FrontSecondaryColor = vec4(0.0); \n";

    //TODO: gl_TexCoord et.al. are depcrecated so we should replace them;
    // this approach also only support up to 8 texture coord units
    for(unsigned i=0; i<numTexCoordSets; ++i )
    {
        buf << "    osg_TexCoord["<< i <<"] = gl_MultiTexCoord"<< i << "; \n";
    }
        
    buf << "} \n";

    std::string str;
    str = buf.str();

    osg::Shader* shader = new osg::Shader(osg::Shader::VERTEX, str);
    shader->setName( VERTEX_SETUP_COLORING );
    return shader;
}


osg::Shader*
ShaderFactory::createDefaultColoringFragmentShader( unsigned numTexImageUnits ) const
{
    std::stringstream buf;

    buf << "#version " << GLSL_VERSION_STR << "\n"
        << PRECISION_MEDIUMP_FLOAT << "\n";
    
    buf << "varying vec4 osg_FrontColor; \n";
    
    if ( numTexImageUnits > 0 )
    {
        buf << "varying vec4 osg_TexCoord[" << Registry::capabilities().getMaxGPUTextureCoordSets() << "]; \n";
        buf << "uniform sampler2D ";
        for( unsigned i=0; i<numTexImageUnits; ++i )
        {
            buf << getSamplerName(i) << (i+1 < numTexImageUnits? "," : "; \n");
        }
    }

    buf << "void osgearth_frag_applyColoring( inout vec4 color ) \n"
        << "{ \n"
        << "    color = color * osg_FrontColor; \n";
    
    if ( numTexImageUnits > 0 )
    {
        buf << "    vec4 texel; \n";

        for(unsigned i=0; i<numTexImageUnits; ++i )
        {
            buf << "    texel = texture2D(" << getSamplerName(i) << ", osg_TexCoord["<< i <<"].st); \n";
            buf << "    color.rgb = mix( color.rgb, texel.rgb, texel.a ); \n";
            if ( i == 0 )
                buf << "    color.a = texel.a * color.a; \n";
        }
    }

    buf << "} \n";

    std::string str;
    str = buf.str();

    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
    shader->setName( FRAGMENT_APPLY_COLORING );
    return shader;
}

#ifdef GLENNS_PER_VERTEX_LIGHTING

osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    std::string str = Stringify() << 

        "#version " GLSL_VERSION_STR "\n"
        PRECISION_MEDIUMP_FLOAT "\n"

        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec4 oe_lighting_adjustment; \n"
        "varying vec4 oe_zero_vec; \n"

        "void osgearth_vert_setupLighting() \n"
        "{ \n"
        "    oe_lighting_adjustment = vec4(1.0); \n"
        "    if (oe_mode_GL_LIGHTING) \n"
        "    { \n"
        "        vec3 N = normalize(gl_NormalMatrix * gl_Normal); \n"
        "        float NdotL = dot( N, normalize(gl_LightSource[0].position.xyz) ); \n"
        "        NdotL = max( 0.0, NdotL ); \n"

        // NOTE: See comment in the fragment shader below for an explanation of
        //       this oe_zero_vec value.
        "        oe_zero_vec = vec4(0.0); \n"

        "        vec4 adj = \n"
        //"            gl_FrontLightModelProduct.sceneColor + \n" // not available in GLES yet
        "            gl_FrontLightProduct[0].ambient + \n"
        "            gl_FrontLightProduct[0].diffuse * NdotL; \n"
        "        oe_lighting_adjustment = clamp( adj, 0.0, 1.0 ); \n"
        "    } \n"
        "} \n";

    osg::Shader* shader = new osg::Shader( osg::Shader::VERTEX, str );
    shader->setName( VERTEX_SETUP_LIGHTING );
    return shader;
}


osg::Shader*
ShaderFactory::createDefaultLightingFragmentShader() const
{
    std::string str = Stringify() <<

        "#version " GLSL_VERSION_STR "\n"
        PRECISION_MEDIUMP_FLOAT "\n"

        "varying vec4 oe_lighting_adjustment; \n"
        "varying vec4 oe_zero_vec; \n"

         "uniform bool oe_mode_GL_LIGHTING; \n"
         "void osgearth_frag_applyLighting( inout vec4 color ) \n"
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
         "        color = color * oe_lighting_adjustment + oe_zero_vec; \n"
         "        color.a = alpha; \n"
         "    } \n"
        "} \n";

    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
    shader->setName( FRAGMENT_APPLY_LIGHTING );
    return shader;
}

#endif


#ifdef GLENNS_PER_FRAGMENT_LIGHTING // does not work on GLES - unresolved

osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    std::string str = Stringify() << 

        "#version " GLSL_VERSION_STR "\n"
        PRECISION_MEDIUMP_FLOAT "\n"

        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec3 oe_lighting_normal; \n"

        "void osgearth_vert_setupLighting() \n"
        "{ \n"
        "    if (oe_mode_GL_LIGHTING) \n"
        "    { \n"
        "        oe_lighting_normal = normalize(gl_NormalMatrix * gl_Normal); \n"
        "    } \n"
        "} \n";

    osg::Shader* shader = new osg::Shader( osg::Shader::VERTEX, str );
    shader->setName( VERTEX_SETUP_LIGHTING );
    return shader;
}


osg::Shader*
ShaderFactory::createDefaultLightingFragmentShader() const
{
    std::string str = Stringify() <<

        "#version " GLSL_VERSION_STR "\n"
        PRECISION_MEDIUMP_FLOAT "\n"

        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec3 oe_lighting_normal; \n"

         "void osgearth_frag_applyLighting( inout vec4 color ) \n"
         "{ \n"
         "    if ( oe_mode_GL_LIGHTING ) \n"
         "    { \n"
         "        float alpha = color.a; \n"
         "        vec3 n = normalize( oe_lighting_normal ); \n"
         "        float NdotL = dot( n, normalize(gl_LightSource[0].position.xyz) ); \n"
         "        NdotL = max( 0.0, NdotL ); \n"
         "        vec4 adjustment = \n"
         //"            gl_FrontLightModelProduct.sceneColor + \n" // not available in GLES yet
         "            gl_FrontLightProduct[0].ambient + \n"
         "            gl_FrontLightProduct[0].diffuse * NdotL; \n"
         "        color *= clamp(adjustment, 0.0, 1.0); \n"

         // specular highlights: (skip them for now)
         //"        float NdotHV = dot( n, gl_LightSource[0].halfVector.xyz ); \n"
         //"        NdotHV = max( 0.0, NdotHV ); \n"
         //"        if ( NdotL * NdotHV > 0.0 ) \n"
         //"            color += gl_FrontLightProduct[0].specular * \n"
         //"                     pow( NdotHV, gl_FrontMaterial.shininess ); \n"

         "        color.a = alpha; \n"
         "    } \n"
        "} \n";

    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
    shader->setName( FRAGMENT_APPLY_LIGHTING );
    return shader;
}

#endif // GLENNS_PER_FRAGMENT_LIGHTING


#ifdef TOMS_PER_VERTEX_LIGHTING

osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    int maxLights = Registry::capabilities().getMaxLights();
    
    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n";

    if ( s_GLES_SHADERS )
    {
        buf << "precision mediump float;\n"
            << osg_LightSourceParameters::glslDefinition() << "\n"
            << osg_LightProducts::glslDefinition() << "\n"
            << "uniform osg_LightSourceParameters osg_LightSource0;\n"
            << "uniform osg_LightProducts osg_FrontLightProduct0;\n";
    }
    
    buf
        << "varying vec4 osg_FrontColor; \n"
        << "varying vec4 osg_FrontSecondaryColor; \n"
        << "uniform bool oe_mode_GL_LIGHTING; \n";
    
    if ( s_GLES_SHADERS )
    {
        buf
            << "void osgearth_vert_setupLighting() \n"
            << "{ \n"
            << "    if (oe_mode_GL_LIGHTING) \n"
            << "    { \n"
            << "        float shine = 10.0;\n"
            << "        vec4 lightModelAmbi = vec4(0.1,0.1,0.1,1.0);\n"
            //gl_FrontMaterial.shininess
            //gl_LightModel.ambient
            << "        vec3 normal = gl_NormalMatrix * gl_Normal; \n"
            << "        float NdotL = dot( normal, normalize(osg_LightSource0.position.xyz) ); \n"
            << "        NdotL = max( 0.0, NdotL ); \n"
            << "        float NdotHV = dot( normal, osg_LightSource0.halfVector.xyz ); \n"
            << "        NdotHV = max( 0.0, NdotHV ); \n"
            
            << "        osg_FrontColor.rgb = osg_FrontColor.rgb * \n"
            << "            clamp( \n"
            << "                lightModelAmbi + \n"
            << "                osg_FrontLightProduct0.ambient +          \n"
            << "                osg_FrontLightProduct0.diffuse * NdotL, 0.0, 1.0).rgb;   \n"
            
            << "        osg_FrontSecondaryColor = vec4(0.0); \n"
            
            << "        if ( NdotL * NdotHV > 0.0 ) \n"
            << "        { \n"
            << "            osg_FrontSecondaryColor.rgb = (osg_FrontLightProduct0.specular * \n"
            << "                                          pow( NdotHV, shine )).rgb;\n"
            << "        } \n"
            << "    } \n"
            << "} \n";
    }
    else // !s_GLES_SHADERS
    {
        buf
            << "void osgearth_vert_setupLighting() \n"
            << "{ \n"
            << "    if (oe_mode_GL_LIGHTING) \n"
            << "    { \n"
            << "        vec3 normal = gl_NormalMatrix * gl_Normal; \n"
            << "        float NdotL = dot( normal, normalize(gl_LightSource[0].position.xyz) ); \n"
            << "        NdotL = max( 0.0, NdotL ); \n"
            << "        float NdotHV = dot( normal, gl_LightSource[0].halfVector.xyz ); \n"
            << "        NdotHV = max( 0.0, NdotHV ); \n"

            << "        osg_FrontColor.rgb = osg_FrontColor.rgb * \n"
            << "            clamp( \n"
            << "                gl_LightModel.ambient + \n"
            << "                gl_FrontLightProduct[0].ambient +          \n"
            << "                gl_FrontLightProduct[0].diffuse * NdotL, 0.0, 1.0).rgb;   \n"

            << "        osg_FrontSecondaryColor = vec4(0.0); \n"
            << "        if ( NdotL * NdotHV > 0.0 ) \n"
            << "        { \n"
            << "            osg_FrontSecondaryColor.rgb = (gl_FrontLightProduct[0].specular * \n"
            << "                                          pow( NdotHV, gl_FrontMaterial.shininess )).rgb;\n"
            << "        } \n"
            << "    } \n"
            << "} \n";
    }

    osg::Shader* shader = new osg::Shader( osg::Shader::VERTEX, buf.str().c_str() );
    shader->setName( VERTEX_SETUP_LIGHTING );
    return shader;
}


osg::Shader*
ShaderFactory::createDefaultLightingFragmentShader() const
{
    std::stringstream buf;
    
    buf << "#version " << GLSL_VERSION_STR << "\n"
        << PRECISION_MEDIUMP_FLOAT << "\n"
    
    << "varying vec4 osg_FrontColor; \n"
    << "varying vec4 osg_FrontSecondaryColor; \n"
    
    << "uniform bool oe_mode_GL_LIGHTING; \n"
    << "void osgearth_frag_applyLighting( inout vec4 color ) \n"
    << "{ \n"
    << "    if ( oe_mode_GL_LIGHTING ) \n"
    << "    { \n"
    << "        float alpha = color.a; \n"
    << "        color = (color * osg_FrontColor) + osg_FrontSecondaryColor; \n"
    << "        color.a = alpha; \n"
    << "    } \n"
    << "} \n";

    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, buf.str().c_str() );
    shader->setName( FRAGMENT_APPLY_LIGHTING );
    return shader;
}

#endif // TOMS_PER_VERTEX_LIGHTING


osg::Shader*
ShaderFactory::createColorFilterChainFragmentShader( const std::string& function, const ColorFilterChain& chain ) const
{
    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n"
        << PRECISION_MEDIUMP_FLOAT << "\n";

    // write out the shader function prototypes:
    for( ColorFilterChain::const_iterator i = chain.begin(); i != chain.end(); ++i )
    {
        ColorFilter* filter = i->get();
        buf << "void " << filter->getEntryPointFunctionName() << "(in int slot, inout vec4 color);\n";
    }

    // write out the main function:
    buf << "void " << function << "(in int slot, inout vec4 color) \n"
        << "{ \n";

    // write out the function calls. if there are none, it's a NOP.
    for( ColorFilterChain::const_iterator i = chain.begin(); i != chain.end(); ++i )
    {
        ColorFilter* filter = i->get();
        buf << "    " << filter->getEntryPointFunctionName() << "(slot, color);\n";
    }
        
    buf << "} \n";

    std::string bufstr;
    bufstr = buf.str();
    return new osg::Shader(osg::Shader::FRAGMENT, bufstr);
}


osg::Uniform*
ShaderFactory::createUniformForGLMode(osg::StateAttribute::GLMode      mode,
                                      osg::StateAttribute::GLModeValue value)
{
    osg::Uniform* u = 0L;

    if ( mode == GL_LIGHTING )
    {
        osg::Uniform* u = new osg::Uniform(osg::Uniform::BOOL, "oe_mode_GL_LIGHTING");
        u->set( (value & osg::StateAttribute::ON) != 0 );
    }

    return u;
}
