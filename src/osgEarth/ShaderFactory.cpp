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
#ifdef OSG_GLES2_AVAILABLE
        << "precision mediump float;\n"
#endif
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
    //OE_INFO << str << std::endl;
    return new osg::Shader( osg::Shader::VERTEX, str );
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
#ifdef OSG_GLES2_AVAILABLE
        << "precision mediump float;\n"
#endif
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
        << "    vec4 color = vec4(1,1,1,1); \n"; //gl_Color; \n"; //vec4(1,1,1,1); \n";

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
    //OE_INFO << str;
    return new osg::Shader( osg::Shader::FRAGMENT, str );
}
 

osg::Shader*
ShaderFactory::createDefaultColoringVertexShader( unsigned numTexCoordSets ) const
{
    std::stringstream buf;

    buf << "#version " << GLSL_VERSION_STR << "\n";
#ifdef OSG_GLES2_AVAILABLE
    buf << "precision mediump float;\n";
#endif
    
    //if ( numTexCoordSets > 0 )
    //{
    //    buf << "varying vec4 osg_TexCoord[" << numTexCoordSets << "];\n";
    //}
    buf << "varying vec4 osg_TexCoord[" << Registry::capabilities().getMaxGPUTextureCoordSets() << "];\n";

    buf
        << "varying vec4 osg_FrontColor;\n"
        << "varying vec4 osg_FrontSecondaryColor;\n"
    
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

    buf << "#version " << GLSL_VERSION_STR << "\n";
#ifdef OSG_GLES2_AVAILABLE
    buf << "precision mediump float;\n";
#endif
    
    buf << "varying vec4 osg_FrontColor;\n";
    
    if ( numTexImageUnits > 0 )
    {
        buf << "varying vec4 osg_TexCoord[" << Registry::capabilities().getMaxGPUTextureCoordSets() << "];\n";
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

#if 1

osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n"

        << "uniform bool osgearth_LightingEnabled; \n"
        << "varying vec3 oe_lighting_normal; \n"
        << "varying vec3 oe_lighting_vertex_eye; \n"

        << "void osgearth_vert_setupLighting() \n"
        << "{ \n"
        << "    if (osgearth_LightingEnabled) \n"
        << "    { \n"
        << "        oe_lighting_normal     = normalize(gl_NormalMatrix * gl_Normal); \n"
        << "        oe_lighting_vertex_eye = vec3(gl_ModelViewMatrix * gl_Vertex); \n"
        << "    } \n"
        << "} \n";

    std::string str;
    str = buf.str();

    osg::Shader* shader = new osg::Shader( osg::Shader::VERTEX, str );
    shader->setName( VERTEX_SETUP_LIGHTING );
    return shader;
}


osg::Shader*
ShaderFactory::createDefaultLightingFragmentShader() const
{
    std::stringstream buf;

#ifndef OSG_GLES2_AVAILABLE  // Regular GLSL version:

    buf << "#version " << GLSL_VERSION_STR << "\n"

        << "uniform bool osgearth_LightingEnabled; \n"
        << "varying vec3 oe_lighting_normal; \n"
        << "varying vec3 oe_lighting_vertex_eye; \n"

        << "void osgearth_frag_applyLighting( inout vec4 color ) \n"
        << "{ \n"
        << "    if ( osgearth_LightingEnabled ) \n"
        << "    { \n"
                "    float alpha = color.a; \n"
                "    vec3 n = normalize( oe_lighting_normal ); \n"
                "    float NdotL = dot( n, normalize(gl_LightSource[0].position.xyz) ); \n"
                "    NdotL = max( 0.0, NdotL ); \n"
                "    float NdotHV = dot( n, gl_LightSource[0].halfVector.xyz ); \n"
                "    NdotHV = max( 0.0, NdotHV ); \n"
                "    vec4 adjustment = \n"
                "        gl_FrontLightModelProduct.sceneColor + \n"
                "        gl_FrontLightProduct[0].ambient + \n"
                "        gl_FrontLightProduct[0].diffuse * NdotL; \n"
                "    color *= clamp(adjustment, 0.0, 1.0); \n"
                "    if ( NdotL * NdotHV > 0.0 ) \n"
                "        color += gl_FrontLightProduct[0].specular * \n"
                "                 pow( NdotHV, gl_FrontMaterial.shininess ); \n"
                "    color.a = alpha; \n"
        << "    } \n"
        << "} \n";

#else // GLES2 version:

    buf << "#version " << GLSL_VERSION_STR << "\n"
        << "precision mediump float;\n"
        
        //add lightsource typedef and uniform array
        << "struct osg_LightSourceParameters {"
        << "    vec4   ambient;"
        << "    vec4   diffuse;"
        << "    vec4   specular;"
        << "    vec4   position;"
        << "    vec4   halfVector;"
        << "    vec3   spotDirection;" 
        << "    float  spotExponent;"
        << "    float  spotCutoff;"
        << "    float  spotCosCutoff;" 
        << "    float  constantAttenuation;"
        << "    float  linearAttenuation;"
        << "    float  quadraticAttenuation;" 
        << "};\n"
        << "uniform osg_LightSourceParameters osg_LightSource[" << maxLights << "];\n"
        
        << "struct  osg_LightProducts {"
        << "    vec4  ambient;"
        << "    vec4  diffuse;"
        << "    vec4  specular;"
        << "};\n"
        << "uniform osg_LightProducts osg_FrontLightProduct[" << maxLights << "];\n"

        //<< "struct osg_LightModelProducs {"
        //<< "    vec4 sceneColor;"
        //<< "};\n"
        //<< "uniform osg_LightModelProducts osg_FrontLightModelProduct; \n"

        << "uniform bool osgearth_LightingEnabled; \n"
        << "varying vec3 oe_lighting_normal; \n"
        << "varying vec3 oe_lighting_vertex_eye; \n"

        << "void osgearth_frag_applyLighting( inout vec4 color ) \n"
        << "{ \n"
        << "    if ( osgearth_LightingEnabled ) \n"
        << "    { \n"
                "    float alpha = color.a; \n"
                "    float FrontMaterial_shininess = 10.0; \n" // just set a constant
                "    vec3 n = normalize( oe_lighting_normal ); \n"
                "    float NdotL = dot( n, normalize(osg_LightSource[0].position.xyz) ); \n"
                "    NdotL = max( 0.0, NdotL ); \n"
                "    float NdotHV = dot( n, osg_LightSource[0].halfVector.xyz ); \n"
                "    NdotHV = max( 0.0, NdotHV ); \n"
                "    vec4 adjustment = \n"
                //"        osg_FrontLightModelProduct.sceneColor + \n" // dont' have this in GLES adapter yet
                "        gl_FrontLightProduct[0].ambient + \n"
                "        gl_FrontLightProduct[0].diffuse * NdotL; \n"
                "    color *= clamp(adjustment, 0.0, 1.0); \n"
                "    if ( NdotL * NdotHV > 0.0 ) \n"
                "        color += osg_FrontLightProduct[0].specular * \n"
                "                 pow( NdotHV, FrontMaterial_shininess ); \n"
                "    color.a = alpha; \n"
        << "    } \n"
        << "} \n";

#endif // OSG_GLES2_AVAILABLE

    std::string str;
    str = buf.str();

    osg::Shader* shader = new osg::Shader( osg::Shader::FRAGMENT, str );
    shader->setName( FRAGMENT_APPLY_LIGHTING );
    return shader;

}

#else //...................................

osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    int maxLights = Registry::capabilities().getMaxLights();
    
    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
    << "precision mediump float;\n"
    
    //add lightsource typedef and uniform array
    << "struct osg_LightSourceParameters {"
    << "    vec4  ambient;"
    << "    vec4  diffuse;"
    << "    vec4  specular;"
    << "    vec4  position;"
    << "    vec4  halfVector;"
    << "    vec3  spotDirection;" 
    << "    float  spotExponent;"
    << "    float  spotCutoff;"
    << "    float  spotCosCutoff;" 
    << "    float  constantAttenuation;"
    << "    float  linearAttenuation;"
    << "    float  quadraticAttenuation;" 
    << "};\n"
    << "uniform osg_LightSourceParameters osg_LightSource[" << maxLights << "];\n"
    
    << "struct  osg_LightProducts {"
    << "    vec4  ambient;"
    << "    vec4  diffuse;"
    << "    vec4  specular;"
    << "};\n"
    << "uniform osg_LightProducts osg_FrontLightProduct[" << maxLights << "];\n"
    
#endif
    
    << "varying vec4 osg_FrontColor;\n"
    << "varying vec4 osg_FrontSecondaryColor;\n"
    
    << "uniform bool osgearth_LightingEnabled; \n"
    
#ifndef OSG_GLES2_AVAILABLE
    << "void osgearth_vert_setupLighting() \n"
    << "{ \n"
    << "    if (osgearth_LightingEnabled) \n"
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
#else // if OSG_GLES2_AVAILABLE
    << "void osgearth_vert_setupLighting() \n"
    << "{ \n"
    << "    if (osgearth_LightingEnabled) \n"
    << "    { \n"
    << "        float shine = 10.0;\n"
    << "        vec4 lightModelAmbi = vec4(0.1,0.1,0.1,1.0);\n"
//gl_FrontMaterial.shininess
//gl_LightModel.ambient
    << "        vec3 normal = gl_NormalMatrix * gl_Normal; \n"
    << "        float NdotL = dot( normal, normalize(osg_LightSource[0].position.xyz) ); \n"
    << "        NdotL = max( 0.0, NdotL ); \n"
    << "        float NdotHV = dot( normal, osg_LightSource[0].halfVector.xyz ); \n"
    << "        NdotHV = max( 0.0, NdotHV ); \n"
    
    << "        osg_FrontColor.rgb = osg_FrontColor.rgb * \n"
    << "            clamp( \n"
    << "                lightModelAmbi + \n"
    << "                osg_FrontLightProduct[0].ambient +          \n"
    << "                osg_FrontLightProduct[0].diffuse * NdotL, 0.0, 1.0).rgb;   \n"
    
    << "        osg_FrontSecondaryColor = vec4(0.0); \n"
    
    << "        if ( NdotL * NdotHV > 0.0 ) \n"
    << "        { \n"
    << "            osg_FrontSecondaryColor.rgb = (osg_FrontLightProduct[0].specular * \n"
    << "                                          pow( NdotHV, shine )).rgb;\n"
    << "        } \n"
    << "    } \n"
    << "} \n";
#endif

    osg::Shader* shader = new osg::Shader( osg::Shader::VERTEX, buf.str().c_str() );
    shader->setName( VERTEX_SETUP_LIGHTING );
    return shader;
}


osg::Shader*
ShaderFactory::createDefaultLightingFragmentShader() const
{
    std::stringstream buf;
    
    buf << "#version " << GLSL_VERSION_STR << "\n"
#ifdef OSG_GLES2_AVAILABLE
    << "precision mediump float;\n"
#endif
    
    << "varying vec4 osg_FrontColor;\n"
    << "varying vec4 osg_FrontSecondaryColor;\n"
    
    << "uniform bool osgearth_LightingEnabled; \n"
    << "void osgearth_frag_applyLighting( inout vec4 color ) \n"
    << "{ \n"
    << "    if ( osgearth_LightingEnabled ) \n"
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
#endif // ........................................................

osg::Shader*
ShaderFactory::createColorFilterChainFragmentShader( const std::string& function, const ColorFilterChain& chain ) const
{
    std::stringstream buf;
    buf << "#version " << GLSL_VERSION_STR << "\n";
#ifdef OSG_GLES2_AVAILABLE
    buf << "precision mediump float;\n";
#endif

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


//--------------------------------------------------------------------------

#if 0
// This is just a holding pen for various stuff

static char s_PerFragmentLighting_VertexShaderSource[] =
    "varying vec3 Normal; \n"
    "varying vec3 Position; \n"
    "void osgearth_vert_setupLighting() \n"
    "{ \n"
    "    Normal = normal; \n"
    "    Position = position; \n"
    "} \n";

static char s_PerFragmentDirectionalLighting_FragmentShaderSource[] =
    "varying vec3 Normal; \n"
    "varying vec3 Position; \n"
    "void osgearth_frag_applyLighting( inout vec4 color ) \n"
    "{ \n"
    "    vec3 n = normalize( Normal ); \n"
    "    float NdotL = dot( n, normalize(gl_LightSource[0].position.xyz) ); \n"
    "    NdotL = max( 0.0, NdotL ); \n"
    "    float NdotHV = dot( n, gl_LightSource[0].halfVector.xyz ); \n"
    "    NdotHV = max( 0.0, NdotHV ); \n"
    "    color *= gl_FrontLightModelProduct.sceneColor + \n"
    "             gl_FrontLightProduct[0].ambient + \n"
    "             gl_FrontLightProduct[0].diffuse * NdotL; \n"
    "    if ( NdotL * NdotHV > 0.0 ) \n"
    "        color += gl_FrontLightProduct[0].specular * \n"
    "                 pow( NdotHV, gl_FrontMaterial.shininess ); \n"
    "} \n";

#endif
