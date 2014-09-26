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

#include <osgEarth/PhongLightingEffect>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderFactory>
#include <osgEarth/StringUtils>
#include <osgEarth/VirtualProgram>

using namespace osgEarth;

namespace
{
#ifdef OSG_GLES2_AVAILABLE
    static const char* Phong_Vertex =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec4 oe_lighting_adjustment; \n"
        "varying vec4 oe_lighting_zero_vec; \n"
        "varying vec3 oe_Normal; \n"

        "void oe_phong_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    if ( oe_mode_GL_LIGHTING == false ) return; \n"
        "    oe_lighting_adjustment = vec4(1.0); \n"
        "    vec3 N = oe_Normal; \n"
        "    float NdotL = dot( N, normalize(gl_LightSource[0].position.xyz) ); \n"
        "    NdotL = max( 0.0, NdotL ); \n"

        // NOTE: See comment in the fragment shader below for an explanation of
        //       this oe_zero_vec value.
        "    oe_lighting_zero_vec = vec4(0.0); \n"

        "    vec4 adj = \n"
        "        gl_FrontLightProduct[0].ambient + \n"
        "        gl_FrontLightProduct[0].diffuse * NdotL; \n"
        "    oe_lighting_adjustment = clamp( adj, 0.0, 1.0 ); \n"
        "} \n";

    static const char* Phong_Fragment =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        
        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec4 oe_lighting_adjustment; \n"
        "varying vec4 oe_lighting_zero_vec; \n"

        "void oe_phong_fragment(inout vec4 color) \n"
        "{ \n"
        "    if ( oe_mode_GL_LIGHTING == false ) return; \n"
        //NOTE: The follow was changed from the single line
        //      "color *= oe_lighting_adjustment" to the current code to fix
        //      an issue on iOS devices.  Adding a varying vec4 value set to
        //      (0.0,0.0,0.0,0.0) to the color should not make a difference,
        //      but it is part of the solution to the issue we were seeing.
        //      Without it and the additional lines of code, the globe was
        //      rendering textureless (just a white surface with lighting).
        "    float alpha = color.a; \n"
        "    color = color * oe_lighting_adjustment + oe_lighting_zero_vec; \n"
        "    color.a = alpha; \n"
        "} \n";

#else

    static const char* Phong_Vertex =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        
        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec3 oe_phong_vertexView3; \n"

        "void oe_phong_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        "    if ( oe_mode_GL_LIGHTING == false ) return; \n"
        "    oe_phong_vertexView3 = VertexVIEW.xyz / VertexVIEW.w; \n"
        "} \n";

    static const char* Phong_Fragment =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "uniform bool oe_mode_GL_LIGHTING; \n"
        "varying vec3 oe_phong_vertexView3; \n"
        "varying vec3 oe_Normal; \n"

        "void oe_phong_fragment(inout vec4 color) \n"
        "{ \n"        
        "    if ( oe_mode_GL_LIGHTING == false ) return; \n"

        "    vec3 L = normalize(gl_LightSource[0].position.xyz); \n"
        "    vec3 N = normalize(oe_Normal); \n"
        
        "    vec4 ambient = gl_FrontLightProduct[0].ambient; \n"

        "    float NdotL = max(dot(N,L), 0.0); \n"

        "    vec4 diffuse = gl_FrontLightProduct[0].diffuse * NdotL; \n"
        
        "    vec4 specular= vec4(0); \n"
        "    if (NdotL > 0.0) \n"
        "    { \n"
        "        vec3 V = normalize(oe_phong_vertexView3); \n"
        "        vec3 H = normalize(L-V); \n"
        "        float HdotN = max(dot(H,N), 0.0); \n"
        "        float shine = clamp(gl_FrontMaterial.shininess, 1.0, 128.0); \n"
        "        specular = gl_FrontLightProduct[0].specular * pow(HdotN, shine); \n"
        "    } \n"

        "    color.rgb *= ambient.rgb + diffuse.rgb + specular.rgb; \n"
        "} \n";
#endif
}

PhongLightingEffect::PhongLightingEffect()
{
    init();
}

PhongLightingEffect::PhongLightingEffect(osg::StateSet* stateset)
{
    init();
    attach( stateset );
}

void
PhongLightingEffect::init()
{
    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        _lightingUniform = Registry::shaderFactory()->createUniformForGLMode( GL_LIGHTING, 1 );
    }
}

void
PhongLightingEffect::setCreateLightingUniform(bool value)
{
    if ( !value )
    {        
        _lightingUniform = 0L;
    }
}

PhongLightingEffect::~PhongLightingEffect()
{
    detach();
}

void
PhongLightingEffect::attach(osg::StateSet* stateset)
{
    if ( stateset && _supported )
    {
        _statesets.push_back(stateset);
        VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
        vp->setName( "osgEarth.PhongLightingEffect" );
        vp->setFunction( "oe_phong_vertex", Phong_Vertex, ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( "oe_phong_fragment", Phong_Fragment, ShaderComp::LOCATION_FRAGMENT_LIGHTING );
        if ( _lightingUniform.valid() )
            stateset->addUniform( _lightingUniform.get() );
    }
}

void
PhongLightingEffect::detach()
{
    if ( _supported )
    {
        for (StateSetList::iterator it = _statesets.begin(); it != _statesets.end(); ++it)
        {
            osg::ref_ptr<osg::StateSet> stateset;
            if ( (*it).lock(stateset) )
            {
                detach( stateset );
                (*it) = 0L;
            }
        }

        _statesets.clear();
    }
}

void
PhongLightingEffect::detach(osg::StateSet* stateset)
{
    if ( stateset && _supported )
    {
        if ( _lightingUniform.valid() )
            stateset->removeUniform( _lightingUniform.get() );

        VirtualProgram* vp = VirtualProgram::get( stateset );
        if ( vp )
        {
            vp->removeShader( "oe_phong_vertex" );
            vp->removeShader( "oe_phong_fragment" );
        }
    }
}
