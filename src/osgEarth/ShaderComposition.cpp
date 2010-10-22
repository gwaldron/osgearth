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
#include <osgEarth/ShaderComposition>

using namespace osgEarth;

#include <osg/Shader>
#include <osg/Program>
#include <osg/State>
#include <osg/Notify>
#include <sstream>

#define LC "[VirtualProgram] "

//------------------------------------------------------------------------

namespace
{
    /** A hack for OSG 2.8.x to get access to the state attribute vector. */
    class StateHack : public osg::State 
    {
    public:        
        typedef std::pair<const osg::StateAttribute*,osg::StateAttribute::OverrideValue> AttributePair;
        typedef std::vector<AttributePair> AttributeVec;

        AttributeVec& getAttributeVec( const osg::StateAttribute* attribute ) 
        {
            AttributeStack& as = _attributeMap[ attribute->getTypeMemberPair() ];
            return as.attributeVec; 
        }

        static AttributeVec& GetAttributeVec( osg::State& state, const osg::StateAttribute* attribute ) 
        {
            StateHack* sh = reinterpret_cast< StateHack* >( &state );
            return sh->getAttributeVec( attribute );
        }
    };
}

//------------------------------------------------------------------------

// If graphics board has program linking problems set MERGE_SHADERS to 1
// Merge shaders can be used to merge shaders strings into one shader. 
#define MERGE_SHADERS 0
#define NOTIFICATION_MESSAGES 0

VirtualProgram::VirtualProgram( unsigned int mask ) : 
_mask( mask ) 
{
    //nop
}

VirtualProgram::VirtualProgram(const VirtualProgram& VirtualProgram, const osg::CopyOp& copyop ) :
osg::Program( VirtualProgram, copyop ),
_shaderMap( VirtualProgram._shaderMap ),
_mask( VirtualProgram._mask )
{
    //nop
}

osg::Shader*
VirtualProgram::getShader( const std::string& shaderSemantic, osg::Shader::Type type )
{
    ShaderMap::key_type key( shaderSemantic, type );
    return _shaderMap[ key ].get();
}

osg::Shader*
VirtualProgram::setShader( const std::string& shaderSemantic, osg::Shader * shader )
{
    if( shader->getType() == osg::Shader::UNDEFINED ) 
        return NULL;

    ShaderMap::key_type key( shaderSemantic, shader->getType() );

    osg::ref_ptr< osg::Shader >  shaderNew     = shader;
    osg::ref_ptr< osg::Shader >& shaderCurrent = _shaderMap[ key ];

#if 0 // Good for debugging of shader linking problems. 
      // Don't do it - User could use the name for its own purposes 
    shaderNew->setName( shaderSemantic );
#endif

    if( shaderCurrent != shaderNew ) {
#if 0
       if( shaderCurrent.valid() )
           Program::removeShader( shaderCurrent.get() );

       if( shaderNew.valid() )
           Program::addShader( shaderNew.get() );
#endif
       shaderCurrent = shaderNew;
    }

    return shaderCurrent.get();
}

void
VirtualProgram::apply( osg::State & state ) const
{
    if( _shaderMap.empty() ) // Virtual Program works as normal Program
        return Program::apply( state );

    StateHack::AttributeVec* av = &StateHack::GetAttributeVec( state, this );
    //osg::State::AttributeVec* av = &state.getAttributeVec(this);

#if NOTIFICATION_MESSAGES
    std::ostream &os  = osg::notify( osg::NOTICE );
    os << "VirtualProgram cumulate Begin" << std::endl;
#endif

    ShaderMap shaderMap;
    for( StateHack::AttributeVec::iterator i = av->begin(); i != av->end(); ++i )
    {
        const osg::StateAttribute* sa = i->first;
        const VirtualProgram* vp = dynamic_cast< const VirtualProgram* >( sa );
        if( vp && ( vp->_mask & _mask ) ) {

#if NOTIFICATION_MESSAGES
            if( vp->getName().empty() )
                os << "VirtualProgram cumulate [ Unnamed VP ] apply" << std::endl;
            else 
                os << "VirtualProgram cumulate ["<< vp->getName() << "] apply" << std::endl;
#endif

            for( ShaderMap::const_iterator i = vp->_shaderMap.begin();
                                           i != vp->_shaderMap.end(); ++i )
            {
                                                    shaderMap[ i->first ] = i->second;
            }

        } else {
#if NOTIFICATION_MESSAGES
            os << "VirtualProgram cumulate ( not VP or mask not match ) ignored" << std::endl;
#endif
            continue; // ignore osg::Programs
        }
    }

    for( ShaderMap::const_iterator i = this->_shaderMap.begin();
                                   i != this->_shaderMap.end(); ++i )
                                        shaderMap[ i->first ] = i->second;

#if NOTIFICATION_MESSAGES
    os << "VirtualProgram cumulate End" << std::endl;
#endif

    if( shaderMap.size() ) {

        ShaderList sl;
        for( ShaderMap::iterator i = shaderMap.begin(); i != shaderMap.end(); ++i )
            sl.push_back( i->second );

        osg::ref_ptr< osg::Program > & program = _programMap[ sl ];

        if( !program.valid() ) {
            program = new osg::Program;
#if !MERGE_SHADERS
            for( ShaderList::iterator i = sl.begin(); i != sl.end(); ++i )
                program->addShader( i->get() );
#else
            std::string strFragment;
            std::string strVertex;
            std::string strGeometry;
            
            for( ShaderList::iterator i = sl.begin(); i != sl.end(); ++i )
            {
                if( i->get()->getType() == osg::Shader::FRAGMENT )
                    strFragment += i->get()->getShaderSource();
                else if ( i->get()->getType() == osg::Shader::VERTEX )
                    strVertex += i->get()->getShaderSource();
                else if ( i->get()->getType() == osg::Shader::GEOMETRY )
                    strGeometry += i->get()->getShaderSource();
            }

            if( strFragment.length() > 0 ) {
                program->addShader( new osg::Shader( osg::Shader::FRAGMENT, strFragment ) );
#if NOTIFICATION_MESSAGES
                os << "====VirtualProgram merged Fragment Shader:"  << std::endl << strFragment << "====" << std::endl;
#endif
            }

            if( strVertex.length() > 0  ) {
                program->addShader( new osg::Shader( osg::Shader::VERTEX, strVertex ) );
#if NOTIFICATION_MESSAGES
                os << "VirtualProgram merged Vertex Shader:"  << std::endl << strVertex << "====" << std::endl;
#endif
            }

            if( strGeometry.length() > 0  ) {
                program->addShader( new osg::Shader( osg::Shader::GEOMETRY, strGeometry ) );
#if NOTIFICATION_MESSAGES
                os << "VirtualProgram merged Geometry Shader:"  << std::endl << strGeometry << "====" << std::endl;
#endif
            }
#endif
        }

        state.applyAttribute( program.get() );
    } else {
        Program::apply( state );
    }

#if NOTIFICATION_MESSAGES
    os << "VirtualProgram Apply" << std::endl;
#endif

}

//----------------------------------------------------------------------------

static char s_PerVertexLighting_VertexShaderSource[] = 
"void osgearth_vert_lighting( in vec3 position, in vec3 normal )                 \n" //1
"{                                                                          \n" //2
"    float NdotL = dot( normal, normalize(gl_LightSource[0].position.xyz) );\n" //3
"    NdotL = max( 0.0, NdotL );                                             \n" //4
"    float NdotHV = dot( normal, gl_LightSource[0].halfVector.xyz );        \n" //5
"    NdotHV = max( 0.0, NdotHV );                                           \n" //6
"                                                                           \n" //7
"    gl_FrontColor = gl_FrontLightModelProduct.sceneColor +                 \n" //8
"                    gl_FrontLightProduct[0].ambient +                      \n" //9
"                    gl_FrontLightProduct[0].diffuse * NdotL;               \n" //10
"                                                                           \n" //11
"    gl_FrontSecondaryColor = vec4(0.0);                                    \n" //12
"                                                                           \n" //13
"    if ( NdotL * NdotHV > 0.0 )                                            \n" //14
"        gl_FrontSecondaryColor = gl_FrontLightProduct[0].specular *        \n" //15
"                                 pow( NdotHV, gl_FrontMaterial.shininess );\n" //16
"                                                                           \n" //17
"    gl_BackColor = gl_FrontColor;                                          \n" //18
"    gl_BackSecondaryColor = gl_FrontSecondaryColor;                        \n" //19
"}                                                                          \n";//5

static char s_PerVertexLighting_FragmentShaderSource[] =
"void osgearth_frag_lighting( inout vec4 color )                                 \n" //1
"{                                                                          \n" //2
"    color = color * gl_Color + gl_SecondaryColor;                          \n" //3
"}                                                                         \n";//20

static char s_Main_FragmentShaderSource[] =
"vec4 osgearth_frag_texture( void );                                                      \n" //1
"void osgearth_frag_lighting( inout vec4 color );                                         \n" //2
"uniform bool osgearth_lighting_enabled; \n"
"                                                                           \n" //3
"void main ()                                                               \n" //4
"{                                                                          \n" //5
"    vec4 color = osgearth_frag_texture();                                                \n" //6
"    if ( osgearth_lighting_enabled ) \n"
"        osgearth_frag_lighting( color );                                                     \n" //7
"    gl_FragColor = color;                                                  \n" //8
"}                                                                          \n";//9

static char s_SimpleTexture_FragmentShaderSource[] =
"uniform sampler2D tex0;                                                    \n" //1
"vec4 osgearth_frag_texture( void )                                         \n" //2
"{                                                                          \n" //3
"    return texture2D( tex0, gl_TexCoord[0].xy );                           \n" //4
"}                                                                          \n";//4

static char s_PerFragmentLighting_VertexShaderSource[] =
"varying vec3 Normal;                                                       \n" //1
"varying vec3 Position;                                                     \n" //2
"                                                                           \n" //3
"void osgearth_vert_lighting( in vec3 position, in vec3 normal )            \n" //4
"{                                                                          \n" //5
"    Normal = normal;                                                       \n" //6
"    Position = position;                                                   \n" //7
"}                                                                          \n";//8

static char s_PerFragmentDirectionalLighting_FragmentShaderSource[] =
"varying vec3 Normal;                                                       \n" //1
"varying vec3 Position; // not used for directional lighting                \n" //2
"                                                                           \n" //3
"void osgearth_frag_lighting( inout vec4 color )                            \n" //4
"{                                                                          \n" //5
"    vec3 n = normalize( Normal );                                          \n" //5
"    float NdotL = dot( n, normalize(gl_LightSource[0].position.xyz) );     \n" //6
"    NdotL = max( 0.0, NdotL );                                             \n" //7
"    float NdotHV = dot( n, gl_LightSource[0].halfVector.xyz );             \n" //8
"    NdotHV = max( 0.0, NdotHV );                                           \n" //9
"                                                                           \n" //10
"    color *= gl_FrontLightModelProduct.sceneColor +                        \n" //11
"             gl_FrontLightProduct[0].ambient +                             \n" //12
"             gl_FrontLightProduct[0].diffuse * NdotL;                      \n" //13
"                                                                           \n" //14
"    if ( NdotL * NdotHV > 0.0 )                                            \n" //15
"        color += gl_FrontLightProduct[0].specular *                        \n" //16
"                 pow( NdotHV, gl_FrontMaterial.shininess );                \n" //17
"}                                                                          \n";//18


//------------------------------------------------------------------------

osg::Shader*
ShaderFactory::createVertexShaderMain( bool hasPreprocess, bool hasPostprocess ) const
{
    std::stringstream buf;
    buf << "void osgearth_vert_texture( in vec3 position, in vec3 normal ); \n"
        << "void osgearth_vert_lighting( in vec3 position, in vec3 normal ); \n"
        << "uniform bool osgearth_lighting_enabled; \n";

    if ( hasPreprocess )
        buf << "void preprocess( in vec3 position, in vec3 normal ); \n";
    if ( hasPostprocess )
        buf << "void postprocess( in vec3 position, in vec3 normal ); \n";

    buf << "void main(void) \n"
        << "{ \n"
        <<     "gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        <<     "vec4 position4 = gl_ModelViewMatrix * gl_Vertex; \n"
        <<     "vec3 position = position4.xyz / position4.w; \n"
        <<     "vec3 normal = normalize( gl_NormalMatrix * gl_Normal ); \n";

    if ( hasPreprocess )
        buf << "preprocess( position, normal ); \n";

    buf <<    "osgearth_vert_texture( position, normal ); \n"
        <<    "if ( osgearth_lighting_enabled ) \n"
        <<    "    osgearth_vert_lighting( position, normal ); \n";
    
    if ( hasPostprocess )
        buf << "postprocess( position, normal ); \n";

    buf << "} \n";

    std::string str = buf.str();
    return new osg::Shader( osg::Shader::VERTEX, str );
}

osg::Shader*
ShaderFactory::createFragmentShaderMain( bool hasPreprocess, bool hasPostprocess ) const
{
    return new osg::Shader( osg::Shader::FRAGMENT, s_Main_FragmentShaderSource );
}

osg::Shader*
ShaderFactory::createDefaultTextureVertexShader( int numTexCoordSets ) const
{
    std::stringstream buf;

    buf << "void osgearth_vert_texture( in vec3 position, in vec3 normal ) \n"
        << "{ \n";

    for(int i=0; i<numTexCoordSets; ++i )
    {
        buf << "gl_TexCoord["<< i <<"] = gl_MultiTexCoord"<< i << "; \n";
    }
        
    buf << "} \n";

    std::string str = buf.str();
    return new osg::Shader( osg::Shader::VERTEX, str );
}

osg::Shader*
ShaderFactory::createDefaultTextureFragmentShader( int numTexImageUnits ) const
{
    std::stringstream buf;

    buf << "uniform sampler2D ";
    for( int i=0; i<numTexImageUnits; ++i )
        buf << "tex" << i << (i+1 < numTexImageUnits? "," : "; \n");

    buf << "vec4 osgearth_frag_texture(void) \n"
        << "{ \n"
        << "    vec3 color = vec3(1,1,1); \n"
        << "    vec4 texel; \n";

    for(int i=0; i<numTexImageUnits; ++i )
    {
        buf << "texel = texture2D(tex" << i << ", gl_TexCoord["<< i <<"].st); \n"
            << "color = mix( color, texel.rgb, texel.a ); \n";
    }
        
    buf << "    return vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    return new osg::Shader( osg::Shader::FRAGMENT, str );
}

osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    return new osg::Shader( osg::Shader::VERTEX, s_PerVertexLighting_VertexShaderSource );
}

osg::Shader*
ShaderFactory::createDefaultLightingFragmentShader() const
{
    return new osg::Shader( osg::Shader::FRAGMENT, s_PerVertexLighting_FragmentShaderSource );
}
