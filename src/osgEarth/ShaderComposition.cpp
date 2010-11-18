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

#include <osgEarth/Registry>
#include <osg/Shader>
#include <osg/Program>
#include <osg/State>
#include <osg/Notify>
#include <sstream>

#define LC "[VirtualProgram] "

using namespace osgEarth;
using namespace osgEarth::ShaderComp;

//------------------------------------------------------------------------

namespace
{
    /** A hack for OSG 2.8.x to get access to the state attribute vector. */
    class StateHack : public osg::State 
    {
    public:        
        typedef std::pair<const osg::StateAttribute*,osg::StateAttribute::OverrideValue> AttributePair;
        typedef std::vector<AttributePair> AttributeVec;

        const AttributeVec* getAttributeVec( const osg::StateAttribute* attribute ) const
        {
            osg::State::AttributeMap::const_iterator i = _attributeMap.find( attribute->getTypeMemberPair() );
            return i != _attributeMap.end() ? &(i->second.attributeVec) : 0L;
        }

        static const AttributeVec* GetAttributeVec( const osg::State& state, const osg::StateAttribute* attribute ) 
        {
            const StateHack* sh = reinterpret_cast< const StateHack* >( &state );
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
    // because we sometimes update/change the attribute's members from within the apply() method
    this->setDataVariance( osg::Object::DYNAMIC );
}

VirtualProgram::VirtualProgram(const VirtualProgram& rhs, const osg::CopyOp& copyop ) :
osg::Program( rhs, copyop ),
_shaderMap( rhs._shaderMap ),
_mask( rhs._mask ),
_functions( rhs._functions )
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

    shaderNew->setName( shaderSemantic );

    if( shaderCurrent != shaderNew )
    {
       shaderCurrent = shaderNew;
    }

    //OE_NOTICE << shader->getShaderSource() << std::endl;

    return shaderCurrent.get();
}

void
VirtualProgram::setFunction(const std::string& functionName,
                            const std::string& shaderSource,
                            FunctionLocation location,
                            float priority)
{
    Threading::ScopedMutexLock lock( _functionsMutex );

    OrderedFunctionMap& ofm = _functions[location];
    ofm.insert( std::pair<float,std::string>( priority, functionName ) );
    osg::Shader::Type type = location == LOCATION_PRE_VERTEX || location == LOCATION_POST_VERTEX ?
        osg::Shader::VERTEX : osg::Shader::FRAGMENT;
    setShader( functionName, new osg::Shader( type, shaderSource ) );
}

void
VirtualProgram::removeShader( const std::string& shaderSemantic, osg::Shader::Type type )
{
    _shaderMap.erase( ShaderMap::key_type( shaderSemantic, type ) );
}

void
VirtualProgram::apply( osg::State & state ) const
{
    if( _shaderMap.empty() ) // Virtual Program works as normal Program
        return Program::apply( state );

    // first, find and collect all the VirtualProgram attributes:
    ShaderMap shaderMap;
    const StateHack::AttributeVec* av = StateHack::GetAttributeVec( state, this );
    if ( av )
    {
        for( StateHack::AttributeVec::const_iterator i = av->begin(); i != av->end(); ++i )
        {
            const osg::StateAttribute* sa = i->first;
            const VirtualProgram* vp = dynamic_cast< const VirtualProgram* >( sa );
            if( vp && ( vp->_mask & _mask ) )
            {
                for( ShaderMap::const_iterator i = vp->_shaderMap.begin(); i != vp->_shaderMap.end(); ++i )
                {
                    shaderMap[ i->first ] = i->second;
                }
            }
        }
    }

    // next add the local shader components to the map:
    for( ShaderMap::const_iterator i = _shaderMap.begin(); i != _shaderMap.end(); ++i )
        shaderMap[ i->first ] = i->second;

    if( shaderMap.size() )
    {
        // next, assemble a list of the shaders in the map so we can compare it:
        ShaderList sl;
        for( ShaderMap::iterator i = shaderMap.begin(); i != shaderMap.end(); ++i )
            sl.push_back( i->second );

        // see if there's already a program associated with this list:
        osg::Program* program = 0L;
        ProgramMap::iterator p = _programMap.find( sl );
        if ( p != _programMap.end() )
        {
            program = p->second.get();
        }
        else
        {
            ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

            // build a new set of accumulated functions, to support the creation of main()
            const_cast<VirtualProgram*>(this)->refreshAccumulatedFunctions( state );
                
            osg::Shader* vert_main = sf->createVertexShaderMain( _accumulatedFunctions );
            const_cast<VirtualProgram*>(this)->setShader( "osgearth_vert_main", vert_main );
            shaderMap[ ShaderSemantic("osgearth_vert_main", osg::Shader::VERTEX) ] = vert_main;

            osg::Shader* frag_main = sf->createFragmentShaderMain( _accumulatedFunctions );
            const_cast<VirtualProgram*>(this)->setShader( "osgearth_frag_main", frag_main );
            shaderMap[ ShaderSemantic("osgearth_frag_main", osg::Shader::FRAGMENT) ] = frag_main;
            
            // rebuild the shader list now that we've changed the shader map.
            sl.clear();
            for( ShaderMap::iterator i = shaderMap.begin(); i != shaderMap.end(); ++i )
                sl.push_back( i->second );

            // Create a new program and add all our shaders.
            program = new osg::Program();

#if !MERGE_SHADERS
            for( ShaderList::iterator i = sl.begin(); i != sl.end(); ++i )
            {
                program->addShader( i->get() );
            }
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

            if( strFragment.length() > 0 )
            {
                program->addShader( new osg::Shader( osg::Shader::FRAGMENT, strFragment ) );
            }

            if( strVertex.length() > 0  )
            {
                program->addShader( new osg::Shader( osg::Shader::VERTEX, strVertex ) );
            }

            if( strGeometry.length() > 0  )
            {
                program->addShader( new osg::Shader( osg::Shader::GEOMETRY, strGeometry ) );
            }
#endif
            // finally, cache the program so we only regenerate it when it changes.
            _programMap[ sl ] = program;
        }

        // finally, apply the program attribute.
        state.applyAttribute( program );
    }
    else
    {
        Program::apply( state );
    }
}

void
VirtualProgram::getFunctions( FunctionLocationMap& out ) const
{
    Threading::ScopedMutexLock lock( const_cast<VirtualProgram*>(this)->_functionsMutex );
    out = _functions;
}

void
VirtualProgram::refreshAccumulatedFunctions( const osg::State& state )
{
    // This method searches the state's attribute stack and accumulates all 
    // the user functions (including those in this program).

    Threading::ScopedMutexLock lock( _functionsMutex );

    _accumulatedFunctions.clear();

    const StateHack::AttributeVec* av = StateHack::GetAttributeVec( state, this );
    for( StateHack::AttributeVec::const_iterator i = av->begin(); i != av->end(); ++i )
    {
        const osg::StateAttribute* sa = i->first;
        const VirtualProgram* vp = dynamic_cast< const VirtualProgram* >( sa );
        if( vp && vp != this && ( vp->_mask & _mask ) )
        {
            FunctionLocationMap rhs;
            vp->getFunctions( rhs );

            for( FunctionLocationMap::const_iterator j = rhs.begin(); j != rhs.end(); ++j )
            {
                const OrderedFunctionMap& ofm = j->second;
                for( OrderedFunctionMap::const_iterator k = ofm.begin(); k != ofm.end(); ++k )
                {
                    _accumulatedFunctions[j->first].insert( *k );
                }
            }
        }
    }

    // add the local ones too:
    for( FunctionLocationMap::const_iterator j = _functions.begin(); j != _functions.end(); ++j )
    {
        const OrderedFunctionMap& ofm = j->second;
        for( OrderedFunctionMap::const_iterator k = ofm.begin(); k != ofm.end(); ++k )
        {
            _accumulatedFunctions[j->first].insert( *k );
        }
    } 
}

//----------------------------------------------------------------------------

osg::Shader*
ShaderFactory::createVertexShaderMain( const FunctionLocationMap& functions ) const
{
    FunctionLocationMap::const_iterator i = functions.find( LOCATION_PRE_VERTEX );
    const OrderedFunctionMap* preVert = i != functions.end() ? &i->second : 0L;

    FunctionLocationMap::const_iterator j = functions.find( LOCATION_POST_VERTEX );
    const OrderedFunctionMap* postVert = j != functions.end() ? &j->second : 0L;

    std::stringstream buf;
    buf << "void osgearth_vert_setupTexturing(); \n"
        << "void osgearth_vert_setupLighting(); \n"
        << "uniform bool osgearth_LightingEnabled; \n"
        << "varying float osgearth_CameraRange; \n";

    if ( preVert )
        for( OrderedFunctionMap::const_iterator i = preVert->begin(); i != preVert->end(); ++i )
            buf << "void " << i->second << "(); \n";

    if ( postVert )
        for( OrderedFunctionMap::const_iterator i = postVert->begin(); i != postVert->end(); ++i )
            buf << "void " << i->second << "(); \n";

    buf << "void main(void) \n"
        << "{ \n"
        << "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"

        << "    vec4 position4 = gl_ModelViewMatrix * gl_Vertex; \n"
        << "    osgearth_CameraRange = length( position4.xyz ); \n"

        << "    vec3 position = position4.xyz / position4.w; \n"
        << "    vec3 normal = normalize( gl_NormalMatrix * gl_Normal ); \n";

    if ( preVert )
        for( OrderedFunctionMap::const_iterator i = preVert->begin(); i != preVert->end(); ++i )
            buf << "    " << i->second << "(); \n";

    buf << "    osgearth_vert_setupTexturing(); \n"
        << "    if ( osgearth_LightingEnabled ) \n"
        << "        osgearth_vert_setupLighting(); \n";
    
    if ( postVert )
        for( OrderedFunctionMap::const_iterator i = postVert->begin(); i != postVert->end(); ++i )
            buf << "    " << i->second << "(); \n";

    buf << "} \n";

    std::string str = buf.str();
    return new osg::Shader( osg::Shader::VERTEX, str );
}


osg::Shader*
ShaderFactory::createFragmentShaderMain( const FunctionLocationMap& functions ) const
{
    FunctionLocationMap::const_iterator i = functions.find( LOCATION_PRE_FRAGMENT );
    const OrderedFunctionMap* preFrag = i != functions.end() ? &i->second : 0L;

    FunctionLocationMap::const_iterator j = functions.find( LOCATION_POST_FRAGMENT );
    const OrderedFunctionMap* postFrag = j != functions.end() ? &j->second : 0L;

    std::stringstream buf;
    buf << "void osgearth_frag_applyTexturing( inout vec4 color ); \n"
        << "void osgearth_frag_applyLighting( inout vec4 color ); \n";

    if ( preFrag )
        for( OrderedFunctionMap::const_iterator i = preFrag->begin(); i != preFrag->end(); ++i )
            buf << "void " << i->second << "( inout vec4 color ); \n";

    if ( postFrag )
        for( OrderedFunctionMap::const_iterator i = postFrag->begin(); i != postFrag->end(); ++i )
            buf << "void " << i->second << "( inout vec4 color ); \n";

    buf << "uniform bool osgearth_LightingEnabled; \n"
        << "void main(void) \n"
        << "{ \n"
        << "    vec4 color = vec4(1,1,1,1); \n";

    if ( preFrag )
        for( OrderedFunctionMap::const_iterator i = preFrag->begin(); i != preFrag->end(); ++i )
            buf << "    " << i->second << "( color ); \n";

    buf << "    osgearth_frag_applyTexturing( color ); \n"
        << "    if (osgearth_LightingEnabled) \n"
        << "        osgearth_frag_applyLighting( color ); \n";

    if ( postFrag )
        for( OrderedFunctionMap::const_iterator i = postFrag->begin(); i != postFrag->end(); ++i )
            buf << "    " << i->second << "( color ); \n";

    buf << "    gl_FragColor = color; \n"
        << "} \n";  

    std::string str = buf.str();
    return new osg::Shader( osg::Shader::FRAGMENT, str );
}


osg::Shader*
ShaderFactory::createDefaultTextureVertexShader( int numTexCoordSets ) const
{
    std::stringstream buf;

    buf << "void osgearth_vert_setupTexturing() \n"
        << "{ \n";

    for(int i=0; i<numTexCoordSets; ++i )
    {
        buf << "    gl_TexCoord["<< i <<"] = gl_MultiTexCoord"<< i << "; \n";
    }
        
    buf << "} \n";

    std::string str = buf.str();
    return new osg::Shader( osg::Shader::VERTEX, str );
}


osg::Shader*
ShaderFactory::createDefaultTextureFragmentShader( int numTexImageUnits ) const
{
    std::stringstream buf;

    buf << "#version 120 \n"
        << "uniform sampler2D ";
    for( int i=0; i<numTexImageUnits; ++i )
        buf << "tex" << i << (i+1 < numTexImageUnits? "," : "; \n");

    buf << "void osgearth_frag_applyTexturing( inout vec4 color ) \n"
        << "{ \n"
        << "    vec3 color3 = color.rgb; \n"
        << "    vec4 texel; \n";

    for(int i=0; i<numTexImageUnits; ++i )
    {
        buf << "    texel = texture2D(tex" << i << ", gl_TexCoord["<< i <<"].st); \n"
            << "    color3 = mix( color3, texel.rgb, texel.a ); \n";
    }
        
    buf << "    color = vec4(color3,color.a); \n"
        << "} \n";

    std::string str = buf.str();
    return new osg::Shader( osg::Shader::FRAGMENT, str );
}


osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    static char s_PerVertexLighting_VertexShaderSource[] = 
        "void osgearth_vert_setupLighting()                                         \n"
        "{                                                                          \n"
        "    vec3 normal = normalize( gl_NormalMatrix * gl_Normal );                \n"
        "    float NdotL = dot( normal, normalize(gl_LightSource[0].position.xyz) );\n"
        "    NdotL = max( 0.0, NdotL );                                             \n"
        "    float NdotHV = dot( normal, gl_LightSource[0].halfVector.xyz );        \n"
        "    NdotHV = max( 0.0, NdotHV );                                           \n"
        "                                                                           \n"
        "    gl_FrontColor = gl_FrontLightModelProduct.sceneColor +                 \n"
        "                    gl_FrontLightProduct[0].ambient +                      \n"
        "                    gl_FrontLightProduct[0].diffuse * NdotL;               \n"
        "                                                                           \n"
        "    gl_FrontSecondaryColor = vec4(0.0);                                    \n"
        "                                                                           \n"
        "    if ( NdotL * NdotHV > 0.0 )                                            \n"
        "        gl_FrontSecondaryColor = gl_FrontLightProduct[0].specular *        \n"
        "                                 pow( NdotHV, gl_FrontMaterial.shininess );\n"
        "                                                                           \n"
        "    gl_BackColor = gl_FrontColor;                                          \n"
        "    gl_BackSecondaryColor = gl_FrontSecondaryColor;                        \n"
        "}                                                                          \n";

    return new osg::Shader( osg::Shader::VERTEX, s_PerVertexLighting_VertexShaderSource );
}


osg::Shader*
ShaderFactory::createDefaultLightingFragmentShader() const
{
    static char s_PerVertexLighting_FragmentShaderSource[] =
        "void osgearth_frag_applyLighting( inout vec4 color )                            \n"
        "{                                                                          \n"
        "    color = color * gl_Color + gl_SecondaryColor;                          \n"
        "}                                                                          \n";

    return new osg::Shader( osg::Shader::FRAGMENT, s_PerVertexLighting_FragmentShaderSource );
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

//------------------------------------------------------------------------
