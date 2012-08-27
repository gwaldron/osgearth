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
#include <osgEarth/ShaderComposition>

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osg/Shader>
#include <osg/Program>
#include <osg/State>
#include <osg/Notify>
#include <sstream>

#define LC "[VirtualProgram] "

using namespace osgEarth;
using namespace osgEarth::ShaderComp;

#define OE_TEST OE_NULL
//#define OE_TEST OE_NOTICE

#ifdef OSG_GLES2_AVAILABLE
    #define MERGE_SHADERS 1
#else
    //#define MERGE_SHADERS 1
#endif

//------------------------------------------------------------------------

#define VERTEX_SETUP_COLORING   "osgearth_vert_setupColoring"
#define VERTEX_SETUP_LIGHTING   "osgearth_vert_setupLighting"
#define FRAGMENT_APPLY_COLORING "osgearth_frag_applyColoring"
#define FRAGMENT_APPLY_LIGHTING "osgearth_frag_applyLighting"

#define OSGEARTH_DUMP_SHADERS "OSGEARTH_DUMP_SHADERS"

namespace
{
    bool s_dumpShaders = false;

    /** A hack for OSG 2.8.x to get access to the state attribute vector. */
    /** TODO: no longer needed in OSG 3+ ?? */
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

RenderingHints::RenderingHints() :
_numTextures( 0 )
{
    //nop
}

RenderingHints::RenderingHints(const RenderingHints& rhs) :
_numTextures( rhs._numTextures )
{
    //nop
}

bool
RenderingHints::operator == (const RenderingHints& rhs) const
{
    return _numTextures == rhs._numTextures;
}

void
RenderingHints::useNumTextures(unsigned num)
{
    _numTextures = std::max( _numTextures, num );
}

//------------------------------------------------------------------------

// same type as PROGRAM (for proper state sorting)
const osg::StateAttribute::Type VirtualProgram::SA_TYPE = osg::StateAttribute::PROGRAM;


VirtualProgram::VirtualProgram( unsigned mask ) : 
_mask   ( mask ),
_inherit( true )
{
    // because we sometimes update/change the attribute's members from within the apply() method
    this->setDataVariance( osg::Object::DYNAMIC );

    // check the the dump env var
    if ( ::getenv(OSGEARTH_DUMP_SHADERS) != 0L )
    {
        s_dumpShaders = true;
    }

    // a template object to hold program data (so we don't have to dupliate all the 
    // osg::Program methods..)
    _template = new osg::Program();
}


VirtualProgram::VirtualProgram(const VirtualProgram& rhs, const osg::CopyOp& copyop ) :
osg::StateAttribute( rhs, copyop ),
//osg::Program( rhs, copyop ),
_shaderMap  ( rhs._shaderMap ),
_mask       ( rhs._mask ),
_functions  ( rhs._functions ),
_inherit    ( rhs._inherit )
{
    //nop
}

int
VirtualProgram::compare(const osg::StateAttribute& sa) const
{
    // check the types are equal and then create the rhs variable
    // used by the COMPARE_StateAttribute_Parameter macros below.
    COMPARE_StateAttribute_Types(VirtualProgram,sa);

    // compare each parameter in turn against the rhs.
    COMPARE_StateAttribute_Parameter(_mask);
    COMPARE_StateAttribute_Parameter(_inherit);
    //COMPARE_StateAttribute_Parameter(_shaderMap);

    // compare the shader maps.
    if ( _shaderMap.size() < rhs._shaderMap.size() ) return -1;
    if ( _shaderMap.size() > rhs._shaderMap.size() ) return 1;

    ShaderMap::const_iterator lhsIter = _shaderMap.begin();
    ShaderMap::const_iterator rhsIter = rhs._shaderMap.begin();

    while( lhsIter != _shaderMap.end() )
    {
        int keyCompare = lhsIter->first.compare( rhsIter->first );
        if ( keyCompare != 0 ) return keyCompare;

        const ShaderEntry& lhsEntry = lhsIter->second;
        const ShaderEntry& rhsEntry = rhsIter->second;
        int shaderComp = lhsEntry.first->compare( *rhsEntry.first.get() );
        if ( shaderComp != 0 ) return shaderComp;

        if ( lhsEntry.second < rhsEntry.second ) return -1;
        if ( lhsEntry.second > rhsEntry.second ) return 1;

        lhsIter++;
        rhsIter++;
    }

    return 0; // passed all the above comparison macros, must be equal.
}

void
VirtualProgram::addBindAttribLocation( const std::string& name, GLuint index )
{
    _attribBindingList[name] = index;
}

void
VirtualProgram::removeBindAttribLocation( const std::string& name )
{
    _attribBindingList.erase(name);
}


osg::Shader*
VirtualProgram::getShader( const std::string& shaderID ) const
{
    ShaderMap::const_iterator i = _shaderMap.find(shaderID);
    return i != _shaderMap.end() ? i->second.first.get() : 0L;
}


osg::Shader*
VirtualProgram::setShader(const std::string&                 shaderID,
                          osg::Shader*                       shader,
                          osg::StateAttribute::OverrideValue ov)
{
    if ( !shader || shader->getType() ==  osg::Shader::UNDEFINED ) 
        return NULL;

    shader->setName( shaderID );
    _shaderMap[shaderID] = ShaderEntry(shader, ov);

    return shader;
}


osg::Shader*
VirtualProgram::setShader(osg::Shader*                       shader,
                          osg::StateAttribute::OverrideValue ov)
{
    if ( !shader || shader->getType() == osg::Shader::UNDEFINED )
        return NULL;

    if ( shader->getName().empty() )
    {
        OE_WARN << LC << "setShader called but the shader name is not set" << std::endl;
        return 0L;
    }

    _shaderMap[shader->getName()] = ShaderEntry(shader, ov);

    return shader;
}


void
VirtualProgram::setFunction(const std::string& functionName,
                            const std::string& shaderSource,
                            FunctionLocation   location,
                            float              priority)
{
    Threading::ScopedMutexLock lock( _functionsMutex );

    OrderedFunctionMap& ofm = _functions[location];
    ofm.insert( std::pair<float,std::string>( priority, functionName ) );
    osg::Shader::Type type = (int)location <= (int)LOCATION_VERTEX_POST_LIGHTING ?
        osg::Shader::VERTEX : osg::Shader::FRAGMENT;

    setShader( functionName, new osg::Shader(type, shaderSource) );
}

void
VirtualProgram::removeShader( const std::string& shaderID )
{
    _shaderMap.erase( shaderID );

    for(FunctionLocationMap::iterator i = _functions.begin(); i != _functions.end(); ++i )
    {
        OrderedFunctionMap& ofm = i->second;
        for( OrderedFunctionMap::iterator j = ofm.begin(); j != ofm.end(); ++j )
        {
            if ( j->second == shaderID )
            {
                ofm.erase( j );
                break;
            }
        }
    }
}

/**
* Adds a new shader entry to the accumulated shader map, respecting the
* override policy of both the existing entry (if there is one) and the 
* new entry.
*/
void 
VirtualProgram::addToAccumulatedMap(ShaderMap&         accumShaderMap,
                                    const std::string& shaderID,
                                    const ShaderEntry& newEntry) const
{
    const osg::StateAttribute::OverrideValue& ov = newEntry.second;

    // see if we're trying to disable a previous entry:
    if ((ov & osg::StateAttribute::ON) == 0 ) //TODO: check for higher override
    {
        // yes? remove it!
        accumShaderMap.erase( shaderID );
    }

    else
    {
        // see if there's a higher-up entry with the same ID:
        ShaderEntry& accumEntry = accumShaderMap[ shaderID ]; 

        // make sure we can add the new one:
        if ((accumEntry.first.get() == 0L ) ||                           // empty slot, fill it
            ((ov & osg::StateAttribute::PROTECTED) != 0) ||              // new entry is protected
            ((accumEntry.second & osg::StateAttribute::OVERRIDE) == 0) ) // old entry does NOT override
        {
            accumEntry = newEntry;
        }
    }
}


void
VirtualProgram::installDefaultColoringAndLightingShaders( unsigned numTextures )
{
    ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

    this->setShader( sf->createDefaultColoringVertexShader(numTextures) );
    this->setShader( sf->createDefaultLightingVertexShader() );

    this->setShader( sf->createDefaultColoringFragmentShader(numTextures) );
    this->setShader( sf->createDefaultLightingFragmentShader() );
}


void
VirtualProgram::installDefaultLightingShaders()
{
    ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

    this->setShader( sf->createDefaultLightingVertexShader() );
    this->setShader( sf->createDefaultLightingFragmentShader() );
}


void
VirtualProgram::installDefaultColoringShaders( unsigned numTextures )
{
    ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

    this->setShader( sf->createDefaultColoringVertexShader(numTextures) );
    this->setShader( sf->createDefaultColoringFragmentShader(numTextures) );
}


void
VirtualProgram::setInheritShaders( bool value )
{
    if ( _inherit != value )
    {
        _inherit = value;
        _programCache.clear();
        _accumulatedFunctions.clear();
    }
}


namespace
{
    typedef std::map<std::string, std::string> HeaderMap;

    void parseShaderForMerging( const std::string& source, unsigned& version, HeaderMap& headers, std::stringstream& body )
    {
        // break into lines:
        StringVector lines;
        StringTokenizer( source, lines, "\n", "", true, false );

        for( StringVector::const_iterator line = lines.begin(); line != lines.end(); ++line )
        {
            if ( line->size() > 0 )
            {
                StringVector tokens;
                StringTokenizer( *line, tokens, " \t", "", false, true );

                if (tokens[0] == "#version")
                {
                    // find the highest version number.
                    if ( tokens.size() > 1 )
                    {
                        unsigned newVersion = osgEarth::as<unsigned>(tokens[1], 0);
                        if ( newVersion > version )
                        {
                            version = newVersion;
                        }
                    }
                }

                else if (
                    tokens[0] == "#extension"   ||
                    tokens[0] == "precision"    ||
                    tokens[0] == "struct"       ||
                    tokens[0] == "varying"      ||
                    tokens[0] == "uniform"      ||
                    tokens[0] == "attribute")
                {
                    std::string& header = headers[*line];
                    header = *line;
                }

                else
                {
                    body << (*line) << "\n";
                }
            }
        }
    }
}


void 
VirtualProgram::addShadersToProgram(const ShaderVector&      shaders, 
                                    const AttribBindingList& attribBindings,
                                    osg::Program*            program )
{
#ifdef MERGE_SHADERS
    bool mergeShaders = true;
#else
    bool mergeShaders = false;
#endif

    if ( mergeShaders )
    {
        unsigned          vertVersion = 0;
        HeaderMap         vertHeaders;
        std::stringstream vertBody;

        unsigned          fragVersion = 0;
        HeaderMap         fragHeaders;
        std::stringstream fragBody;

        // parse the shaders, combining header lines and finding the highest version:
        for( VirtualProgram::ShaderVector::const_iterator i = shaders.begin(); i != shaders.end(); ++i )
        {
            osg::Shader* s = i->get();
            if ( s->getType() == osg::Shader::VERTEX )
            {
                parseShaderForMerging( s->getShaderSource(), vertVersion, vertHeaders, vertBody );
            }
            else if ( s->getType() == osg::Shader::FRAGMENT )
            {
                parseShaderForMerging( s->getShaderSource(), fragVersion, fragHeaders, fragBody );
            }
        }

        // write out the merged shader code:
        std::string vertBodyText;
        vertBodyText = vertBody.str();
        std::stringstream vertShaderBuf;
        if ( vertVersion > 0 )
            vertShaderBuf << "#version " << vertVersion << "\n";
        for( HeaderMap::const_iterator h = vertHeaders.begin(); h != vertHeaders.end(); ++h )
            vertShaderBuf << h->second << "\n";
        vertShaderBuf << vertBodyText << "\n";
        vertBodyText = vertShaderBuf.str();

        std::string fragBodyText;
        fragBodyText = fragBody.str();
        std::stringstream fragShaderBuf;
        if ( fragVersion > 0 )
            fragShaderBuf << "#version " << fragVersion << "\n";
        for( HeaderMap::const_iterator h = fragHeaders.begin(); h != fragHeaders.end(); ++h )
            fragShaderBuf << h->second << "\n";
        fragShaderBuf << fragBodyText << "\n";
        fragBodyText = fragShaderBuf.str();

        // add them to the program.
        program->addShader( new osg::Shader(osg::Shader::VERTEX, vertBodyText) );
        program->addShader( new osg::Shader(osg::Shader::FRAGMENT, fragBodyText) );

        OE_TEST << LC 
            << "\nMERGED VERTEX SHADER: \n\n" << vertBodyText << "\n\n"
            << "MERGED FRAGMENT SHADER: \n\n" << fragBodyText << "\n" << std::endl;
    }
    else
    {
        for( VirtualProgram::ShaderVector::const_iterator i = shaders.begin(); i != shaders.end(); ++i )
        {
            program->addShader( i->get() );
            if ( s_dumpShaders )
                OE_NOTICE << LC << "SHADER " << i->get()->getName() << ":\n" << i->get()->getShaderSource() << "\n" << std::endl;
        }
    }

    // add the attribute bindings
    for( VirtualProgram::AttribBindingList::const_iterator abl = attribBindings.begin(); abl != attribBindings.end(); ++abl )
    {
        program->addBindAttribLocation( abl->first, abl->second );
    }
}


void
VirtualProgram::addTemplateDataToProgram( osg::Program* program )
{
    const osg::Program::FragDataBindingList& fbl = _template->getFragDataBindingList();
    for( osg::Program::FragDataBindingList::const_iterator i = fbl.begin(); i != fbl.end(); ++i )
        program->addBindFragDataLocation( i->first, i->second );

    const osg::Program::UniformBlockBindingList& ubl = _template->getUniformBlockBindingList();
    for( osg::Program::UniformBlockBindingList::const_iterator i = ubl.begin(); i != ubl.end(); ++i )
        program->addBindUniformBlock( i->first, i->second );
}


osg::Program*
VirtualProgram::buildProgram(osg::State&        state, 
                             ShaderMap&         accumShaderMap,
                             AttribBindingList& accumAttribBindings)
{
    OE_TEST << LC << "Building new Program for VP " << getName() << std::endl;

    // build a new set of accumulated functions, to support the creation of main()
    refreshAccumulatedFunctions( state );

    // No matching program in the cache; make it.
    ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

    // create the MAINs
    osg::Shader* old_vert_main = getShader( "osgearth_vert_main" );
    osg::ref_ptr<osg::Shader> vert_main = sf->createVertexShaderMain( _accumulatedFunctions );
    setShader( "osgearth_vert_main", vert_main.get() );
    addToAccumulatedMap( accumShaderMap, "osgearth_vert_main", ShaderEntry(vert_main.get(), osg::StateAttribute::ON) );

    osg::Shader* old_frag_main = getShader( "osgearth_frag_main" );
    osg::ref_ptr<osg::Shader> frag_main = sf->createFragmentShaderMain( _accumulatedFunctions );
    setShader( "osgearth_frag_main", frag_main );
    addToAccumulatedMap( accumShaderMap, "osgearth_frag_main", ShaderEntry(frag_main.get(), osg::StateAttribute::ON) );

    // rebuild the shader list now that we've changed the shader map.
    ShaderVector vec;
    for( ShaderMap::iterator i = accumShaderMap.begin(); i != accumShaderMap.end(); ++i )
    {
        vec.push_back( i->second.first.get() );
    }

    // Create a new program and add all our shaders.
    if ( s_dumpShaders )
        OE_NOTICE << LC << "---------PROGRAM: " << getName() << " ---------------\n" << std::endl;

    osg::Program* program = new osg::Program();
    program->setName(getName());
    addShadersToProgram( vec, accumAttribBindings, program );
    addTemplateDataToProgram( program );


    // Since we replaced the "mains", we have to go through the cache and update all its
    // entries to point at the new mains instead of the old ones.
    if ( old_vert_main || old_frag_main )
    {
        ProgramMap newProgramCache;

        for( ProgramMap::iterator m = _programCache.begin(); m != _programCache.end(); ++m )
        {
            const ShaderVector& original = m->first;

            // build a new cache key:
            ShaderVector newKey;

            for( ShaderVector::const_iterator i = original.begin(); i != original.end(); ++i )
            {
                if ( i->get() == old_vert_main )
                    newKey.push_back( vert_main.get() );
                else if ( i->get() == old_frag_main )
                    newKey.push_back( frag_main.get() );
                else
                    newKey.push_back( i->get() );
            }

            osg::Program* newProgram = new osg::Program();
            newProgram->setName( m->second->getName() );
            addShadersToProgram( original, m->second->getAttribBindingList(), newProgram );
            addTemplateDataToProgram( newProgram );

#if 0
            osg::Program* p = m->second.get();

            for( unsigned n = 0; n < p->getNumShaders(); --n )
            {
                osg::Shader* s = p->getShader(n);
                if ( s == old_vert_main )
                {
                    p->removeShader( s );
                    p->addShader( vert_main.get() );
                    --n;
                }
                else if ( s == old_frag_main )
                {
                    p->removeShader( s );
                    p->addShader( frag_main.get() );
                    --n;
                }
            }
#endif

            newProgramCache[newKey] = newProgram;
        }

        _programCache = newProgramCache;
    }

    // finally, put own new program in the cache.
    _programCache[ vec ] = program;

    return program;
}


void
VirtualProgram::apply( osg::State& state ) const
{
    if ( _shaderMap.empty() )
    {
        // if there's no data in the VP, unload any existing program.
        const unsigned int contextID = state.getContextID();
        const osg::GL2Extensions* extensions = osg::GL2Extensions::Get(contextID,true);
        if( ! extensions->isGlslSupported() ) return;

        extensions->glUseProgram( 0 );
        state.setLastAppliedProgramObject(0);
        return;
    }

    // first, find and collect all the VirtualProgram attributes:
    ShaderMap         accumShaderMap;
    AttribBindingList accumAttribBindings;
    
    if ( _inherit )
    {
        const StateHack::AttributeVec* av = StateHack::GetAttributeVec( state, this );
        if ( av && av->size() > 0 )
        {
            // find the deepest VP that doesn't inherit:
            unsigned start = 0;
            for( start = (int)av->size()-1; start > 0; --start )
            {
                const VirtualProgram* vp = dynamic_cast<const VirtualProgram*>( (*av)[start].first );
                if ( vp && (vp->_mask & _mask) && vp->_inherit == false )
                    break;
            }
            
            // collect shaders from there to here:
            for( unsigned i=start; i<av->size(); ++i )
            {
                const VirtualProgram* vp = dynamic_cast<const VirtualProgram*>( (*av)[i].first );
                if ( vp && (vp->_mask && _mask) )
                {
                    for( ShaderMap::const_iterator i = vp->_shaderMap.begin(); i != vp->_shaderMap.end(); ++i )
                    {
                        addToAccumulatedMap( accumShaderMap, i->first, i->second );
                    }

                    const AttribBindingList& abl = vp->getAttribBindingList();
                    accumAttribBindings.insert( abl.begin(), abl.end() );
                }
            }
        }
    }
    
    // next add the local shader components to the map, respecting the override values:
    for( ShaderMap::const_iterator i = _shaderMap.begin(); i != _shaderMap.end(); ++i )
    {
        addToAccumulatedMap( accumShaderMap, i->first, i->second );
    }
    const AttribBindingList& abl = this->getAttribBindingList();
    accumAttribBindings.insert( abl.begin(), abl.end() );
    
    
    if ( accumShaderMap.size() )
    {
        // next, assemble a list of the shaders in the map so we can use it as our
        // program cache key.
        // (Note: at present, the "cache key" does not include any information on the vertex
        // attribute bindings. Technically it should, but in practice this might not be an
        // issue; it is unlikely one would have two identical shader programs with different
        // bindings.)
        ShaderVector vec;
        vec.reserve( accumShaderMap.size() );
        for( ShaderMap::iterator i = accumShaderMap.begin(); i != accumShaderMap.end(); ++i )
        {
            ShaderEntry& entry = i->second;
            vec.push_back( entry.first.get() );
        }
        
        // see if there's already a program associated with this list:
        osg::Program* program = 0L;
        
        // look up the program:
        {
            Threading::ScopedReadLock shared( _programCacheMutex );
            
            ProgramMap::const_iterator p = _programCache.find( vec );
            if ( p != _programCache.end() )
            {
                program = p->second.get();
            }
        }
        
        // if not found, lock and build it:
        if ( !program )
        {
            Threading::ScopedWriteLock exclusive( _programCacheMutex );
            
            // look again in case of contention:
            ProgramMap::const_iterator p = _programCache.find( vec );
            if ( p != _programCache.end() )
            {
                program = p->second.get();
            }
            else
            {
                VirtualProgram* nc = const_cast<VirtualProgram*>(this);
                program = nc->buildProgram( state, accumShaderMap, accumAttribBindings );
            }
        }
        
        // finally, apply the program attribute.
        program->apply( state );
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

    // mutex no longer required since this method is called safely
    //Threading::ScopedMutexLock lock( _functionsMutex );

    _accumulatedFunctions.clear();

    if ( _inherit )
    {
        const StateHack::AttributeVec* av = StateHack::GetAttributeVec( state, this );
        if ( av && av->size() > 0 )
        {
            // find the closest VP that doesn't inherit:
            unsigned start;
            for( start = (int)av->size()-1; start > 0; --start )
            {
                const VirtualProgram* vp = dynamic_cast<const VirtualProgram*>( (*av)[start].first );
                if ( vp && (vp->_mask & _mask) && vp->_inherit == false )
                    break;
            }

            // collect functions from there on down.
            for( unsigned i=start; i<av->size(); ++i )
            {
                const VirtualProgram* vp = dynamic_cast<const VirtualProgram*>( (*av)[i].first );
                if ( vp && (vp->_mask && _mask) && (vp != this) )
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

std::string
ShaderFactory::getSamplerName( unsigned unit ) const
{
    return Stringify() << "osgearth_tex" << unit;
}


osg::Shader*
ShaderFactory::createVertexShaderMain( const FunctionLocationMap& functions ) const
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
        << "void osgearth_vert_setupColoring(); \n"
        << "void osgearth_vert_setupLighting(); \n";

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
ShaderFactory::createFragmentShaderMain( const FunctionLocationMap& functions ) const
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
        << "void osgearth_frag_applyColoring( inout vec4 color ); \n"
        << "void osgearth_frag_applyLighting( inout vec4 color ); \n";

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

    buf << "    osgearth_frag_applyColoring( color ); \n";//EDITHERE

    if ( preLighting )
        for( OrderedFunctionMap::const_iterator i = preLighting->begin(); i != preLighting->end(); ++i )
            buf << "    " << i->second << "( color ); \n";
    
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
    buf << "varying vec4 osg_TexCoord[" << Registry::instance()->getCapabilities().getMaxGPUTextureCoordSets() << "];\n";

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
        //buf << "varying vec4 osg_TexCoord[" << numTexImageUnits << "];\n";
        buf << "varying vec4 osg_TexCoord[" << Registry::instance()->getCapabilities().getMaxGPUTextureCoordSets() << "];\n";
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


osg::Shader*
ShaderFactory::createDefaultLightingVertexShader() const
{
    int maxLights = Registry::instance()->getCapabilities().getMaxLights();
    
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

//    << "        gl_BackColor = gl_FrontColor; \n"
//    << "        gl_BackSecondaryColor = gl_FrontSecondaryColor; \n"
    << "    } \n"
    << "} \n";
#else
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
    //    << "        gl_BackColor = gl_FrontColor; \n"
    //    << "        gl_BackSecondaryColor = gl_FrontSecondaryColor; \n"
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

