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
#include <osgEarth/VirtualProgram>

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ShaderUtils>
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

//------------------------------------------------------------------------

#define VERTEX_MAIN             "osgearth_vert_main"
#define FRAGMENT_MAIN           "osgearth_frag_main"

// environment variable control
#define OSGEARTH_DUMP_SHADERS  "OSGEARTH_DUMP_SHADERS"
#define OSGEARTH_MERGE_SHADERS "OSGEARTH_MERGE_SHADERS"

namespace
{
#ifdef OSG_GLES2_AVAILABLE
    bool s_mergeShaders = true;
#else
    bool s_mergeShaders = false;
#endif

    bool s_dumpShaders = false;        // debugging

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

// same type as PROGRAM (for proper state sorting)
const osg::StateAttribute::Type VirtualProgram::SA_TYPE = osg::StateAttribute::PROGRAM;


VirtualProgram::VirtualProgram( unsigned mask ) : 
_mask              ( mask ),
_inherit           ( true ),
_useLightingShaders( true )
{
    // because we sometimes update/change the attribute's members from within the apply() method
    this->setDataVariance( osg::Object::DYNAMIC );

    // check the the dump env var
    if ( ::getenv(OSGEARTH_DUMP_SHADERS) != 0L )
    {
        s_dumpShaders = true;
    }

    // check the merge env var
    if ( ::getenv(OSGEARTH_MERGE_SHADERS) != 0L )
    {
        s_mergeShaders = true;
    }

    // a template object to hold program data (so we don't have to dupliate all the 
    // osg::Program methods..)
    _template = new osg::Program();
}


VirtualProgram::VirtualProgram(const VirtualProgram& rhs, const osg::CopyOp& copyop ) :
osg::StateAttribute( rhs, copyop ),
_shaderMap         ( rhs._shaderMap ),
_mask              ( rhs._mask ),
_functions         ( rhs._functions ),
_inherit           ( rhs._inherit ),
_useLightingShaders( rhs._useLightingShaders ),
_template          ( osg::clone(rhs._template.get()) )
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

    // compare the template settings.
    int templateCompare = _template->compare( *(rhs.getTemplate()) );
    if ( templateCompare != 0 ) return templateCompare;

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

    // pre-processes the shader's source to include GLES uniforms as necessary
    // (no-op on non-GLES)
    ShaderPreProcessor::run( shader );

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

    // if there's already a function by this name, remove it
    for( OrderedFunctionMap::iterator i = ofm.begin(); i != ofm.end(); )
    {
        if ( i->second.compare(functionName) == 0 )
        {
            OrderedFunctionMap::iterator j = i;
            ++j;
            ofm.erase( i );
            i = j;
        }
        else
        {
            ++i;
        }
    }
    
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
VirtualProgram::installDefaultColoringAndLightingShaders(unsigned                           numTextures,
                                                         osg::StateAttribute::OverrideValue qual )
{
    ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

    this->setShader( sf->createDefaultColoringVertexShader(numTextures), qual );
    this->setShader( sf->createDefaultLightingVertexShader(), qual );

    this->setShader( sf->createDefaultColoringFragmentShader(numTextures), qual );
    this->setShader( sf->createDefaultLightingFragmentShader(), qual );

    setUseLightingShaders( true );
}


void
VirtualProgram::installDefaultLightingShaders(osg::StateAttribute::OverrideValue qual)
{
    ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

    this->setShader( sf->createDefaultLightingVertexShader(), qual );
    this->setShader( sf->createDefaultLightingFragmentShader(), qual );

    setUseLightingShaders( true );
}


void
VirtualProgram::installDefaultColoringShaders(unsigned                           numTextures,
                                              osg::StateAttribute::OverrideValue qual )
{
    ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();

    this->setShader( sf->createDefaultColoringVertexShader(numTextures), qual );
    this->setShader( sf->createDefaultColoringFragmentShader(numTextures), qual );
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


void
VirtualProgram::setUseLightingShaders( bool value )
{
    if ( _useLightingShaders != value )
    {
        _useLightingShaders = value;
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

        for( StringVector::const_iterator line_iter = lines.begin(); line_iter != lines.end(); ++line_iter )
        {
            std::string line = trim(*line_iter);

            if ( line.size() > 0 )
            {
                StringVector tokens;
                StringTokenizer( line, tokens, " \t", "", false, true );

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
                    std::string& header = headers[line];
                    header = line;
                }

                else
                {
                    body << (*line_iter) << "\n";
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
    if ( s_mergeShaders )
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

        if ( s_dumpShaders )
        {
            OE_NOTICE << LC 
                << "\nMERGED VERTEX SHADER: \n\n" << vertBodyText << "\n\n"
                << "MERGED FRAGMENT SHADER: \n\n" << fragBodyText << "\n" << std::endl;
        }
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

    // create the MAINs. Save the old ones so we can replace them in the cache.
    osg::ref_ptr<osg::Shader> oldVertMain = _vertMain.get();
    _vertMain = sf->createVertexShaderMain( _accumulatedFunctions, _useLightingShaders );

    osg::ref_ptr<osg::Shader> oldFragMain = _fragMain.get();
    _fragMain = sf->createFragmentShaderMain( _accumulatedFunctions, _useLightingShaders );

    // rebuild the shader list now that we've changed the shader map.
    ShaderVector keyVector;
    for( ShaderMap::iterator i = accumShaderMap.begin(); i != accumShaderMap.end(); ++i )
    {
        keyVector.push_back( i->second.first.get() );
    }

    // finally, add the mains (AFTER building the key vector)
    ShaderVector buildVector( keyVector );
    buildVector.push_back( _vertMain.get() );
    buildVector.push_back( _fragMain.get() );


    // Create a new program and add all our shaders.
    if ( s_dumpShaders )
        OE_NOTICE << LC << "---------PROGRAM: " << getName() << " ---------------\n" << std::endl;

    osg::Program* program = new osg::Program();
    program->setName(getName());
    addShadersToProgram( buildVector, accumAttribBindings, program );
    //addShadersToProgram( vec, accumAttribBindings, program );
    addTemplateDataToProgram( program );


    // Since we replaced the "mains", we have to go through the cache and update all its
    // entries to point at the new mains instead of the old ones.
    if ( oldVertMain.valid() || oldFragMain.valid() )
    {
        ProgramMap newProgramCache;

        for( ProgramMap::iterator m = _programCache.begin(); m != _programCache.end(); ++m )
        {
            const ShaderVector& originalKey = m->first;

            osg::Program* newProgram = new osg::Program();
            newProgram->setName( m->second->getName() );

            ShaderVector newBuildKey( originalKey );
            newBuildKey.push_back( _vertMain.get() );
            newBuildKey.push_back( _fragMain.get() );
            addShadersToProgram( newBuildKey, m->second->getAttribBindingList(), newProgram );

            addTemplateDataToProgram( newProgram );

            newProgramCache[originalKey] = newProgram;
        }

        _programCache = newProgramCache;
    }

    // finally, put own new program in the cache.
    _programCache[ keyVector ] = program;

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
