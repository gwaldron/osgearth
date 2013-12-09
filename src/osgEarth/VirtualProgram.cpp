/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <OpenThreads/Thread>

#define LC "[VirtualProgram] "

using namespace osgEarth;
using namespace osgEarth::ShaderComp;

#define OE_TEST OE_NULL
//#define OE_TEST OE_NOTICE

//#define USE_ATTRIB_ALIASES

//------------------------------------------------------------------------

// environment variable control
#define OSGEARTH_DUMP_SHADERS  "OSGEARTH_DUMP_SHADERS"
#define OSGEARTH_MERGE_SHADERS "OSGEARTH_MERGE_SHADERS"

namespace
{
#ifdef OSG_GLES2_AVAILABLE
    // GLES requires all shader code be merged into a since source
    bool s_mergeShaders = true;
#else
    bool s_mergeShaders = false;
#endif

    bool s_dumpShaders = false;        // debugging

    /** A device that lets us do a const search on the State's attribute map. OSG does not yet
        have a const way to do this. It has getAttributeVec() but that is non-const (it creates
        the vector if it doesn't exist); Newer versions have getAttributeMap(), but that does not
        go back to OSG 3.0. */
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
    typedef std::map<std::string, std::string> HeaderMap;

    // removes leading and trailing whitespace, and replaces all other
    // whitespace with single spaces
    std::string trimAndCompress(const std::string& in)
    {
        bool inwhite = true;
        std::stringstream buf;
        for( unsigned i=0; i<in.length(); ++i )
        {
            char c = in.at(i);
            if ( ::isspace(c) )
            {
                if ( !inwhite )
                {
                    buf << ' ';
                    inwhite = true;
                }
            }
            else
            {
                inwhite = false;
                buf << c;
            }
        }
        std::string r;
        r = buf.str();
        return trim(r);
    }


    void parseShaderForMerging( const std::string& source, unsigned& version, HeaderMap& headers, std::stringstream& body, bool& compatibility)
    {
        // break into lines:
        StringVector lines;
        StringTokenizer( source, lines, "\n", "", true, false );

        for( StringVector::const_iterator line_iter = lines.begin(); line_iter != lines.end(); ++line_iter )
        {
            std::string line = trimAndCompress(*line_iter);

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

                        // compatability profile?
                         if (tokens.size() > 2 )
                         {
                            if ( tokens[2] == "compatibility" )
                            {
                                compatibility = true;
                            }
                         }
                    }
                }

                else if (
                    tokens[0] == "#extension"   ||
                    tokens[0] == "#define"      ||
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


    bool s_attribAliasSortFunc(const std::pair<std::string,std::string>& a, const std::pair<std::string,std::string>& b) {
        return a.first.size() > b.first.size();
    }


    /**
    * Replaces a shader's attribute values with their aliases
    */
    void applyAttributeAliases(
        osg::Shader*                             shader, 
        const VirtualProgram::AttribAliasVector& sortedAliases)
    {
        std::string src = shader->getShaderSource();
        for( VirtualProgram::AttribAliasVector::const_iterator i = sortedAliases.begin(); i != sortedAliases.end(); ++i )
        {
            //OE_DEBUG << LC << "Replacing " << i->first << " with " << i->second << std::endl;
            osgEarth::replaceIn( src, i->first, i->second );
        }
        shader->setShaderSource( src );
    }


    /**
    * Adds a new shader entry to the accumulated shader map, respecting the
    * override policy of both the existing entry (if there is one) and the 
    * new entry.
    */
    void addToAccumulatedMap(VirtualProgram::ShaderMap&         accumShaderMap,
                             const std::string&                 shaderID,
                             const VirtualProgram::ShaderEntry& newEntry)
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
            VirtualProgram::ShaderEntry& accumEntry = accumShaderMap[ shaderID ]; 

            // make sure we can add the new one:
            if ((accumEntry.first.get() == 0L ) ||                           // empty slot, fill it
                ((ov & osg::StateAttribute::PROTECTED) != 0) ||              // new entry is protected
                ((accumEntry.second & osg::StateAttribute::OVERRIDE) == 0) ) // old entry does NOT override
            {
                accumEntry = newEntry;
            }
        }
    }


    /**
    * Apply the data binding information from a template program to the
    * target program.
    */
    void addTemplateDataToProgram( const osg::Program* templateProgram, osg::Program* program )
    {
        const osg::Program::FragDataBindingList& fbl = templateProgram->getFragDataBindingList();
        for( osg::Program::FragDataBindingList::const_iterator i = fbl.begin(); i != fbl.end(); ++i )
            program->addBindFragDataLocation( i->first, i->second );

        const osg::Program::UniformBlockBindingList& ubl = templateProgram->getUniformBlockBindingList();
        for( osg::Program::UniformBlockBindingList::const_iterator i = ubl.begin(); i != ubl.end(); ++i )
            program->addBindUniformBlock( i->first, i->second );
    }


    /**
    * Populates the specified Program with passed-in shaders.
    */
    void addShadersToProgram(const VirtualProgram::ShaderVector&      shaders, 
                             const VirtualProgram::AttribBindingList& attribBindings,
                             const VirtualProgram::AttribAliasMap&    attribAliases,
                             osg::Program*                            program )
    {
#ifdef USE_ATTRIB_ALIASES
        // apply any vertex attribute aliases. But first, sort them from longest to shortest 
        // so we don't get any overlap and bad replacements.
        VirtualProgram::AttribAliasVector sortedAliases;
        sortedAliases.reserve( attribAliases.size() );
        sortedAliases.insert(sortedAliases.begin(), attribAliases.begin(), attribAliases.end());
        std::sort( sortedAliases.begin(), sortedAliases.end(), s_attribAliasSortFunc );

        for( VirtualProgram::ShaderVector::const_iterator i = shaders.begin(); i != shaders.end(); ++i )
        {
            osg::Shader* shader = i->get();
            applyAttributeAliases( shader, sortedAliases );
        }
#endif

        // merge the shaders if necessary.
        if ( s_mergeShaders )
        {
            unsigned          vertVersion = 0;
            HeaderMap         vertHeaders;
            std::stringstream vertBody;
            bool              vertCompatibility = false;

            unsigned          fragVersion = 0;
            HeaderMap         fragHeaders;
            std::stringstream fragBody;
            bool              fragCompatibility = false;

            // parse the shaders, combining header lines and finding the highest version:
            for( VirtualProgram::ShaderVector::const_iterator i = shaders.begin(); i != shaders.end(); ++i )
            {
                osg::Shader* s = i->get();
                if ( s->getType() == osg::Shader::VERTEX )
                {
                    parseShaderForMerging( s->getShaderSource(), vertVersion, vertHeaders, vertBody, vertCompatibility );
                }
                else if ( s->getType() == osg::Shader::FRAGMENT )
                {
                    parseShaderForMerging( s->getShaderSource(), fragVersion, fragHeaders, fragBody, fragCompatibility );
                }
            }

            // write out the merged shader code:
            std::string vertBodyText;
            vertBodyText = vertBody.str();
            std::stringstream vertShaderBuf;
            if ( vertVersion > 0 )
                vertShaderBuf << "#version " << vertVersion << (vertCompatibility ? " compatibility" : "") << "\n";
            for( HeaderMap::const_iterator h = vertHeaders.begin(); h != vertHeaders.end(); ++h )
                vertShaderBuf << h->second << "\n";
            vertShaderBuf << vertBodyText << "\n";
            vertBodyText = vertShaderBuf.str();

            std::string fragBodyText;
            fragBodyText = fragBody.str();
            std::stringstream fragShaderBuf;
            if ( fragVersion > 0 )
                fragShaderBuf << "#version " << fragVersion << (fragCompatibility ? " compatibility" : "") << "\n";
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


    /**
    * Assemble a new OSG shader Program from the provided components.
    * Outputs the uniquely-identifying "key vector" and returns the new program.
    */
    osg::Program* buildProgram(const std::string&                  programName,
                               osg::State&                         state,
                               ShaderComp::FunctionLocationMap&    accumFunctions,
                               VirtualProgram::ShaderMap&          accumShaderMap,
                               VirtualProgram::AttribBindingList&  accumAttribBindings,
                               VirtualProgram::AttribAliasMap&     accumAttribAliases,
                               osg::Program*                       templateProgram,
                               VirtualProgram::ShaderVector&       outputKeyVector)
    {

        // create new MAINs for this function stack.
        osg::Shader* vertHooks = Registry::shaderFactory()->createVertexShaderHooks( accumFunctions );
        osg::Shader* fragHooks = Registry::shaderFactory()->createFragmentShaderHooks( accumFunctions );

        // build a new "key vector" now that we've changed the shader map.
        // we call is a key vector because it uniquely identifies this shader program
        // based on its accumlated function set.
        for( VirtualProgram::ShaderMap::iterator i = accumShaderMap.begin(); i != accumShaderMap.end(); ++i )
        {
            outputKeyVector.push_back( i->second.first.get() );
        }

        // ensure we have main functions
        FunctionLocationMap::const_iterator vertMainItr = accumFunctions.find( LOCATION_VERTEX_MAIN );
        if ( vertMainItr == accumFunctions.end() )
        {
            outputKeyVector.push_back( Registry::shaderFactory()->createVertexShaderMain() );
        }

        FunctionLocationMap::const_iterator fragMainItr = accumFunctions.find( LOCATION_FRAGMENT_MAIN );
        if ( fragMainItr == accumFunctions.end() )
        {
            outputKeyVector.push_back( Registry::shaderFactory()->createFragmentShaderMain() );
        }

        // finally, add the mains (AFTER building the key vector .. we don't want or
        // need to mains in the key vector since they are completely derived from the
        // other elements of the key vector.)
        VirtualProgram::ShaderVector buildVector( outputKeyVector );
        buildVector.push_back( vertHooks );
        buildVector.push_back( fragHooks );

        if ( s_dumpShaders )
            OE_NOTICE << LC << "---------PROGRAM: " << programName << " ---------------\n" << std::endl;

        // Create the new program.
        osg::Program* program = new osg::Program();
        program->setName( programName );
        addShadersToProgram( buildVector, accumAttribBindings, accumAttribAliases, program );
        addTemplateDataToProgram( templateProgram, program );

        return program;
    }
}

//------------------------------------------------------------------------

// same type as PROGRAM (for proper state sorting)
const osg::StateAttribute::Type VirtualProgram::SA_TYPE = osg::StateAttribute::PROGRAM;

VirtualProgram* 
VirtualProgram::getOrCreate(osg::StateSet* stateset)
{
    if ( !stateset )
        return 0L;

    VirtualProgram* vp = dynamic_cast<VirtualProgram*>( stateset->getAttribute(SA_TYPE) );
    if ( !vp )
    {
        vp = new VirtualProgram();
        vp->setInheritShaders(true);
        stateset->setAttributeAndModes( vp, osg::StateAttribute::ON );
    }
    return vp;
}

VirtualProgram* 
VirtualProgram::get(osg::StateSet* stateset)
{
    if ( !stateset )
        return 0L;

    return dynamic_cast<VirtualProgram*>( stateset->getAttribute(SA_TYPE) );
}

const VirtualProgram* 
VirtualProgram::get(const osg::StateSet* stateset)
{
    if ( !stateset )
        return 0L;

    return dynamic_cast<const VirtualProgram*>( stateset->getAttribute(SA_TYPE) );
}

VirtualProgram*
VirtualProgram::cloneOrCreate(const osg::StateSet* src, osg::StateSet* dest)
{
    if ( !dest )
        return 0L;

    const VirtualProgram* vp = 0L;

    if ( src )
    {
        vp = get( src );
    }

    if ( !vp )
    {
        return getOrCreate( dest );
    }

    else
    {
        VirtualProgram* cloneVP = osg::clone( vp, osg::CopyOp::DEEP_COPY_ALL );
        cloneVP->setInheritShaders(true);
        dest->setAttributeAndModes(cloneVP, osg::StateAttribute::ON);
        return cloneVP;
    }
}

//------------------------------------------------------------------------


VirtualProgram::VirtualProgram( unsigned mask ) : 
_mask              ( mask ),
_inherit           ( true ),
_inheritSet        ( false )
{
    // check the the dump env var
    if ( ::getenv(OSGEARTH_DUMP_SHADERS) != 0L )
    {
        s_dumpShaders = true;
        s_mergeShaders = true;
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
_inheritSet        ( rhs._inheritSet ),
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

    // compare the shader maps. Need to lock them while comparing.
    {
        Threading::ScopedReadLock shared( _dataModelMutex );

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
    }

    return 0; // passed all the above comparison macros, must be equal.
}

void
VirtualProgram::addBindAttribLocation( const std::string& name, GLuint index )
{
    Threading::ScopedWriteLock exclusive( _dataModelMutex );

#ifdef USE_ATTRIB_ALIASES
    _attribAliases[name] = Stringify() << "oe_attrib_" << index;
    _attribBindingList[_attribAliases[name]] = index;
#else
    _attribBindingList[name] = index;
#endif
}

void
VirtualProgram::removeBindAttribLocation( const std::string& name )
{
    Threading::ScopedWriteLock exclusive( _dataModelMutex );

#ifdef USE_ATTRIB_ALIASES
    std::map<std::string,std::string>::iterator i = _attribAliases.find(name);
    if ( i != _attribAliases.end() )
        _attribBindingList.erase(i->second);
#else
    _attribBindingList.erase(name);
#endif
}

void
VirtualProgram::compileGLObjects(osg::State& state) const
{
    //nop - precompilation not required
}

void
VirtualProgram::resizeGLObjectBuffers(unsigned maxSize)
{
  Threading::ScopedWriteLock exclusive( _programCacheMutex );

//  OE_WARN << LC << "Resize VP " << getName() << std::endl;

  for (ProgramMap::iterator i = _programCache.begin();
    i != _programCache.end(); ++i)
  {
    i->second->resizeGLObjectBuffers(maxSize);
  }
}

void
VirtualProgram::releaseGLObjects(osg::State* state) const
{
  Threading::ScopedWriteLock exclusive( _programCacheMutex );

//  OE_WARN << LC << "Release VP " << getName() << std::endl;

  for (ProgramMap::const_iterator i = _programCache.begin();
    i != _programCache.end(); ++i)
  {
    i->second->releaseGLObjects(state);
  }
}

osg::Shader*
VirtualProgram::getShader( const std::string& shaderID ) const
{
    Threading::ScopedReadLock readonly( _dataModelMutex );

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

    // set the inherit flag if it's not initialized
    if ( !_inheritSet )
    {
        setInheritShaders( true );
    }

    // set the name to the ID:
    shader->setName( shaderID );

    // pre-processes the shader's source to include GLES uniforms as necessary
    // (no-op on non-GLES)
    ShaderPreProcessor::run( shader );

    // lock the data model and insert the new shader.
    {
        Threading::ScopedWriteLock exclusive( _dataModelMutex );
        _shaderMap[shaderID] = ShaderEntry(shader, ov);
    }

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

    // set the inherit flag if it's not initialized
    if ( !_inheritSet )
    {
        setInheritShaders( true );
    }

    // pre-processes the shader's source to include GLES uniforms as necessary
    // (no-op on non-GLES)
    ShaderPreProcessor::run( shader );

    // lock the data model while changing it.
    {
        Threading::ScopedWriteLock exclusive( _dataModelMutex );
        _shaderMap[shader->getName()] = ShaderEntry(shader, ov);
    }

    return shader;
}


void
VirtualProgram::setFunction(const std::string& functionName,
                            const std::string& shaderSource,
                            FunctionLocation   location,
                            float              priority)
{
    // set the inherit flag if it's not initialized
    if ( !_inheritSet )
    {
        setInheritShaders( true );
    }

    // lock the functions map while iterating and then modifying it:
    {
        Threading::ScopedWriteLock exclusive( _dataModelMutex );

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

        // create and add the new shader function.
        osg::Shader::Type type = (int)location <= (int)LOCATION_VERTEX_MAIN ?
            osg::Shader::VERTEX : osg::Shader::FRAGMENT;

        osg::Shader* shader = new osg::Shader(type, shaderSource);
        shader->setName( functionName );

        // pre-processes the shader's source to include GLES uniforms as necessary
        ShaderPreProcessor::run( shader );

        _shaderMap[functionName] = ShaderEntry(shader, osg::StateAttribute::ON);

    } // release lock
}

void
VirtualProgram::removeShader( const std::string& shaderID )
{
    // lock the functions map while making changes:
    Threading::ScopedWriteLock exclusive( _dataModelMutex );

    _shaderMap.erase( shaderID );

    for(FunctionLocationMap::iterator i = _functions.begin(); i != _functions.end(); ++i )
    {
        OrderedFunctionMap& ofm = i->second;
        for( OrderedFunctionMap::iterator j = ofm.begin(); j != ofm.end(); ++j )
        {
            if ( j->second == shaderID )
            {
                ofm.erase( j );

                // if the function map for this location is now empty,
                // remove the location map altogether.
                if ( ofm.size() == 0 )
                {
                    _functions.erase( i );
                }
                return;
            }
        }
    }
}


void
VirtualProgram::setInheritShaders( bool value )
{
    if ( _inherit != value || !_inheritSet )
    {
        _inherit = value;

        // clear the program cache please
        {
            Threading::ScopedWriteLock exclusive(_programCacheMutex);
            _programCache.clear();
        }

        _inheritSet = true;
    }
}


namespace
{
}

void
VirtualProgram::apply( osg::State& state ) const
{
    if (_shaderMap.empty() && !_inheritSet)
    {
        // If there's no data in the VP, and never has been, unload any existing program.
        // NOTE: OSG's State processor creates a "global default attribute" for each type.
        // Sine we have no way of knowing whether the user created the VP or OSG created it
        // as the default fallback, we use the "_inheritSet" flag to differeniate. This
        // prevents any shader leakage from a VP-enabled node.
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
    AttribAliasMap    accumAttribAliases;
    
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
                    ShaderMap vpShaderMap;
                    vp->getShaderMap( vpShaderMap );

                    for( ShaderMap::const_iterator i = vpShaderMap.begin(); i != vpShaderMap.end(); ++i )
                    {
                        addToAccumulatedMap( accumShaderMap, i->first, i->second );
                    }

                    const AttribBindingList& abl = vp->getAttribBindingList();
                    accumAttribBindings.insert( abl.begin(), abl.end() );

#ifdef USE_ATTRIB_ALIASES
                    const AttribAliasMap& aliases = vp->getAttribAliases();
                    accumAttribAliases.insert( aliases.begin(), aliases.end() );
#endif
                }
            }
        }
    }

    // next add the local shader components to the map, respecting the override values:
    {
        Threading::ScopedReadLock readonly(_dataModelMutex);

        for( ShaderMap::const_iterator i = _shaderMap.begin(); i != _shaderMap.end(); ++i )
        {
            addToAccumulatedMap( accumShaderMap, i->first, i->second );
        }

        const AttribBindingList& abl = this->getAttribBindingList();
        accumAttribBindings.insert( abl.begin(), abl.end() );

#ifdef USE_ATTRIB_ALIASES
        const AttribAliasMap& aliases = this->getAttribAliases();
        accumAttribAliases.insert( aliases.begin(), aliases.end() );
#endif
    }


    if ( true ) //even with nothing in the map, we still want mains! -gw  //accumShaderMap.size() )
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
        osg::ref_ptr<osg::Program> program;
        
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
        if ( !program.valid() )
        {
            // build a new set of accumulated functions, to support the creation of main()
            ShaderComp::FunctionLocationMap accumFunctions;
            accumulateFunctions( state, accumFunctions );

            // now double-check the program cache, and failing that, build the
            // new shader Program.
            {
                Threading::ScopedWriteLock exclusive( _programCacheMutex );
                
                // double-check: look again ito negate race conditions
                ProgramMap::const_iterator p = _programCache.find( vec );
                if ( p != _programCache.end() )
                {
                    program = p->second.get();
                }
                else
                {
                    ShaderVector keyVector;

                    //OE_NOTICE << LC << "Building new Program for VP " << getName() << std::endl;

                    program = buildProgram(
                        getName(),
                        state,
                        accumFunctions,
                        accumShaderMap, 
                        accumAttribBindings, 
                        accumAttribAliases, 
                        _template.get(),
                        keyVector);

                    // finally, put own new program in the cache.
                    _programCache[ keyVector ] = program;
                }
            }
        }
        
        // finally, apply the program attribute.
        if ( program.valid() )
        {
            program->apply( state );

#if 0 // test code for detecting race conditions
            for(int i=0; i<10000; ++i) {
                state.setLastAppliedProgramObject(0L);
                program->apply( state );
            }
#endif
        }
    }
}

void
VirtualProgram::getFunctions( FunctionLocationMap& out ) const
{
    // make a safe copy of the functions map.
    Threading::ScopedReadLock shared( _dataModelMutex );
    out = _functions;
}

void
VirtualProgram::getShaderMap( ShaderMap& out ) const
{
    // make a safe copy of the functions map.
    Threading::ScopedReadLock shared( _dataModelMutex );
    out = _shaderMap;
}

void
VirtualProgram::accumulateFunctions(const osg::State&                state,
                                    ShaderComp::FunctionLocationMap& result) const
{
    // This method searches the state's attribute stack and accumulates all 
    // the user functions (including those in this program).
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
                        const OrderedFunctionMap& source = j->second;
                        OrderedFunctionMap&       dest   = result[j->first];

                        for( OrderedFunctionMap::const_iterator k = source.begin(); k != source.end(); ++k )
                        {
                            // remove/override an existing function with the same name
                            for( OrderedFunctionMap::iterator exists = dest.begin(); exists != dest.end(); ++exists )
                            {
                                if ( exists->second.compare( k->second ) == 0 )
                                {
                                    dest.erase(exists);
                                    break;
                                }
                            }
                            dest.insert( *k );
                        }
                    }
                }
            }
        }
    }

    // add the local ones too:
    {
        Threading::ScopedReadLock readonly( _dataModelMutex );

        for( FunctionLocationMap::const_iterator j = _functions.begin(); j != _functions.end(); ++j )
        {
            const OrderedFunctionMap& source = j->second;
            OrderedFunctionMap&       dest   = result[j->first];

            for( OrderedFunctionMap::const_iterator k = source.begin(); k != source.end(); ++k )
            {
                // remove/override an existing function with the same name
                for( OrderedFunctionMap::iterator exists = dest.begin(); exists != dest.end(); ++exists )
                {
                    if ( exists->second.compare( k->second ) == 0 )
                    {
                        dest.erase(exists);
                        break;
                    }
                }
                dest.insert( *k );
            }
        }
    }
}
