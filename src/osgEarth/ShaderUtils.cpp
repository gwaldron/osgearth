/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/ShaderUtils>
#include <osgEarth/ShaderFactory>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <osgEarth/GLSLChunker>

using namespace osgEarth;

//------------------------------------------------------------------------

namespace 
{
#if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
    static bool s_NO_FFP = true;
#else
    static bool s_NO_FFP = false;
#endif


    typedef std::list<const osg::StateSet*> StateSetStack;
    
    static const osg::Material*
    getFrontMaterial(const StateSetStack& statesetStack)
    {
        const osg::Material* base_material = NULL;
        osg::StateAttribute::GLModeValue base_val = osg::StateAttribute::ON;
        
        for(StateSetStack::const_iterator itr = statesetStack.begin();
            itr != statesetStack.end();
            ++itr)
        {
            
            osg::StateAttribute::GLModeValue val = (*itr)->getMode(GL_DIFFUSE);//?
            
            //if ( (val & osg::StateAttribute::INHERIT) == 0 )
            {
            //    if ((val & osg::StateAttribute::PROTECTED)!=0 ||
             //       (base_val & osg::StateAttribute::OVERRIDE)==0)
                {
                    base_val = val;
                    const osg::StateAttribute* materialAtt = (*itr)->getAttribute(osg::StateAttribute::MATERIAL);
                    if(materialAtt){
                        const osg::Material* asMaterial = dynamic_cast<const osg::Material*>(materialAtt);
                        if(val){
                            base_material = asMaterial;
                        }
                    }
                }
            }
            
        }
        return base_material;
    }
}

#undef LC
#define LC "[ShaderUtils] "

namespace
{
    // Code borrowed from osg::State.cpp
    bool replace(std::string& str, const std::string& original_phrase, const std::string& new_phrase)
    {
        bool replacedStr = false;
        std::string::size_type pos = 0;
        while((pos=str.find(original_phrase, pos))!=std::string::npos)
        {
            std::string::size_type endOfPhrasePos = pos+original_phrase.size();
            if (endOfPhrasePos<str.size())
            {
                char c = str[endOfPhrasePos];
                if ((c>='0' && c<='9') ||
                    (c>='a' && c<='z') ||
                    (c>='A' && c<='Z') ||
                    (c==']'))
                {
                    pos = endOfPhrasePos;
                    continue;
                }
            }

            replacedStr = true;
            str.replace(pos, original_phrase.size(), new_phrase);
        }
        return replacedStr;
    }

    bool replaceAndInsertDeclaration(std::string& source, std::string::size_type declPos, const std::string& originalStr, const std::string& newStr, const std::string& declarationPrefix, const std::string& declarationSuffix ="")
    {
        bool yes = replace(source, originalStr, newStr);
        if ( yes )
        {
            source.insert(declPos, declarationPrefix + newStr + declarationSuffix + std::string(";\n"));
        }
        return yes;
    }

    bool replaceAndInsertLiteral(std::string& source, std::string::size_type declPos, const std::string& originalStr, const std::string& newStr, const std::string& lit)
    {
        bool yes = replace(source, originalStr, newStr);
        if ( yes )
        {
            source.insert(declPos, lit);
        }
        return yes;
    }

    int replaceVarying(GLSLChunker::Chunks& chunks, int index, const StringVector& tokens, int offset, const std::string& prefix)
    {
        std::stringstream buf;
        buf << "#pragma vp_varying";
        if ( !prefix.empty() )
            buf << " " << prefix;

        for(int i=offset; i<tokens.size(); ++i)
        {
            if ( !tokens[i].empty() )
            {
                int len = tokens[i].length();
                if ( tokens[i][len-1] == ';' )
                    buf << " " << tokens[i].substr(0, len-1); // strip semicolon
                else
                    buf << " " << tokens[i];
            }
        }
        
        chunks[index].text = buf.str();
        chunks[index].type = GLSLChunker::Chunk::TYPE_DIRECTIVE;

        std::stringstream buf2;
        for(int i=offset; i<tokens.size(); ++i)
            buf2 << (i==offset?"":" ") << tokens[i];

        GLSLChunker::Chunk newChunk;
        newChunk.type = GLSLChunker::Chunk::TYPE_STATEMENT;
        newChunk.text = buf2.str();
        chunks.insert( chunks.begin()+index, newChunk );

        return index+1;
    }

    bool replaceVaryings(osg::Shader::Type type, GLSLChunker::Chunks& chunks)
    {
        bool madeChanges = false;

        for(int i=0; i<chunks.size(); ++i)
        {
            if ( chunks[i].type == GLSLChunker::Chunk::TYPE_STATEMENT )
            {
                std::string replacement;
                /*
                StringVector tokens;
                StringTokenizer(chunks[i].text, tokens, " \t\n", "", false, true);
                */
                const std::vector<std::string>& tokens = chunks[i].tokens;

                if      ( tokens.size() > 1 && tokens[0] == "out" && type != osg::Shader::FRAGMENT )
                    i = replaceVarying(chunks, i, tokens, 1, ""), madeChanges = true;
                else if ( tokens.size() > 1 && tokens[0] == "in" && type != osg::Shader::VERTEX )
                    i = replaceVarying(chunks, i, tokens, 1, ""), madeChanges = true;
                else if ( tokens.size() > 2 && tokens[0] == "varying" && tokens[1] == "out" && type != osg::Shader::FRAGMENT )
                    i = replaceVarying(chunks, i, tokens, 2, ""), madeChanges = true;
                else if ( tokens.size() > 2 && tokens[0] == "flat" && tokens[1] == "out" && type != osg::Shader::FRAGMENT )
                    i = replaceVarying(chunks, i, tokens, 2, "flat"), madeChanges = true;
                else if ( tokens.size() > 2 && tokens[0] == "flat" && tokens[1] == "in" && type != osg::Shader::VERTEX )
                    i = replaceVarying(chunks, i, tokens, 2, "flat"), madeChanges = true;
                else if ( tokens.size() > 1 && tokens[0] == "varying" )
                    i = replaceVarying(chunks, i, tokens, 1, ""), madeChanges = true;
            }
        }

        return madeChanges;
    }

    void applySupportForNoFFPImpl(GLSLChunker::Chunks& chunks)
    {
#if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE) && !defined(OSG_GLES3_AVAILABLE) //osg state convertVertexShaderSourceToOsgBuiltIns inserts these and the double declaration is causing an error in gles

        // for geometry and tessellation shaders, replace the built-ins with 
        // osg uniform aliases.
        const char* lines[4] = {
            "uniform mat4 osg_ModelViewMatrix;",
            "uniform mat4 osg_ProjectionMatrix;",
            "uniform mat4 osg_ModelViewProjectionMatrix;",
            "uniform mat3 osg_NormalMatrix;"
        };
    
        GLSLChunker chunker;

        for (GLSLChunker::Chunks::iterator chunk = chunks.begin(); chunk != chunks.end(); ++chunk)
        {
            if (chunk->type != GLSLChunker::Chunk::TYPE_DIRECTIVE ||
                (chunk->tokens.size()>0 && chunk->tokens[0].compare(0, 3, "#if")==0))
            {
                for (unsigned line = 0; line < 4; ++line) {
                    chunk = chunks.insert(chunk, chunker.chunkLine(lines[line]));
                    ++chunk;
                }
                break;
            }
        }

        chunker.replace(chunks, "gl_ModelViewMatrix", "osg_ModelViewMatrix");
        chunker.replace(chunks, "gl_ProjectionMatrix", "osg_ProjectionMatrix");
        chunker.replace(chunks, "gl_ModelViewProjectionMatrix", "osg_ModelViewProjectionMatrix");
        chunker.replace(chunks, "gl_NormalMatrix", "osg_NormalMatrix");
    
#endif // !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
    }
}

#if 0
void
ShaderPreProcessor::applySupportForNoFFP(osg::Shader* shader)
{
    if (!shader)
        return;
            
#if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)

    GLSLChunker chunker;
    GLSLChunker::Chunks chunks;
    chunker.read(shader->getShaderSource(), chunks);

    applySupportForNoFFPImpl(chunks);

    std::string output;
    chunker.write(chunks, output);
    shader->setShaderSource(output);

#endif // !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
}
#endif

void
ShaderPreProcessor::run(osg::Shader* shader)
{
    if ( shader )
    {
        bool dirty = false;

        std::string source = shader->getShaderSource();

        // First replace any quotes with spaces. Quotes are illegal.
        if ( source.find('\"') != std::string::npos )
        {
            osgEarth::replaceIn(source, "\"", " ");
            dirty = true;
        }

        // Chunk the shader.
        GLSLChunker chunker;
        GLSLChunker::Chunks chunks;
        chunker.read( source, chunks );


        if (shader->getType() != osg::Shader::FRAGMENT)
        {
            //applySupportForNoFFPImpl(chunks);
        }

        // Replace varyings with directives that the ShaderFactory can interpret
        // when creating interface blocks.
        replaceVaryings( shader->getType(), chunks );
        chunker.write( chunks, source );
        shader->setShaderSource( source );

        //OE_WARN << source << std::endl << std::endl;
    }
}

//------------------------------------------------------------------------

ArrayUniform::ArrayUniform( const std::string& name, osg::Uniform::Type type, osg::StateSet* stateSet, unsigned size )
{
    attach( name, type, stateSet, size );
}

void
ArrayUniform::attach( const std::string& name, osg::Uniform::Type type, osg::StateSet* stateSet, unsigned size )
{
    _uniform    = stateSet->getUniform( name );
    _uniformAlt = stateSet->getUniform( name + "[0]" );

    if ( !isValid() )
    {
        _uniform    = new osg::Uniform( type, name, size );
        _uniformAlt = new osg::Uniform( type, name + "[0]", size );
        stateSet->addUniform( _uniform.get() );
        stateSet->addUniform( _uniformAlt.get() );
    }

    _stateSet = stateSet;
}

void 
ArrayUniform::setElement( unsigned index, int value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void 
ArrayUniform::setElement( unsigned index, unsigned value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void 
ArrayUniform::setElement( unsigned index, bool value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void 
ArrayUniform::setElement( unsigned index, float value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void
ArrayUniform::setElement( unsigned index, const osg::Matrix& value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void
ArrayUniform::setElement( unsigned index, const osg::Vec3& value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

bool 
ArrayUniform::getElement( unsigned index, int& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, unsigned& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, bool& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, float& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, osg::Matrix& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, osg::Vec3& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}


void
ArrayUniform::ensureCapacity( unsigned newSize )
{
    if ( isValid() && _uniform->getNumElements() < newSize )
    {
        osg::ref_ptr<osg::StateSet> stateSet_safe = _stateSet.get();
        if ( stateSet_safe.valid() )
        {
            osg::ref_ptr<osg::Uniform> _oldUniform    = _uniform.get();
            osg::ref_ptr<osg::Uniform> _oldUniformAlt = _oldUniform.get();

            stateSet_safe->removeUniform( _uniform->getName() );
            stateSet_safe->removeUniform( _uniformAlt->getName() );

            _uniform    = new osg::Uniform( _uniform->getType(), _uniform->getName(), newSize );
            _uniformAlt = new osg::Uniform( _uniform->getType(), _uniform->getName() + "[0]", newSize );

            switch( _oldUniform->getInternalArrayType(_oldUniform->getType()) )
            {
            case GL_FLOAT:
              {
                for( unsigned i = 0; i < _oldUniform->getNumElements(); ++i )
                {
                  float value;
                  _oldUniform->getElement(i, value);
                  setElement( i, value );
                }
              }
              break;

            case GL_INT:
              {
                for( unsigned i = 0; i < _oldUniform->getNumElements(); ++i )
                {
                  int value;
                  _oldUniform->getElement(i, value);
                  setElement( i, value );
                }
              }
              break;

            case GL_UNSIGNED_INT:
              {
                for( unsigned i = 0; i < _oldUniform->getNumElements(); ++i )
                {
                  unsigned value;
                  _oldUniform->getElement(i, value);
                  setElement( i, value );
                }
              }
              break;
            }

            stateSet_safe->addUniform( _uniform.get() );
            stateSet_safe->addUniform( _uniformAlt.get() );

            stateSet_safe.release(); // don't want to unref delete
        }
    }
}

void
ArrayUniform::detach()
{
    if ( isValid() )
    {
        osg::ref_ptr<osg::StateSet> stateSet_safe = _stateSet.get();
        if ( stateSet_safe.valid() )
        {
            stateSet_safe->removeUniform( _uniform->getName() );
            stateSet_safe->removeUniform( _uniformAlt->getName() );

            _uniform = 0L;
            _uniformAlt = 0L;
            _stateSet = 0L;

            stateSet_safe.release(); // don't want to unref delete
        }
    }
}

//...................................................................

RangeUniformCullCallback::RangeUniformCullCallback() :
_dump( false )
{
    _uniform = osgEarth::Registry::instance()->shaderFactory()->createRangeUniform();

    _stateSet = new osg::StateSet();
    _stateSet->addUniform( _uniform.get() );
}

void
RangeUniformCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

    const osg::BoundingSphere& bs = node->getBound();

    float range = nv->getDistanceToViewPoint( bs.center(), true );

    // range = distance from the viewpoint to the outside of the bounding sphere.
    _uniform->set( range - bs.radius() );

    if ( _dump )
    {
        OE_NOTICE
            << "Range = " << range 
            << ", center = " << bs.center().x() << "," << bs.center().y()
            << ", radius = " << bs.radius() << std::endl;
    }
    
    cv->pushStateSet( _stateSet.get() );
    traverse(node, nv);
    cv->popStateSet();
}

//------------------------------------------------------------------------

void
DiscardAlphaFragments::install(osg::StateSet* ss, float minAlpha) const
{
    if ( ss && minAlpha < 1.0f && Registry::capabilities().supportsGLSL() )
    {
        VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
        if ( vp )
        {
            vp->setName("Discard Alpha");

            std::string code = Stringify()
                << "#version " << GLSL_VERSION_STR << "\n"
                << GLSL_DEFAULT_PRECISION_FLOAT << "\n"
                << "void oe_discardalpha_frag(inout vec4 color) { \n"
                << "    if ( color.a < " << std::setprecision(1) << minAlpha << ") discard;\n"
                << "} \n";

            vp->setFunction(
                "oe_discardalpha_frag",
                code,
                ShaderComp::LOCATION_FRAGMENT_COLORING,
                0L, 0.95f);
        }
    }
}
 
void
DiscardAlphaFragments::uninstall(osg::StateSet* ss) const
{
    if ( ss )
    {
        VirtualProgram* vp = VirtualProgram::get(ss);
        if ( vp )
        {
            vp->removeShader("oe_discardalpha_frag");
        }
    }
}
