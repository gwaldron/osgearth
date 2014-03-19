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

#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/StringUtils>

#include <osg/Drawable>
#include <osg/Geode>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/TextureRectangle>
#include <osg/Texture2DArray>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/ClipNode>
#include <osg/ValueObject>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgText/Text>

#define LC "[ShaderGenerator] "

#define SHADERGEN_PL_EXTENSION "osgearth_shadergen"

#define SHADERGEN_HINT_IGNORE "osgEarth.ShaderGenerator.ignore"

using namespace osgEarth;

//------------------------------------------------------------------------

// compatibility string for GLES:

#ifdef OSG_GLES2_AVAILABLE
#   define GLSL_PRECISION "precision mediump float;"
#   define MEDIUMP        "mediump "
#   define LOWP           "lowp "
#   define HIGHP          "highp "
#else
#   define GLSL_PRECISION ""
#   define MEDIUMP        ""
#   define LOWP           ""
#   define HIGHP          ""
#endif

// shader names
#define TEX_COORD      "oe_sg_texcoord"
#define TEX_COORD_TEXT "oe_sg_texcoord_text"
#define SAMPLER        "oe_sg_sampler"
#define SAMPLER_TEXT   "oe_sg_sampler_text"
#define ATTRIB         "oe_sg_attrib"
#define TEXENV_COLOR   "oe_sg_texenvcolor"

#define VERTEX_FUNCTION   "oe_sg_vert"
#define FRAGMENT_FUNCTION "oe_sg_frag"

// other stuff
#define INDENT "    "

//------------------------------------------------------------------------

struct OSGEarthShaderGenPseudoLoader : public osgDB::ReaderWriter
{
    OSGEarthShaderGenPseudoLoader()
    {
        this->supportsExtension( SHADERGEN_PL_EXTENSION, "ShaderGen pseudoloader" );
    }

    const char* className()
    {
        return "OSGEarth ShaderGen pseudoloader";
    }

    bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, SHADERGEN_PL_EXTENSION );
    }

    ReadResult readObject(const std::string& filename, const osgDB::Options* options) const
    {
        return readNode( filename, options );
    }

    ReadResult readNode(const std::string& filename, const osgDB::Options* options) const
    {
        if ( !acceptsExtension(osgDB::getFileExtension(filename)) )
            return ReadResult::FILE_NOT_HANDLED;

        std::string stripped = osgDB::getNameLessExtension(filename);

        OE_INFO << LC << "Loading " << stripped << " and generating shaders." << std::endl;
        
        osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(stripped, options);
        if ( node )
        {
            ShaderGenerator gen;
            gen.setProgramName(osgDB::getSimpleFileName(stripped));
            gen.run(node, Registry::stateSetCache());
        }

        return node.valid() ? ReadResult(node.release()) : ReadResult::ERROR_IN_READING_FILE;
    }
};

REGISTER_OSGPLUGIN(SHADERGEN_PL_EXTENSION, OSGEarthShaderGenPseudoLoader)

//------------------------------------------------------------------------

namespace
{
    /**
     * The OSG State extended with mode/attribute accessors.
     */
    class StateEx : public osg::State
    {
    public:
        StateEx() : State() {}
        
        osg::StateAttribute::GLModeValue getMode(osg::StateAttribute::GLMode mode,
                                                 osg::StateAttribute::GLModeValue def = osg::StateAttribute::INHERIT) const
        {
            return getMode(_modeMap, mode, def);
        }
        
        osg::StateAttribute* getAttribute(osg::StateAttribute::Type type, unsigned int member = 0) const
        {
            return getAttribute(_attributeMap, type, member);
        }
        
        osg::StateAttribute::GLModeValue getTextureMode(unsigned int unit,
                                                        osg::StateAttribute::GLMode mode,
                                                        osg::StateAttribute::GLModeValue def = osg::StateAttribute::INHERIT) const
        {
            return unit < _textureModeMapList.size() ? getMode(_textureModeMapList[unit], mode, def) : def;
        }

        unsigned getNumTextureAttributes() const 
        {
            return _textureAttributeMapList.size();
        }

        osg::StateAttribute* getTextureAttribute(unsigned int unit, osg::StateAttribute::Type type) const
        {
            return unit < _textureAttributeMapList.size() ? getAttribute(_textureAttributeMapList[unit], type, 0) : 0;
        }
        
        osg::Uniform* getUniform(const std::string& name) const
        {
            UniformMap::const_iterator it = _uniformMap.find(name);
            return it != _uniformMap.end() ? 
            const_cast<osg::Uniform *>(it->second.uniformVec.back().first) : 0;
        }
        
    protected:
        
        osg::StateAttribute::GLModeValue getMode(const ModeMap &modeMap,
                                                 osg::StateAttribute::GLMode mode, 
                                                 osg::StateAttribute::GLModeValue def = osg::StateAttribute::INHERIT) const
        {
            ModeMap::const_iterator it = modeMap.find(mode);
            return (it != modeMap.end() && it->second.valueVec.size()) ? it->second.valueVec.back() : def;
        }
        
        osg::StateAttribute* getAttribute(const AttributeMap &attributeMap,
                                          osg::StateAttribute::Type type, unsigned int member = 0) const
        {
            AttributeMap::const_iterator it = attributeMap.find(std::make_pair(type, member));
            return (it != attributeMap.end() && it->second.attributeVec.size()) ? 
            const_cast<osg::StateAttribute*>(it->second.attributeVec.back().first) : 0;
        }
    };
}

//------------------------------------------------------------------------

ShaderGenerator::ShaderGenerator()
{
    // find everything regardless of node masking
    setTraversalMode( TRAVERSE_ALL_CHILDREN );
    setNodeMaskOverride( ~0 );

    // set a default program name:
    setProgramName( "osgEarth.ShaderGenerator" );

    // make sure we support shaders:
    _active = Registry::capabilities().supportsGLSL();
    if ( _active )
    {
        _state = new StateEx();
    }
}

void
ShaderGenerator::setIgnoreHint(osg::Node* graph, bool ignore)
{
    if (graph)
    {
        graph->setUserValue( SHADERGEN_HINT_IGNORE, ignore );
    }
}

void
ShaderGenerator::setProgramName(const std::string& name)
{
    _name = name;
}

void
ShaderGenerator::addAcceptCallback(AcceptCallback* cb)
{
    _acceptCallbacks.push_back( cb );
}

bool
ShaderGenerator::accept(const osg::StateAttribute* sa) const
{
    if ( sa == 0L )
        return false;

    for(AcceptCallbackVector::const_iterator i = _acceptCallbacks.begin(); i != _acceptCallbacks.end(); ++i )
    {
        if ( !i->get()->accept(sa) )
            return false;
    }
    return true;
}

void
ShaderGenerator::run(osg::Node* graph, StateSetCache* cache)
{
    if ( graph )
    {
        // generate shaders:
        graph->accept( *this );

        // perform GL state sharing
        if ( cache )
            cache->optimize( graph );

        osg::StateSet* stateset = graph->getOrCreateStateSet();

        // install a blank VP at the top as the default.
        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( !vp )
        {
            vp = VirtualProgram::getOrCreate(stateset);
            vp->setInheritShaders( true );
            vp->setName( _name );
        }
    }
}

void 
ShaderGenerator::apply( osg::Node& node )
{
    if ( !_active )
        return;

    bool ignore;
    if ( node.getUserValue(SHADERGEN_HINT_IGNORE, ignore) && ignore )
        return;

    osg::ref_ptr<osg::StateSet> stateset = node.getStateSet();
    if ( stateset.valid() )
    {
        _state->pushStateSet( stateset.get() );
    }

    traverse(node);

    if ( stateset.valid() )
    {
        _state->popStateSet();
    }
}

void 
ShaderGenerator::apply( osg::Geode& node )
{
    if ( !_active )
        return;

    bool ignore;
    if ( node.getUserValue(SHADERGEN_HINT_IGNORE, ignore) && ignore )
        return;

    osg::ref_ptr<osg::StateSet> stateset = node.getStateSet();
    if ( stateset.valid() )
    {
        _state->pushStateSet( stateset.get() );
    }

    unsigned numDrawables = node.getNumDrawables();
    bool traverseDrawables = true;

    // This block checks whether all the geode's drawables are equivalent,
    // i.e., they are the same type (geometry or text) and none of them
    // have their own state sets. IF that's the case, we can create a 
    // single shader program for the entire geode. This is an optimization.
    if ( stateset.valid() )
    {
        unsigned d;
        unsigned numInheritingText = 0, numInheritingGeometry = 0;
        for( d = 0; d < numDrawables; ++d )
        {
            osg::Drawable* drawable = node.getDrawable(d);
            if ( drawable->getStateSet() == 0L )
            {
                if ( drawable->asGeometry() )
                    numInheritingGeometry++;
                else if ( dynamic_cast<osgText::Text*>(drawable) )
                    numInheritingText++;
            }
        }

        if (numInheritingGeometry == numDrawables )
        {
            osg::ref_ptr<osg::StateSet> replacement;
            if ( processGeometry(stateset.get(), replacement) )
            {
                node.setStateSet(replacement.get() );
                traverseDrawables = false;
            }
        }
        else if (numInheritingText == numDrawables )
        {
            osg::ref_ptr<osg::StateSet> replacement;
            if ( processText(stateset.get(), replacement) )
            {
                node.setStateSet(replacement.get() );
                traverseDrawables = false;
            }
        }
    }

    // Drawables have state sets, so let's traverse them.
    if ( traverseDrawables )
    {
        for( unsigned d = 0; d < node.getNumDrawables(); ++d )
        {
            apply( node.getDrawable(d) );
        }
    }

    if ( stateset.valid() )
    {
        _state->popStateSet();
    }
}


void 
ShaderGenerator::apply( osg::Drawable* drawable )
{
    if ( drawable )
    {
        osg::ref_ptr<osg::StateSet> ss = drawable->getStateSet();
        if ( ss.valid() )
        {
            _state->pushStateSet(ss.get());
        }

        osg::ref_ptr<osg::StateSet> replacement;
        if ( dynamic_cast<osgText::Text*>(drawable) != 0L )
        {
            if ( processText(ss.get(), replacement) )
            {
                drawable->setStateSet( replacement.get() );
            }
        }
        else
        {
            osg::Geometry* geom = drawable->asGeometry();
            if ( geom )
            {
                geom->setUseVertexBufferObjects(true);
                geom->setUseDisplayList(false);
            }

            if ( processGeometry(ss.get(), replacement) )
            {
                drawable->setStateSet(replacement.get());
            }
        }

        if ( ss.valid() )
        {
            _state->popStateSet();
        }
    }
}


void
ShaderGenerator::apply(osg::PagedLOD& node)
{
    if ( !_active )
        return;

    bool ignore;
    if ( node.getUserValue(SHADERGEN_HINT_IGNORE, ignore) && ignore )
        return;

    for( unsigned i=0; i<node.getNumFileNames(); ++i )
    {
        static Threading::Mutex s_mutex;
        s_mutex.lock();
        const std::string& filename = node.getFileName( i );
        if (!filename.empty() && 
            osgDB::getLowerCaseFileExtension(filename).compare(SHADERGEN_PL_EXTENSION) != 0 )
        {
            node.setFileName( i, Stringify() << filename << "." << SHADERGEN_PL_EXTENSION );
        }
        s_mutex.unlock();
    }

    apply( static_cast<osg::LOD&>(node) );
}


void
ShaderGenerator::apply(osg::ProxyNode& node)
{
    if ( !_active )
        return;

    bool ignore;
    if ( node.getUserValue(SHADERGEN_HINT_IGNORE, ignore) && ignore )
        return;

    if ( node.getLoadingExternalReferenceMode() != osg::ProxyNode::LOAD_IMMEDIATELY )
    {
        // rewrite the filenames to include the shadergen pseudo-loader extension so
        // that dynamically loaded children will have the same shadergen applied.
        for( unsigned i=0; i<node.getNumFileNames(); ++i )
        {
            const std::string& filename = node.getFileName( i );
            if (!filename.empty() && 
                osgDB::getLowerCaseFileExtension(filename).compare(SHADERGEN_PL_EXTENSION) != 0 )
            {
                node.setFileName( i, Stringify() << filename << "." << SHADERGEN_PL_EXTENSION );
            }
        }
    }

    apply( static_cast<osg::Group&>(node) );
}


void
ShaderGenerator::apply(osg::ClipNode& node)
{
    static const char* s_clip_source =
        "#version " GLSL_VERSION_STR "\n"
        "void oe_sg_set_clipvertex(inout vec4 vertexVIEW)\n"
        "{\n"
        "    gl_ClipVertex = vertexVIEW; \n"
        "}\n";

    if ( !_active )
        return;

    bool ignore;
    if ( node.getUserValue(SHADERGEN_HINT_IGNORE, ignore) && ignore )
        return;

    VirtualProgram* vp = VirtualProgram::getOrCreate(node.getOrCreateStateSet());
    if ( vp->referenceCount() == 1 ) vp->setName( _name );
    vp->setFunction( "oe_sg_set_clipvertex", s_clip_source, ShaderComp::LOCATION_VERTEX_VIEW );

    apply( static_cast<osg::Group&>(node) );
}


bool
ShaderGenerator::processText(const osg::StateSet* ss, osg::ref_ptr<osg::StateSet>& replacement)
{
    // do nothing if there's no GLSL support
    if ( !_active )
        return false;

    // State object with extra accessors:
    StateEx* state = static_cast<StateEx*>(_state.get());

    // check for a real osg::Program. If it exists, bail out so that OSG
    // can use the program already in the graph
    osg::StateAttribute* program = state->getAttribute(osg::StateAttribute::PROGRAM);
    if ( dynamic_cast<osg::Program*>(program) != 0L )
        return false;

    // new state set:
    replacement = ss ? osg::clone(ss, osg::CopyOp::SHALLOW_COPY) : new osg::StateSet();

    // new VP:
    VirtualProgram* vp = 0L;
    if ( VirtualProgram::get(replacement.get()) )
        vp =  osg::clone(VirtualProgram::get(replacement.get()), osg::CopyOp::DEEP_COPY_ALL);
    else
        vp = VirtualProgram::getOrCreate(replacement.get());

    if ( vp->referenceCount() == 1 )
        vp->setName( _name );

    std::string vertSrc =
        "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION "\n"
        "varying " MEDIUMP "vec4 " TEX_COORD_TEXT ";\n"
        "void " VERTEX_FUNCTION "(inout vec4 vertexVIEW)\n"
        "{ \n"
        INDENT TEX_COORD_TEXT " = gl_MultiTexCoord0;\n"
        "} \n";

    std::string fragSrc =
        "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION "\n"
        "uniform sampler2D " SAMPLER_TEXT ";\n"
        "varying " MEDIUMP "vec4 " TEX_COORD_TEXT ";\n"
        "void " FRAGMENT_FUNCTION "(inout vec4 color)\n"
        "{ \n"
        INDENT MEDIUMP "vec4 texel = texture2D(" SAMPLER_TEXT ", " TEX_COORD_TEXT ".xy);\n"
        INDENT "color.a *= texel.a; \n"
        "}\n";

    vp->setFunction( VERTEX_FUNCTION,   vertSrc, ShaderComp::LOCATION_VERTEX_VIEW );
    vp->setFunction( FRAGMENT_FUNCTION, fragSrc, ShaderComp::LOCATION_FRAGMENT_COLORING );
    replacement->getOrCreateUniform( SAMPLER_TEXT, osg::Uniform::SAMPLER_2D )->set( 0 );

    return replacement.valid();
}


bool
ShaderGenerator::processGeometry(const osg::StateSet*         original, 
                                 osg::ref_ptr<osg::StateSet>& replacement)
{
    // do nothing if there's no GLSL support
    if ( !_active )
        return false;

    // State object with extra accessors:
    StateEx* state = static_cast<StateEx*>(_state.get());

    // check for a real osg::Program in the whole state stack. If it exists, bail out
    // so that OSG can use the program already in the graph. We never override a
    // full Program.
    osg::StateAttribute* program = state->getAttribute(osg::StateAttribute::PROGRAM);
    if ( dynamic_cast<osg::Program*>(program) != 0L )
        return false;

    // copy or create a new stateset (that we may or may not use depending on
    // what we find)
    osg::ref_ptr<osg::StateSet> new_stateset = 
        original ? osg::clone(original, osg::CopyOp::SHALLOW_COPY) :
        new osg::StateSet();

    // likewise, create a VP that we might populate.
    osg::ref_ptr<VirtualProgram> vp = VirtualProgram::cloneOrCreate(original, new_stateset);

    // we'll set this to true if the new stateset goes into effect and
    // needs to be returned.
    bool need_new_stateset = false;
    
    // give the VP a name if it needs one.
    if ( vp->getName().empty() )
        vp->setName( _name );

    // Check whether the lighting state has changed and install a mode uniform.
    // TODO: fix this
    if ( original && original->getMode(GL_LIGHTING) != osg::StateAttribute::INHERIT )
    {
        need_new_stateset = true;

        osg::StateAttribute::GLModeValue value = state->getMode(GL_LIGHTING); // from the state, not the ss.
        new_stateset->addUniform( Registry::shaderFactory()->createUniformForGLMode(GL_LIGHTING, value) );
    }

    // if the stateset changes any texture attributes, we need a new virtual program:
    if (state->getNumTextureAttributes() > 0)
    {
        // start generating the shader source.
        GenBuffers buf;
        buf.stateSet = new_stateset;

        // compatibility strings make it work in GL or GLES.
        buf.vertHead << "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION;
        buf.fragHead << "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION;

        // function declarations:
        buf.vertBody << "void " VERTEX_FUNCTION "(inout vec4 vertex_view)\n{\n";
        buf.fragBody << "void " FRAGMENT_FUNCTION "(inout vec4 color)\n{\n";

        bool wroteTexelDecl = false;

        // Loop over all possible texture image units.
        int maxUnit = Registry::capabilities().getMaxGPUTextureUnits();

        for( int unit = 0; unit < maxUnit; ++unit )
        {
            if ( !wroteTexelDecl )
            {
                buf.fragBody << INDENT << MEDIUMP "vec4 texel; \n";
                wroteTexelDecl = true;
            }

            osg::Texture* tex = dynamic_cast<osg::Texture*>(
                state->getTextureAttribute(unit, osg::StateAttribute::TEXTURE));

            if (accept(tex) && !ImageUtils::isFloatingPointInternalFormat(tex->getInternalFormat()))
            {
                osg::TexGen* texgen = dynamic_cast<osg::TexGen*>(state->getTextureAttribute(unit, osg::StateAttribute::TEXGEN));
                osg::TexEnv* texenv = dynamic_cast<osg::TexEnv*>(state->getTextureAttribute(unit, osg::StateAttribute::TEXENV));

                if ( apply(tex, texgen, texenv, unit, buf) == true )
                {
                   need_new_stateset = true;
                }
            }
        }

        if ( need_new_stateset )
        {
            // close out functions:
            buf.vertBody << "}\n";
            buf.fragBody << "}\n";

            // Extract the shader source strings (win compat method)
            std::string vertBodySrc, vertSrc, fragBodySrc, fragSrc;
            vertBodySrc = buf.vertBody.str();
            buf.vertHead << vertBodySrc;
            vertSrc = buf.vertHead.str();
            fragBodySrc = buf.fragBody.str();
            buf.fragHead << fragBodySrc;
            fragSrc = buf.fragHead.str();

            // inject the shaders:
            vp->setFunction( VERTEX_FUNCTION,   vertSrc, ShaderComp::LOCATION_VERTEX_VIEW );
            vp->setFunction( FRAGMENT_FUNCTION, fragSrc, ShaderComp::LOCATION_FRAGMENT_COLORING );
        }
    }

    if ( need_new_stateset )
    {
        replacement = new_stateset.get();
    }
    return replacement.valid();
}


bool
ShaderGenerator::apply(osg::Texture* tex, 
                       osg::TexGen*  texgen,
                       osg::TexEnv*  texenv, 
                       int           unit,
                       GenBuffers&   buf)
{
   bool ok = true;

   buf.vertHead << "varying " MEDIUMP "vec4 " TEX_COORD << unit << ";\n";
   buf.fragHead << "varying " MEDIUMP "vec4 " TEX_COORD << unit << ";\n";

   apply( texgen, unit, buf );

   if ( dynamic_cast<osg::Texture1D*>(tex) )
   {
      apply(static_cast<osg::Texture1D*>(tex), unit, buf);
   }
   else if ( dynamic_cast<osg::Texture2D*>(tex) )
   {
      apply(static_cast<osg::Texture2D*>(tex), unit, buf);
   }
   else if ( dynamic_cast<osg::Texture3D*>(tex) )
   {
      apply(static_cast<osg::Texture3D*>(tex), unit, buf);
   }
   else if ( dynamic_cast<osg::TextureRectangle*>(tex) )
   {
      apply(static_cast<osg::TextureRectangle*>(tex), unit, buf);
   }
   else if ( dynamic_cast<osg::Texture2DArray*>(tex) )
   {
      apply(static_cast<osg::Texture2DArray*>(tex), unit, buf);
   }
   else
   {
      OE_WARN << LC << "Unsupported texture type: " << tex->className() << std::endl;
      ok = false;
   }

   if ( ok )
   {
      apply( texenv, unit, buf );
   }

   return ok;
}


bool
ShaderGenerator::apply(osg::TexEnv* texenv, int unit, GenBuffers& buf)
{
    // see if we have a texenv; if so get its blending mode.
    osg::TexEnv::Mode blendingMode = osg::TexEnv::MODULATE;

    if ( accept(texenv) )
    {
        blendingMode = texenv->getMode();

        if ( blendingMode == osg::TexEnv::BLEND )
        {
            std::string texEnvColorUniform = Stringify() << TEXENV_COLOR << unit;
            buf.stateSet
                ->getOrCreateUniform(texEnvColorUniform, osg::Uniform::FLOAT_VEC4)
                ->set( texenv->getColor() );
        }
    }

    // See http://www.opengl.org/sdk/docs/man/xhtml/glTexEnv.xml
    switch( blendingMode )
    {
    case osg::TexEnv::REPLACE:
        buf.fragBody
            << INDENT "color = texel; \n";
        break;
    case osg::TexEnv::MODULATE:
        buf.fragBody
            << INDENT "color = color * texel; \n";
        break;
    case osg::TexEnv::DECAL:
        buf.fragBody
            << INDENT "color.rgb = color.rgb * (1.0 - texel.a) + (texel.rgb * texel.a); \n";
        break;
    case osg::TexEnv::BLEND:
        buf.fragHead
            << "uniform " MEDIUMP "vec4 " TEXENV_COLOR << unit << "\n;";
        buf.fragBody
            << INDENT "color.rgb = color.rgb * (1.0 - texel.rgb) + (" << TEXENV_COLOR << unit << ".rgb * texel.rgb); \n"
            << INDENT "color.a   = color.a * texel.a; \n";
        break;
    case osg::TexEnv::ADD:
    default:
        buf.fragBody
            << INDENT "color.rgb = color.rgb + texel.rgb; \n"
            << INDENT "color.a   = color.a * texel.a; \n";
    }

    return true;
}

bool
ShaderGenerator::apply(osg::TexGen* texgen, int unit, GenBuffers& buf)
{
    bool genDefault = false;

    // by default, do not use texture coordinate generation:
    if ( !accept(texgen) )
    {
        genDefault = true;
    }

    else
    {
        // Hdle different TexGen modes.
        // From the GLSL Orange Book.
        switch( texgen->getMode() )
        {
        case osg::TexGen::OBJECT_LINEAR:
            buf.vertBody
                << INDENT "{\n"
                << INDENT TEX_COORD << unit << " = "
                <<      "gl_Vertex.x*gl_ObjectPlaneS[" <<unit<< "] + "
                <<      "gl_Vertex.y*gl_ObjectPlaneT[" <<unit<< "] + "
                <<      "gl_Vertex.z*gl_ObjectPlaneR[" <<unit<< "] + "
                <<      "gl_Vertex.w*gl_ObjectPlaneQ[" <<unit<< "]; \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::EYE_LINEAR:
            buf.vertBody
                << INDENT "{\n"
                << INDENT TEX_COORD << unit << " = "
                <<      "vertex_view.x*gl_EyePlaneS[" <<unit<< "] + "
                <<      "vertex_view.y*gl_EyePlaneT[" <<unit<< "] + "
                <<      "vertex_view.z*gl_EyePlaneR[" <<unit<< "] + "
                <<      "vertex_view.w*gl_EyePlaneQ[" <<unit<< "]; \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::SPHERE_MAP:
            buf.vertBody 
                << INDENT "{\n" // scope it in case there are > 1
                << INDENT "vec3 view_vec = normalize(vertex_view.xyz/vertex_view.w); \n"
                << INDENT "vec3 r = reflect(view_vec, oe_Normal);\n"
                << INDENT "r.z += 1.0; \n"
                << INDENT "float m = 2.0 * sqrt(dot(r,r)); \n"
                << INDENT TEX_COORD << unit << " = vec4(r.x/m + 0.5, r.y/m + 0.5, 0.0, 1.0); \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::REFLECTION_MAP:
            buf.vertBody
                << INDENT "{\n"
                << INDENT "vec3 view_vec = normalize(vertex_view.xyz/vertex_view.w);\n"
                << INDENT TEX_COORD << unit << " = vec4(reflect(view_vec, oe_Normal), 1.0); \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::NORMAL_MAP:
            buf.vertBody
                << INDENT "{\n"
                << INDENT TEX_COORD << unit << " = vec4(oe_Normal, 1.0); \n"
                << INDENT "}\n";
            break;

        default: // fall back on non-gen setup.
            genDefault = true;
            break;
        }
    }
    
    if ( genDefault )
    {
        // GLSL only supports built-in "gl_MultiTexCoord{0..7}"
        if ( unit <= 7 )
        {
            buf.vertBody
                << INDENT << TEX_COORD << unit << " = gl_MultiTexCoord" << unit << ";\n";
        }
        else
        {
            OE_INFO << LC
                << "Texture coordinate on unit (" << unit << ") "
                << "requires a custom vertex attribute (osg_MultiTexCoord" << unit << ")."
                << std::endl;

            buf.vertBody 
                << INDENT << TEX_COORD << unit << " = osg_MultiTexCoord" << unit << ";\n";
        }
    }

    return true;
}

bool
ShaderGenerator::apply(osg::Texture1D* tex, int unit, GenBuffers& buf)
{
    buf.fragHead << "uniform sampler1D " SAMPLER << unit << ";\n";
    buf.fragBody << INDENT "texel = texture1D(" SAMPLER << unit << ", " TEX_COORD << unit << ".x);\n";
    buf.stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_1D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::Texture2D* tex, int unit, GenBuffers& buf)
{
    buf.fragHead << "uniform sampler2D " SAMPLER << unit << ";\n";
    buf.fragBody << INDENT "texel = texture2D(" SAMPLER << unit << ", " TEX_COORD << unit << ".xy);\n";
    buf.stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_2D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::Texture3D* tex, int unit, GenBuffers& buf)
{
    buf.fragHead << "uniform sampler3D " SAMPLER << unit << ";\n";
    buf.fragBody << INDENT "texel = texture3D(" SAMPLER << unit << ", " TEX_COORD << unit << ".xyz);\n";
    buf.stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_3D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::TextureRectangle* tex, int unit, GenBuffers& buf)
{
    buf.vertHead << "#extension GL_ARB_texture_rectangle : enable\n";

    buf.fragHead << "uniform sampler2DRect " SAMPLER << unit << ";\n";
    buf.fragBody << INDENT "texel = texture2DRect(" SAMPLER << unit << ", " TEX_COORD << unit << ".xy);\n";
    buf.stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_2D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::Texture2DArray* tex, int unit, GenBuffers& buf)
{    
    buf.fragHead <<  "#extension GL_EXT_texture_array : enable \n";    

    buf.fragHead << "uniform sampler2DArray " SAMPLER << unit << ";\n";
    buf.fragBody << INDENT "texel = texture2DArray(" SAMPLER << unit << ", " TEX_COORD << unit << ".xyz);\n";
    buf.stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_2D )->set( unit );         

    return true;
}
