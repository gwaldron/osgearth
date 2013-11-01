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
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/ClipNode>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgText/Text>

#define LC "[ShaderGenerator] "

#define SHADERGEN_PL_EXTENSION "osgearth_shadergen"

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

ShaderGenerator::ShaderGenerator() :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_name( "osgEarth.ShaderGenerator" )
{
    _active = Registry::capabilities().supportsGLSL();
    if ( _active )
    {
        _state = new StateEx();
    }
}

void
ShaderGenerator::setProgramName(const std::string& name)
{
    _name = name;
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

        // install a blank VP at the top as the default.
        VirtualProgram* vp = VirtualProgram::get(graph->getStateSet());
        if ( !vp )
        {
            vp = VirtualProgram::getOrCreate( graph->getOrCreateStateSet() );
            vp->setInheritShaders( true );
            vp->setName( _name );
        }
    }
}

void 
ShaderGenerator::apply( osg::Node& node )
{
    if ( !_active ) return;

    if ( node.getStateSet() )
        _state->pushStateSet( node.getStateSet() );

    traverse(node);

    if ( node.getStateSet() )
        _state->popStateSet();
}


void 
ShaderGenerator::apply( osg::Geode& geode )
{
    if ( !_active ) return;

    if ( geode.getStateSet() )
        _state->pushStateSet( geode.getStateSet() );

    for( unsigned d = 0; d < geode.getNumDrawables(); ++d )
    {
        apply( geode.getDrawable(d) );
    }

    if ( geode.getStateSet() )
        _state->popStateSet();
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

#if 0
        // optimize state set sharing
        if ( _stateSetCache.valid() && replacement.valid() )
        {
            if ( _stateSetCache->share(replacement, replacement) )
            {
                drawable->setStateSet( replacement.get() );
            }
        }
#endif
    }
}


void
ShaderGenerator::apply(osg::PagedLOD& node)
{
    if ( !_active ) return;

    for( unsigned i=0; i<node.getNumFileNames(); ++i )
    {
        const std::string& filename = node.getFileName( i );
        if (!filename.empty() && 
            osgDB::getLowerCaseFileExtension(filename).compare(SHADERGEN_PL_EXTENSION) != 0 )
        {
            node.setFileName( i, Stringify() << filename << "." << SHADERGEN_PL_EXTENSION );
        }
    }

    apply( static_cast<osg::LOD&>(node) );
}


void
ShaderGenerator::apply(osg::ProxyNode& node)
{
    if ( !_active ) return;

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
        "void sg_set_clipvertex(inout vec4 vertexVIEW)\n"
        "{\n"
        "    gl_ClipVertex = vertexVIEW; \n"
        "}\n";

    if ( !_active ) return;

    VirtualProgram* vp = VirtualProgram::getOrCreate(node.getOrCreateStateSet());
    if ( vp->referenceCount() == 1 ) vp->setName( _name );
    vp->setFunction( "sg_set_clipvertex", s_clip_source, ShaderComp::LOCATION_VERTEX_VIEW );

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
        "void " VERTEX_FUNCTION "(inout vec4 vertex_view)\n"
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
ShaderGenerator::processGeometry( const osg::StateSet* ss, osg::ref_ptr<osg::StateSet>& replacement )
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

    // prepare to generate:
    osg::ref_ptr<osg::StateSet> new_stateset = ss ? osg::clone(ss, osg::CopyOp::SHALLOW_COPY) : new osg::StateSet();
    VirtualProgram* vp = VirtualProgram::cloneOrCreate(ss, new_stateset);
    bool need_new_stateset = false;
    
    if ( vp->referenceCount() == 1 )
        vp->setName( _name );

    // Check whether the lighting state has changed and install a mode uniform.
    // TODO: fix this
    if ( ss && ss->getMode(GL_LIGHTING) != osg::StateAttribute::INHERIT )
    {
        need_new_stateset = true;

        osg::StateAttribute::GLModeValue value = state->getMode(GL_LIGHTING); // from the state, not the ss.
        new_stateset->addUniform( Registry::shaderFactory()->createUniformForGLMode(GL_LIGHTING, value) );
    }

    // if the stateset changes any texture attributes, we need a new virtual program:
    if (state->getNumTextureAttributes() > 0)
    {
        need_new_stateset = true;

        // work off the state's accumulated texture attribute set:
        int texCount = state->getNumTextureAttributes();

        // start generating the shader source.
        std::stringstream vertHead, vertBody, fragHead, fragBody;

        // compatibility strings make it work in GL or GLES.
        vertHead << "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION;
        fragHead << "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION;

        // function declarations:
        vertBody << "void " VERTEX_FUNCTION "(inout vec4 vertex_view)\n{\n";

        fragBody << "void " FRAGMENT_FUNCTION "(inout vec4 color)\n{\n";

        for( int t = 0; t < texCount; ++t )
        {
            if (t == 0)
            {
                fragBody << INDENT << MEDIUMP "vec4 texel; \n";
            }

            osg::StateAttribute* tex = state->getTextureAttribute( t, osg::StateAttribute::TEXTURE );
            if ( tex )
            {
                // see if we have a texenv; if so get its blending mode.
                osg::TexEnv::Mode blendingMode = osg::TexEnv::MODULATE;
                osg::TexEnv* env = dynamic_cast<osg::TexEnv*>(state->getTextureAttribute(t, osg::StateAttribute::TEXENV) );
                if ( env )
                {
                    blendingMode = env->getMode();
                    if ( blendingMode == osg::TexEnv::BLEND )
                    {
                        new_stateset->getOrCreateUniform( Stringify() << TEXENV_COLOR << t, osg::Uniform::FLOAT_VEC4 )->set( env->getColor() );
                    }
                }

                osg::TexGen::Mode texGenMode = osg::TexGen::OBJECT_LINEAR;
                osg::TexGen* texGen = dynamic_cast<osg::TexGen*>(state->getTextureAttribute(t, osg::StateAttribute::TEXGEN));
                if ( texGen )
                {
                    texGenMode = texGen->getMode();
                }

                vertHead << "varying " MEDIUMP "vec4 " TEX_COORD << t << ";\n";
                fragHead << "varying " MEDIUMP "vec4 " TEX_COORD << t << ";\n";

                // handle different TexGen modes.
                switch(texGenMode)
                {
                case osg::TexGen::SPHERE_MAP:
                    vertBody 
                        //todo: consolidate.
                        << INDENT "{\n" // scope it in case there are > 1
                        << INDENT "vec3 v = normalize(vec3(vertex_view));\n"
                        << INDENT "vec3 n = normalize(gl_NormalMatrix * gl_Normal);\n"
                        << INDENT "vec3 r = reflect(v, n);\n"
                        << INDENT "float m = 2.0 * sqrt(r.x*r.x + r.y*r.y + (r.z+1.0)*(r.z+1.0));\n"
                        << INDENT TEX_COORD << t << ".s = r.x/m + 0.5;\n"
                        << INDENT TEX_COORD << t << ".t = r.y/m + 0.5;\n"
                        << INDENT "}\n";
                    break;

                default:
                    vertBody 
                        << INDENT << TEX_COORD << t << " = gl_MultiTexCoord" << t << ";\n";
                    break;
                }


                if ( dynamic_cast<osg::Texture1D*>(tex) )
                {
                    fragHead << "uniform sampler1D " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture1D(" SAMPLER << t << ", " TEX_COORD << t << ".x);\n";
                    new_stateset->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_1D )->set( t );
                }
                else if ( dynamic_cast<osg::Texture2D*>(tex) )
                {
                    fragHead << "uniform sampler2D " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture2D(" SAMPLER << t << ", " TEX_COORD << t << ".xy);\n";
                    new_stateset->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_2D )->set( t );
                }

#if 0 // works, but requires a higher version of GL?
                else if ( dynamic_cast<osg::TextureRectangle*>(tex) )
                {
                    fragHead << "uniform sampler2Drect " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture2Drect(" SAMPLER << t << ", " TEX_COORD << t << ".xy);\n";
                    new_stateset->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_2D_RECT )->set( t );
                }
#endif
                // doesn't work. why?
                else if ( dynamic_cast<osg::TextureRectangle*>(tex) )
                {
                    osg::Image* image = static_cast<osg::TextureRectangle*>(tex)->getImage();

                    vertBody 
                        << INDENT << TEX_COORD << t << ".x /= " << (image->s()-1) << ".0;\n"
                        << INDENT << TEX_COORD << t << ".y /= " << (image->t()-1) << ".0;\n";

                    fragHead << "uniform sampler2D " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture2D(" SAMPLER << t << ", " TEX_COORD << t << ".xy);\n";
                    new_stateset->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_2D )->set( t );
                }

                else if ( dynamic_cast<osg::Texture3D*>(tex) )
                {
                    fragHead << "uniform sampler3D " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture3D(" SAMPLER << t << ", " TEX_COORD << t << ".xyz);\n";
                    new_stateset->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_3D )->set( t );
                }

                // See http://www.opengl.org/sdk/docs/man/xhtml/glTexEnv.xml
                switch( blendingMode )
                {
                case osg::TexEnv::REPLACE:
                    fragBody
                        << INDENT "color = texel; \n";
                    break;
                case osg::TexEnv::MODULATE:
                    fragBody
                        << INDENT "color = color * texel; \n";
                    break;
                case osg::TexEnv::DECAL:
                    fragBody
                        << INDENT "color.rgb = color.rgb * (1.0 - texel.a) + (texel.rgb * texel.a); \n";
                    break;
                case osg::TexEnv::BLEND:
                    fragHead
                        << "uniform " MEDIUMP "vec4 " TEXENV_COLOR << t << "\n;";
                    fragBody
                        << INDENT "color.rgb = color.rgb * (1.0 - texel.rgb) + (" << TEXENV_COLOR << t << ".rgb * texel.rgb); \n"
                        << INDENT "color.a   = color.a * texel.a; \n";
                    break;
                case osg::TexEnv::ADD:
                default:
                    fragBody
                        << INDENT "color.rgb = color.rgb + texel.rgb; \n"
                        << INDENT "color.a   = color.a * texel.a; \n";
                }
            }
        }

        // close out functions:
        vertBody << "}\n";
        fragBody << "}\n";

        // Extract the shader source strings (win compat method)
        std::string vertBodySrc, vertSrc, fragBodySrc, fragSrc;
        vertBodySrc = vertBody.str();
        vertHead << vertBodySrc;
        vertSrc = vertHead.str();
        fragBodySrc = fragBody.str();
        fragHead << fragBodySrc;
        fragSrc = fragHead.str();

        // inject the shaders:
        vp->setFunction( VERTEX_FUNCTION,   vertSrc, ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( FRAGMENT_FUNCTION, fragSrc, ShaderComp::LOCATION_FRAGMENT_COLORING );
    }

    if ( need_new_stateset )
    {
        replacement = new_stateset.get();
    }
    return replacement.valid();
}
