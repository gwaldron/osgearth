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

#include <osgEarth/Capabilities>
#include <osgEarth/Registry>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/StringUtils>

#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/TexEnv>
#include <osgText/Text>

#define LC "[ShaderGenerator] "

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
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
{
    _state = new StateEx();
    _stateSetCache = new StateSetCache();
    _defaultVP = new VirtualProgram();
    _defaultVP->installDefaultColoringAndLightingShaders();
}


ShaderGenerator::ShaderGenerator( StateSetCache* cache ) :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
{
    _state = new StateEx();
    _stateSetCache = cache ? cache : new StateSetCache();
    _defaultVP = new VirtualProgram();
    _defaultVP->installDefaultColoringAndLightingShaders();
}


void 
ShaderGenerator::apply( osg::Node& node )
{
    osg::ref_ptr<osg::StateSet> ss = node.getStateSet();
    if ( ss.valid() )
    {
        _state->pushStateSet( ss.get() );

        osg::ref_ptr<osg::StateSet> replacement;
        if ( processGeometry(ss.get(), replacement) )
        {
            _state->popStateSet();
            node.setStateSet( replacement.get() );
            _state->pushStateSet( replacement.get() );
        }
    }

    traverse(node);

    if ( ss.get() )
    {
        _state->popStateSet();
    }
}


void 
ShaderGenerator::apply( osg::Geode& geode )
{
    osg::ref_ptr<osg::StateSet> ss = geode.getStateSet();
    if ( ss.valid() )
    {
        _state->pushStateSet( ss.get() );

        osg::ref_ptr<osg::StateSet> replacement;
        if ( processGeometry(ss.get(), replacement) )
        {
            _state->popStateSet();
            geode.setStateSet( replacement.get() );
            _state->pushStateSet( replacement.get() );
        }
    }

    for( unsigned d = 0; d < geode.getNumDrawables(); ++d )
    {
        apply( geode.getDrawable(d) );
    }

    if ( ss.valid() )
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
                if ( processGeometry(ss.get(), replacement) )
                {
                    drawable->setStateSet(replacement.get());
                }
            }

            _state->popStateSet();
        }
    }
}


bool
ShaderGenerator::processText( osg::StateSet* ss, osg::ref_ptr<osg::StateSet>& replacement )
{
    // do nothing if there's no GLSL support
    if ( !Registry::capabilities().supportsGLSL() )
        return false;

    // State object with extra accessors:
    StateEx* state = static_cast<StateEx*>(_state.get());

    // check for a real osg::Program. If it exists, bail out so that OSG
    // can use the program already in the graph
    osg::StateAttribute* program = state->getAttribute(osg::StateAttribute::PROGRAM);
    if ( dynamic_cast<osg::Program*>(program) != 0L )
        return false;

    // New stateset that we'll merge with the existing one.
    osg::ref_ptr<osg::StateSet> newStateSet = new osg::StateSet();

    std::string vertSrc =
        "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION "\n"
        "varying " MEDIUMP "vec4 " TEX_COORD_TEXT ";\n"
        "void " VERTEX_FUNCTION "()\n"
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

    VirtualProgram* vp = dynamic_cast<VirtualProgram*>(program);
    if ( !vp )
    {
        vp = osg::clone( _defaultVP.get() );
    }

    vp->setUseLightingShaders( false );
    vp->setFunction( VERTEX_FUNCTION,   vertSrc, ShaderComp::LOCATION_VERTEX_PRE_LIGHTING );
    vp->setFunction( FRAGMENT_FUNCTION, fragSrc, ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );
    newStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );
    newStateSet->getOrCreateUniform( SAMPLER_TEXT, osg::Uniform::SAMPLER_2D )->set( 0 );

    // merge in the new state set:
    osg::ref_ptr<osg::StateSet> current = osg::clone( ss );
    current->merge( *newStateSet.get() );

    // optimize sharing:
    _stateSetCache->share(current, replacement);

    return true;
}


bool
ShaderGenerator::processGeometry( osg::StateSet* ss, osg::ref_ptr<osg::StateSet>& replacement )
{
    // do nothing if there's no GLSL support
    if ( !Registry::capabilities().supportsGLSL() )
        return false;

    // State object with extra accessors:
    StateEx* state = static_cast<StateEx*>(_state.get());

    // check for a real osg::Program. If it exists, bail out so that OSG
    // can use the program already in the graph
    osg::StateAttribute* program = state->getAttribute(osg::StateAttribute::PROGRAM);
    if ( dynamic_cast<osg::Program*>(program) != 0L )
        return false;

    // New stateset that we'll merge with the existing one.
    osg::ref_ptr<osg::StateSet> newStateSet = new osg::StateSet();

    // Install the default program for starters, if a VP is not already installed
    // on the current stateset.
    program = ss->getAttribute(osg::StateAttribute::PROGRAM);
    if ( !program )
    {
        newStateSet->setAttributeAndModes( _defaultVP.get(), osg::StateAttribute::ON );
    }

    // check whether the lighting state has changed.
    if ( ss->getMode(GL_LIGHTING) != osg::StateAttribute::INHERIT )
    {
        ShaderFactory* sf = Registry::instance()->getShaderFactory();
        osg::StateAttribute::GLModeValue value = state->getMode(GL_LIGHTING); // from the state, not the ss.
        newStateSet->addUniform( sf->getUniformForGLMode(GL_LIGHTING, value) );
    }

    // if the stateset changes any texture attributes, we need a new virtual program:
    if (ss->getTextureAttributeList().size() > 0)
    {
        // work off the state's accumulated texture attribute set:
        int texCount = state->getNumTextureAttributes();

        // check for an existing VirtualProgram; if found, we'll add to it.
        // if not, we'll clone the default and modify it.
        osg::ref_ptr<VirtualProgram> vp = dynamic_cast<VirtualProgram*>(program);
        if ( !vp.valid() )
        {
            vp = osg::clone( _defaultVP.get() );
        }

        // start generating the shader source.
        std::stringstream vertHead, vertBody, fragHead, fragBody;

        // compatibility strings make it work in GL or GLES.
        vertHead << "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION;
        fragHead << "#version " GLSL_VERSION_STR "\n" GLSL_PRECISION;

        // function declarations:
        vertBody << "void " VERTEX_FUNCTION "()\n{\n";

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
                        newStateSet->getOrCreateUniform( Stringify() << TEXENV_COLOR << t, osg::Uniform::FLOAT_VEC4 )->set( env->getColor() );
                    }
                }

                vertHead << "varying " MEDIUMP "vec4 " TEX_COORD << t << ";\n";
                vertBody << INDENT << TEX_COORD << t << " = gl_MultiTexCoord" << t << ";\n";

                fragHead << "varying " MEDIUMP "vec4 " TEX_COORD << t << ";\n";

                if ( dynamic_cast<osg::Texture1D*>(tex) )
                {
                    fragHead << "uniform sampler1D " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture1D(" SAMPLER << t << ", " TEX_COORD << t << ".x);\n";
                    newStateSet->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_1D )->set( t );
                }
                else if ( dynamic_cast<osg::Texture2D*>(tex) )
                {
                    fragHead << "uniform sampler2D " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture2D(" SAMPLER << t << ", " TEX_COORD << t << ".xy);\n";
                    newStateSet->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_2D )->set( t );
                }
                else if ( dynamic_cast<osg::Texture3D*>(tex) )
                {
                    fragHead << "uniform sampler3D " SAMPLER << t << ";\n";
                    fragBody << INDENT "texel = texture3D(" SAMPLER << t << ", " TEX_COORD << t << ".xyz);\n";
                    newStateSet->getOrCreateUniform( Stringify() << SAMPLER << t, osg::Uniform::SAMPLER_3D )->set( t );
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
        vp->setFunction( VERTEX_FUNCTION,   vertSrc, ShaderComp::LOCATION_VERTEX_PRE_LIGHTING );
        vp->setFunction( FRAGMENT_FUNCTION, fragSrc, ShaderComp::LOCATION_FRAGMENT_PRE_LIGHTING );

        newStateSet->setAttributeAndModes( vp.get(), osg::StateAttribute::ON );
    }

    // merge in the new state set:
    osg::ref_ptr<osg::StateSet> current = osg::clone( ss );
    current->merge( *newStateSet.get() );

    // optimize sharing:
    _stateSetCache->share(current, replacement);

    return true;
}
