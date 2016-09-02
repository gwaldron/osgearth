
/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

#include <osgEarth/ShaderGenerator>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderFactory>
#include <osgEarth/StringUtils>
#include <osgEarth/URI>

#include <osg/Drawable>
#include <osg/Geode>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/TextureRectangle>
#include <osg/Texture2DMultisample>
#include <osg/Texture2DArray>
#include <osg/TextureBuffer>
#include <osg/TextureCubeMap>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/TexMat>
#include <osg/ClipNode>
#include <osg/PointSprite>
#include <osg/ValueObject>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgText/Text>
#include <osgSim/LightPointNode>

#define LC "[ShaderGenerator] "

#define SHADERGEN_PL_EXTENSION "osgearth_shadergen"

#define SHADERGEN_HINT_IGNORE "osgEarth.ShaderGenerator.ignore"

// Set this to detect whether a Geode's drawables all have the same VP
// profile, and if so, promote that VP to the Geode's state set.
#define PROMOTE_EQUIVALENT_DRAWABLE_VP_TO_GEODE 1

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
#define TEX_MATRIX     "oe_sg_texmat"

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

    const char* className() const
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

        OE_INFO << LC << "Loading " << stripped << " from PLOD/Proxy and generating shaders." << std::endl;
        
        osgEarth::ReadResult result = URI(stripped).readNode(options);
        if ( result.succeeded() && result.getNode() != 0L )
        {
            osg::ref_ptr<osg::Node> node = result.releaseNode();

            osgEarth::Registry::shaderGenerator().run(
                node.get(),
                osgDB::getSimpleFileName(stripped),
                Registry::stateSetCache() );

            return ReadResult( node.release() );
        }

        else
        {
            OE_WARN << LC << "Error loading \"" << stripped << "\": " << result.errorDetail() << "\n";
            return ReadResult::ERROR_IN_READING_FILE;
        }
    }
};

REGISTER_OSGPLUGIN(SHADERGEN_PL_EXTENSION, OSGEarthShaderGenPseudoLoader)

//------------------------------------------------------------------------

namespace
{
    struct ActiveAttributeCollector : public osg::StateAttribute::ModeUsage
    {
        ActiveAttributeCollector(osg::StateSet* stateset, const osg::StateAttribute* sa, unsigned unit=0) :
            _stateset(stateset),
            _sa      (const_cast<osg::StateAttribute*>(sa)),
            _unit    (unit) {}

        virtual ~ActiveAttributeCollector() {}

        virtual void usesMode(osg::StateAttribute::GLMode mode)
        {
            if (_stateset->getMode(mode) & osg::StateAttribute::ON)
            {
                if ( _sa->isTextureAttribute() )
                    _stateset->setTextureAttribute(_unit, _sa, osg::StateAttribute::ON);
                else
                    _stateset->setAttribute(_sa, osg::StateAttribute::ON);
            }
        }

        virtual void usesTextureMode(osg::StateAttribute::GLMode mode)
        {
            if (_stateset->getTextureMode(_unit, mode) & osg::StateAttribute::ON)
            {
                if ( _sa->isTextureAttribute() )
                    _stateset->setTextureAttribute(_unit, _sa, osg::StateAttribute::ON);
                else
                    _stateset->setAttribute(_sa, osg::StateAttribute::ON);
            }
        }

        osg::StateSet*       _stateset;
        osg::StateAttribute* _sa;
        unsigned             _unit;
    };


    /**
     * The OSG State extended with mode/attribute accessors.
     */
    class StateEx : public osg::State
    {
    public:
        StateEx() : State() {}

        // Captures the ACTIVE state into a state set. i.e., only state attributes
        // set to ON.
        osg::StateSet* capture() const
        {
            osg::StateSet* stateset = new osg::StateSet();

            // add ON modes to the new stateset:
            for(ModeMap::const_iterator i=_modeMap.begin();
                i!=_modeMap.end();
                ++i)
            {
                // note GLMode = mitr->first
                const ModeStack& ms = i->second;
                if (!ms.valueVec.empty())
                {
                    stateset->setMode(i->first,ms.valueVec.back());
                }
            }

            // add ON texture modes to the new stateset:
            for(unsigned unit=0; unit<_textureModeMapList.size(); ++unit)
            {
                const ModeMap& modeMap = _textureModeMapList[unit];
                for(ModeMap::const_iterator i = modeMap.begin(); i != modeMap.end(); ++i)
                {
                    const ModeStack& ms = i->second;
                    if (!ms.valueVec.empty())
                    {
                        stateset->setTextureMode(unit, i->first, ms.valueVec.back());
                    }
                }
            }

            for(AttributeMap::const_iterator i=_attributeMap.begin();
                i!=_attributeMap.end();
                ++i)
            {
                const AttributeStack& as = i->second;
                if (!as.attributeVec.empty())
                {
                    const osg::State::AttributePair& pair = as.attributeVec.back();
                    osg::StateAttribute* sa = const_cast<osg::StateAttribute*>(pair.first);
                    ActiveAttributeCollector collector(stateset, sa);
                    bool modeless = isModeless(sa) || !sa->getModeUsage(collector);
                    if (modeless)
                    {
                        // if getModeUsage returns false, there are no modes associated with
                        // this attr, so just add it (it can't be forcably disabled)
                        stateset->setAttribute(sa, osg::StateAttribute::ON);
                    }
                }
            }

            for(unsigned unit=0; unit<_textureAttributeMapList.size(); ++unit)
            {
                const AttributeMap& attrMap = _textureAttributeMapList[unit];
                for(AttributeMap::const_iterator i = attrMap.begin(); i != attrMap.end(); ++i)
                {                    
                    const AttributeStack& as = i->second;
                    if (!as.attributeVec.empty())
                    {
                        const osg::State::AttributePair& pair = as.attributeVec.back();
                        osg::StateAttribute* sa = const_cast<osg::StateAttribute*>(pair.first);
                        ActiveAttributeCollector collector(stateset, sa, unit);
						bool modeless = isModeless(sa) || !sa->getModeUsage(collector);
						if (modeless)
						{
                            // if getModeUsage returns false, there are no modes associated with
                            // this attr, so just add it (it can't be forcably disabled)
                            stateset->setTextureAttribute(unit, sa, osg::StateAttribute::ON);
                        }
                    }
                }
            }

            return stateset;
        }

        // some attrs dont' properly report mode usage until OSG 3.3.1.
        // ref: https://github.com/openscenegraph/osg/commit/22af59482ac4f727eeed5b97476a3a47d7fe8a69
        bool isModeless(osg::StateAttribute* sa) const
        {
#if OSG_VERSION_LESS_THAN(3,3,1)            
            return
                dynamic_cast<osg::Texture2DArray*>(sa) ||
                dynamic_cast<osg::Texture2DMultisample*>(sa) ||
				dynamic_cast<osg::TextureBuffer*>(sa);
#else
            return false;
#endif
        }
    };

    // if the node has a stateset, clone it and replace it with the clone.
    // otherwise, just create a new stateset on the node.
    osg::StateSet* cloneOrCreateStateSet(osg::Node* node)
    {
        if ( node->getStateSet() )
        {
            node->setStateSet( osg::clone(node->getStateSet(), osg::CopyOp::SHALLOW_COPY) );
            return node->getStateSet();
        }
        else
        {
            return node->getOrCreateStateSet();
        }
    }
}

//...........................................................................

ShaderGenerator::GenBuffers::GenBuffers() :
_version( GLSL_VERSION ),
_stateSet(0L)
{
    //nop
}

bool
ShaderGenerator::GenBuffers::requireVersion(unsigned glslVersion, unsigned glesVersion)
{
    unsigned versionToCheck = Registry::capabilities().isGLES() ? glesVersion : glslVersion;

    if ( !Registry::capabilities().supportsGLSL(versionToCheck) )
        return false;

    if ( versionToCheck > _version )
        _version = versionToCheck;

    return true;
}


//...........................................................................

ShaderGenerator::ShaderGenerator()
{
    // find everything regardless of node masking
    setTraversalMode( TRAVERSE_ALL_CHILDREN );
    setNodeMaskOverride( ~0 );
    _state = new StateEx();
    _active = true;
    _duplicateSharedSubgraphs = false;
}

// pre-3.3.0, NodeVisitor didn't have a copy constructor.
#if OSG_VERSION_LESS_THAN(3,3,0)
ShaderGenerator::ShaderGenerator(const ShaderGenerator& rhs, const osg::CopyOp& copy) :
osg::NodeVisitor         (),
_active                  (rhs._active),
_duplicateSharedSubgraphs(rhs._duplicateSharedSubgraphs)
{
    _visitorType              = rhs._visitorType;
    _traversalMode            = rhs._traversalMode;
    _traversalMask            = rhs._traversalMask;
    _nodeMaskOverride         = rhs._nodeMaskOverride;
    _state = new StateEx();
}
#else
ShaderGenerator::ShaderGenerator(const ShaderGenerator& rhs, const osg::CopyOp& copy) :
osg::NodeVisitor         (rhs, copy),
_active                  (rhs._active),
_duplicateSharedSubgraphs(rhs._duplicateSharedSubgraphs)
{
    _state = new StateEx();
}
#endif

void
ShaderGenerator::setIgnoreHint(osg::Object* object, bool ignore)
{
    if (object)
    {
        object->setUserValue( SHADERGEN_HINT_IGNORE, ignore );
    }
}

bool
ShaderGenerator::ignore(const osg::Object* object)
{
    bool value;
    return object && object->getUserValue(SHADERGEN_HINT_IGNORE, value) && value;
}

void
ShaderGenerator::setDuplicateSharedSubgraphs(bool value)
{
    _duplicateSharedSubgraphs = value;
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

    if ( ignore(sa) )
        return false;

    for(AcceptCallbackVector::const_iterator i = _acceptCallbacks.begin(); i != _acceptCallbacks.end(); ++i )
    {
        if ( !i->get()->accept(sa) )
            return false;
    }
    return true;
}

void
ShaderGenerator::run(osg::Node*         graph,
                     const std::string& vpName, 
                     StateSetCache*     cache)
{
    if ( graph )
    {
        // generate shaders:
        graph->accept( *this );

        // perform GL state sharing
        optimizeStateSharing( graph, cache );

        osg::StateSet* stateset = cloneOrCreateStateSet(graph);

        // install a blank VP at the top as the default.
        VirtualProgram* vp = VirtualProgram::get(stateset);
        if ( !vp )
        {
            vp = VirtualProgram::getOrCreate(stateset);
            vp->setInheritShaders( true );
            vp->setName( vpName );
        }
    }
}

void
ShaderGenerator::optimizeStateSharing(osg::Node* node, StateSetCache* cache)
{
    if ( node && cache )
        cache->optimize(node);
}

void
ShaderGenerator::duplicateSharedNode(osg::Node& node)
{
    if ( node.getNumParents() > 1 )
    {
        for(int i=1; i<(int)node.getNumParents(); ++i)
        {
            osg::Group* parent = node.getParent(i);
            osg::Node* replicant = osg::clone(
                &node, 
                osg::CopyOp::DEEP_COPY_NODES | osg::CopyOp::DEEP_COPY_DRAWABLES | osg::CopyOp::DEEP_COPY_ARRAYS);
            parent->replaceChild(&node, replicant);
        }
    }
}

void 
ShaderGenerator::apply(osg::Node& node)
{
    if ( !_active )
        return;

    if ( ignore(&node) )
        return;

    if ( _duplicateSharedSubgraphs )
        duplicateSharedNode(node);

    applyNonCoreNodeIfNecessary( node );

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
ShaderGenerator::apply( osg::Group& group )
{
    apply( static_cast<osg::Node&>(group) );
}

void 
ShaderGenerator::apply( osg::Geode& node )
{
    if ( !_active )
        return;

    if ( ignore(&node) )
        return;
    
    if ( _duplicateSharedSubgraphs )
        duplicateSharedNode(node);

    osg::ref_ptr<osg::StateSet> stateset = node.getStateSet();
    if ( stateset.valid() )
    {
        _state->pushStateSet( stateset.get() );
    }

    unsigned numDrawables = node.getNumDrawables();
    bool traverseDrawables = true;

#ifdef PROMOTE_EQUIVALENT_DRAWABLE_VP_TO_GEODE
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
#endif

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

#if OSG_VERSION_GREATER_OR_EQUAL(3,3,3)
void
ShaderGenerator::apply( osg::Drawable& drawable )
{
    if ( !_active )
        return;

    if ( ignore(&drawable) )
        return;

    if ( _duplicateSharedSubgraphs )
        duplicateSharedNode(drawable);

    apply( &drawable );
}
#endif

void 
ShaderGenerator::apply( osg::Drawable* drawable )
{
    if ( drawable )
    {
        if (_drawablesVisited.find(drawable) != _drawablesVisited.end())
            return;
        else
            _drawablesVisited.insert(drawable);

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
    
    if ( ignore(&node) )
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

    if ( ignore(&node) )
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

    if ( ignore(&node) )
        return;

    osg::StateSet* stateSet = cloneOrCreateStateSet(&node);
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
    if ( vp->referenceCount() == 1 ) vp->setName( _name );
    vp->setFunction( "oe_sg_set_clipvertex", s_clip_source, ShaderComp::LOCATION_VERTEX_VIEW, 0.95f );

    apply( static_cast<osg::Group&>(node) );
}

void
ShaderGenerator::applyNonCoreNodeIfNecessary(osg::Node& node)
{
    if ( dynamic_cast<osgSim::LightPointNode*>(&node) )
    {
        apply( static_cast<osgSim::LightPointNode&>(node) );
    }
}

void
ShaderGenerator::apply(osgSim::LightPointNode& node)
{
    if ( node.getPointSprite() )
    {
        osg::ref_ptr<osg::StateSet> stateset;
        
        // if the node has state, clone it so we can add our temp attribute.
        stateset = node.getStateSet() ?
            osg::clone(node.getStateSet(), osg::CopyOp::SHALLOW_COPY) :
            new osg::StateSet();

        // add a temporary point sprite so the generator will make sprite code.
        osg::ref_ptr<osg::PointSprite> sprite = new osg::PointSprite();
        stateset->setTextureAttributeAndModes(0, sprite.get());

        _state->pushStateSet( stateset.get() );

        osg::ref_ptr<osg::StateSet> replacement;
        if ( processGeometry(stateset.get(), replacement) )
        {
            // remove the temporary sprite.
            replacement->removeTextureAttribute(0, sprite.get());
            node.setStateSet(replacement.get() );
        }

        _state->popStateSet();
    }
}

bool
ShaderGenerator::processText(const osg::StateSet* ss, osg::ref_ptr<osg::StateSet>& replacement)
{
    // do nothing if there's no GLSL support
    if ( !_active )
        return false;

    // Capture the active current state:
    osg::ref_ptr<osg::StateSet> current = static_cast<StateEx*>(_state.get())->capture();

    // check for a real osg::Program. If it exists, bail out so that OSG
    // can use the program already in the graph
    osg::StateAttribute* program = current->getAttribute(osg::StateAttribute::PROGRAM);
    if ( dynamic_cast<osg::Program*>(program) != 0L )
        return false;

    // New state set. We never modify existing statesets.
    replacement = ss ? osg::clone(ss, osg::CopyOp::SHALLOW_COPY) : new osg::StateSet();

    // new VP:
    osg::ref_ptr<VirtualProgram> vp = VirtualProgram::cloneOrCreate(replacement.get());
    
    // give the VP a name if it needs one.
    if ( vp->getName().empty() )
    {
        vp->setName( _name );
    }

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
        //INDENT MEDIUMP "vec4 texel = texture2DLod(" SAMPLER_TEXT ", " TEX_COORD_TEXT ".xy, 0.0);\n"
        INDENT "color.a *= texel.a; \n"
        "}\n";

    vp->setFunction( VERTEX_FUNCTION,   vertSrc, ShaderComp::LOCATION_VERTEX_MODEL, 0.5f );
    vp->setFunction( FRAGMENT_FUNCTION, fragSrc, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.5f );
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
    
    // capture the active current state:
    osg::ref_ptr<osg::StateSet> current = static_cast<StateEx*>(_state.get())->capture();

    // check for a real osg::Program in the whole state stack. If it exists, bail out
    // so that OSG can use the program already in the graph. We never override a
    // full Program.
    osg::StateAttribute* program = current->getAttribute(osg::StateAttribute::PROGRAM);
    if ( dynamic_cast<osg::Program*>(program) != 0L )
        return false;

    // Copy or create a new stateset (that we may or may not use depending on
    // what we find). Never modify an existing stateset!
    osg::ref_ptr<osg::StateSet> newStateSet =
        original ? osg::clone(original, osg::CopyOp::SHALLOW_COPY) :
        new osg::StateSet();

    // likewise, create a VP that we might populate.
    osg::ref_ptr<VirtualProgram> vp = VirtualProgram::cloneOrCreate(original, newStateSet);

    // we'll set this to true if the new stateset goes into effect and
    // needs to be returned.
    bool needNewStateSet = false;
    bool needVertexFunction = false;
    bool needFragmentFunction = false;
    
    // give the VP a name if it needs one.
    if ( vp->getName().empty() )
    {
        vp->setName( _name );
    }

    // Check whether the lighting state has changed and install a mode uniform.
    // TODO: fix this
    if ( original && original->getMode(GL_LIGHTING) != osg::StateAttribute::INHERIT )
    {
        needNewStateSet = true;
        osg::StateAttribute::GLModeValue value = current->getMode(GL_LIGHTING);
        newStateSet->addUniform( Registry::shaderFactory()->createUniformForGLMode(GL_LIGHTING, value) );
    }
    
    // start generating the shader source.
    GenBuffers buf;
    buf._stateSet = newStateSet.get();

    // if the stateset changes any texture attributes, we need a new virtual program:
    if (current->getTextureAttributeList().size() > 0)
    {
        bool wroteTexelDecl = false;

        // Loop over all possible texture image units.
        int maxUnit = Registry::capabilities().getMaxGPUTextureUnits();

        for( int unit = 0; unit < maxUnit; ++unit )
        {
            if ( !wroteTexelDecl )
            {
                buf._fragBody << INDENT << MEDIUMP "vec4 texel; \n";
                wroteTexelDecl = true;
            }

            osg::Texture* tex = dynamic_cast<osg::Texture*>( current->getTextureAttribute(unit, osg::StateAttribute::TEXTURE) );

            if (accept(tex) && !ImageUtils::isFloatingPointInternalFormat(tex->getInternalFormat()))
            {
                osg::TexGen* texgen = dynamic_cast<osg::TexGen*>(current->getTextureAttribute(unit, osg::StateAttribute::TEXGEN));
                osg::TexEnv* texenv = dynamic_cast<osg::TexEnv*>(current->getTextureAttribute(unit, osg::StateAttribute::TEXENV));
                osg::TexMat* texmat = dynamic_cast<osg::TexMat*>(current->getTextureAttribute(unit, osg::StateAttribute::TEXMAT));
                osg::PointSprite* sprite = dynamic_cast<osg::PointSprite*>(current->getTextureAttribute(unit, osg::StateAttribute::POINTSPRITE));
                
                if ( apply(tex, texgen, texenv, texmat, sprite, unit, buf) == true )
                {
                    needNewStateSet = true;
                }
            }
        }
    }

    // Process the state attributes.
    osg::StateSet::AttributeList& attrs = current->getAttributeList();
    if ( apply(attrs, buf) )
    {
        needNewStateSet = true;
    }

    if ( needNewStateSet )
    {
        std::string version = GLSL_VERSION_STR;

        std::string vertHeadSource;
        vertHeadSource = buf._vertHead.str();

        std::string vertBodySource;
        vertBodySource = buf._vertBody.str();


        if ( !vertHeadSource.empty() || !vertBodySource.empty() )
        {
            std::string vertSource = Stringify()
                << "#version " << version << "\n" GLSL_PRECISION "\n"
                << vertHeadSource
                << "void " VERTEX_FUNCTION "(inout vec4 vertex_view)\n{\n"
                << vertBodySource
                << "}\n";

            vp->setFunction(VERTEX_FUNCTION, vertSource, ShaderComp::LOCATION_VERTEX_VIEW, 0.5f);
        }


        std::string fragHeadSource;
        fragHeadSource = buf._fragHead.str();

        std::string fragBodySource;
        fragBodySource = buf._fragBody.str();

        if ( !fragHeadSource.empty() || !fragBodySource.empty() )
        {
            std::string fragSource = Stringify()
                << "#version " << version << "\n" GLSL_PRECISION "\n"
                << fragHeadSource
                << "void " FRAGMENT_FUNCTION "(inout vec4 color)\n{\n"
                << fragBodySource
                << "}\n";

            vp->setFunction(FRAGMENT_FUNCTION, fragSource, ShaderComp::LOCATION_FRAGMENT_COLORING, 0.5f);
        }
    }

    if ( needNewStateSet )
    {
        replacement = newStateSet.get();
    }
    return replacement.valid();
}


bool
ShaderGenerator::apply(osg::Texture*     tex, 
                       osg::TexGen*      texgen,
                       osg::TexEnv*      texenv,
                       osg::TexMat*      texmat,
                       osg::PointSprite* sprite,
                       int               unit,
                       GenBuffers&       buf)
{
   bool ok = true;

   buf._vertHead << "varying " MEDIUMP "vec4 " TEX_COORD << unit << ";\n";
   buf._fragHead << "varying " MEDIUMP "vec4 " TEX_COORD << unit << ";\n";

   apply( texgen, unit, buf );
   apply( texmat, unit, buf );

   if ( sprite )
   {
      apply(sprite, unit, buf);
   }
   else if ( dynamic_cast<osg::Texture1D*>(tex) )
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
   else if ( dynamic_cast<osg::TextureCubeMap*>(tex) )
   {
       apply(static_cast<osg::TextureCubeMap*>(tex), unit, buf);
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
            buf._stateSet
                ->getOrCreateUniform(texEnvColorUniform, osg::Uniform::FLOAT_VEC4)
                ->set( texenv->getColor() );
        }
    }

    // See http://www.opengl.org/sdk/docs/man/xhtml/glTexEnv.xml
    switch( blendingMode )
    {
    case osg::TexEnv::REPLACE:
        buf._fragBody
            << INDENT "color = texel; \n";
        break;
    case osg::TexEnv::MODULATE:
        buf._fragBody
            << INDENT "color = color * texel; \n";
        break;
    case osg::TexEnv::DECAL:
        buf._fragBody
            << INDENT "color.rgb = color.rgb * (1.0 - texel.a) + (texel.rgb * texel.a); \n";
        break;
    case osg::TexEnv::BLEND:
        buf._fragHead
            << "uniform " MEDIUMP "vec4 " TEXENV_COLOR << unit << "\n;";
        buf._fragBody
            << INDENT "color.rgb = color.rgb * (1.0 - texel.rgb) + (" << TEXENV_COLOR << unit << ".rgb * texel.rgb); \n"
            << INDENT "color.a   = color.a * texel.a; \n";
        break;
    case osg::TexEnv::ADD:
    default:
        buf._fragBody
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
        // Handle different TexGen modes.
        // From the GLSL Orange Book.
        switch( texgen->getMode() )
        {
        case osg::TexGen::OBJECT_LINEAR:
            buf._vertBody
                << INDENT "{\n"
                << INDENT TEX_COORD << unit << " = "
                <<      "gl_Vertex.x*gl_ObjectPlaneS[" <<unit<< "] + "
                <<      "gl_Vertex.y*gl_ObjectPlaneT[" <<unit<< "] + "
                <<      "gl_Vertex.z*gl_ObjectPlaneR[" <<unit<< "] + "
                <<      "gl_Vertex.w*gl_ObjectPlaneQ[" <<unit<< "]; \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::EYE_LINEAR:
            buf._vertBody
                << INDENT "{\n"
                << INDENT TEX_COORD << unit << " = "
                <<      "vertex_view.x*gl_EyePlaneS[" <<unit<< "] + "
                <<      "vertex_view.y*gl_EyePlaneT[" <<unit<< "] + "
                <<      "vertex_view.z*gl_EyePlaneR[" <<unit<< "] + "
                <<      "vertex_view.w*gl_EyePlaneQ[" <<unit<< "]; \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::SPHERE_MAP:
            buf._vertHead
                << "varying vec3 vp_Normal;\n";
            buf._vertBody 
                << INDENT "{\n" // scope it in case there are > 1
                << INDENT "vec3 view_vec = normalize(vertex_view.xyz/vertex_view.w); \n"
                << INDENT "vec3 r = reflect(view_vec, vp_Normal);\n"
                << INDENT "r.z += 1.0; \n"
                << INDENT "float m = 2.0 * sqrt(dot(r,r)); \n"
                << INDENT TEX_COORD << unit << " = vec4(r.x/m + 0.5, r.y/m + 0.5, 0.0, 1.0); \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::REFLECTION_MAP:
            buf._vertHead
                << "varying vec3 vp_Normal;\n";
            buf._vertBody
                << INDENT "{\n"
                << INDENT "vec3 view_vec = normalize(vertex_view.xyz/vertex_view.w);\n"
                << INDENT TEX_COORD << unit << " = vec4(reflect(view_vec, vp_Normal), 1.0); \n"
                << INDENT "}\n";
            break;

        case osg::TexGen::NORMAL_MAP:
            buf._vertHead
                << "varying vec3 vp_Normal;\n";
            buf._vertBody
                << INDENT "{\n"
                << INDENT TEX_COORD << unit << " = vec4(vp_Normal, 1.0); \n"
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
            buf._vertBody
                << INDENT << TEX_COORD << unit << " = gl_MultiTexCoord" << unit << ";\n";
        }
        else
        {
            OE_INFO << LC
                << "Texture coordinate on unit (" << unit << ") "
                << "requires a custom vertex attribute (osg_MultiTexCoord" << unit << ")."
                << std::endl;

            buf._vertBody 
                << INDENT << TEX_COORD << unit << " = osg_MultiTexCoord" << unit << ";\n";
        }
    }

    return true;
}

bool
ShaderGenerator::apply(osg::TexMat* texmat, int unit, GenBuffers& buf)
{
    if ( accept(texmat) )
    {
        std::string texMatUniform = Stringify() << TEX_MATRIX << unit;

        buf._vertHead << "uniform mat4 " << texMatUniform << ";\n";
        buf._vertBody << INDENT << TEX_COORD << unit << " = " << texMatUniform << " * " << TEX_COORD<<unit << ";\n";

        buf._stateSet
            ->getOrCreateUniform(texMatUniform, osg::Uniform::FLOAT_MAT4)
            ->set( texmat->getMatrix() );
    }

    return true;
}

bool
ShaderGenerator::apply(osg::Texture1D* tex, int unit, GenBuffers& buf)
{
    buf._fragHead << "uniform sampler1D " SAMPLER << unit << ";\n";
    buf._fragBody << INDENT "texel = texture1D(" SAMPLER << unit << ", " TEX_COORD << unit << ".x);\n";
    buf._stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_1D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::Texture2D* tex, int unit, GenBuffers& buf)
{
    buf._fragHead << "uniform sampler2D " SAMPLER << unit << ";\n";
    buf._fragBody << INDENT "texel = texture2D(" SAMPLER << unit << ", " TEX_COORD << unit << ".xy);\n";
    buf._stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_2D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::Texture3D* tex, int unit, GenBuffers& buf)
{
    buf._fragHead << "uniform sampler3D " SAMPLER << unit << ";\n";
    buf._fragBody << INDENT "texel = texture3D(" SAMPLER << unit << ", " TEX_COORD << unit << ".xyz);\n";
    buf._stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_3D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::TextureRectangle* tex, int unit, GenBuffers& buf)
{
    buf._vertHead << "#extension GL_ARB_texture_rectangle : enable\n";

    buf._fragHead << "uniform sampler2DRect " SAMPLER << unit << ";\n";
    buf._fragBody << INDENT "texel = texture2DRect(" SAMPLER << unit << ", " TEX_COORD << unit << ".xy);\n";
    buf._stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_2D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::Texture2DArray* tex, int unit, GenBuffers& buf)
{    
    buf._fragHead <<  "#extension GL_EXT_texture_array : enable \n";    

    buf._fragHead << "uniform sampler2DArray " SAMPLER << unit << ";\n";
    buf._fragBody << INDENT "texel = texture2DArray(" SAMPLER << unit << ", " TEX_COORD << unit << ".xyz);\n";
    buf._stateSet->getOrCreateUniform( Stringify() << SAMPLER << unit, osg::Uniform::SAMPLER_2D_ARRAY )->set( unit );         

    return true;
}

bool
ShaderGenerator::apply(osg::TextureCubeMap* tex, int unit, GenBuffers& buf)
{
    std::string sampler = Stringify() << SAMPLER << unit;
    buf._fragHead << "uniform samplerCube " << sampler << ";\n";
    buf._fragBody << INDENT "texel = textureCube(" << sampler << ", " TEX_COORD << unit << ".xyz);\n";
    buf._stateSet->getOrCreateUniform( sampler, osg::Uniform::SAMPLER_CUBE )->set( unit );         

    return true;
}

bool
ShaderGenerator::apply(osg::PointSprite* tex, int unit, GenBuffers& buf)
{
    if ( !buf.requireVersion(120) ) return false;

    std::string sampler = Stringify() << SAMPLER << unit;
    buf._fragHead << "uniform sampler2D " << sampler << ";\n";
    buf._fragBody << INDENT << "texel = texture2D(" << sampler << ", gl_PointCoord);\n";
    buf._stateSet->getOrCreateUniform( sampler, osg::Uniform::SAMPLER_2D )->set( unit );

    return true;
}

bool
ShaderGenerator::apply(osg::StateSet::AttributeList& attrs, GenBuffers& buf)
{
    bool addedSomething = false;

    for(osg::StateSet::AttributeList::iterator i = attrs.begin(); i != attrs.end(); ++i)
    {
        osg::StateAttribute* attr = i->second.first.get();
        if ( apply(attr, buf) )
        {
            addedSomething = true;
        }
    }

    return addedSomething;
}

bool
ShaderGenerator::apply(osg::StateAttribute* attr, GenBuffers& buf)
{
    // NOP for now.
    return false;
}
