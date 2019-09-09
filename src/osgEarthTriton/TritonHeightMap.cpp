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
#include "TritonHeightMap"
#include "TritonContext"
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/TerrainEngineNode>

#define LC "[TritonHeightMap] "

using namespace osgEarth::Triton;

namespace
{    
    const char* vertexShader =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "#pragma import_defines(OE_TRITON_MASK_MATRIX);\n"

        "// terrain SDK:\n"
        "float oe_terrain_getElevation(); \n"

        "out float oe_triton_elev;\n"
        
        "#ifdef OE_TRITON_MASK_MATRIX\n"
        "out vec2 maskCoords;\n"
        "uniform mat4 OE_TRITON_MASK_MATRIX;\n"
        "vec4 oe_layer_tilec;\n"
        "#endif\n"

        "void oe_triton_setupHeightMap(inout vec4 unused) \n"
        "{ \n"
        "    oe_triton_elev = oe_terrain_getElevation(); \n"
        "#ifdef OE_TRITON_MASK_MATRIX\n"
        "    maskCoords = (OE_TRITON_MASK_MATRIX * oe_layer_tilec).st;\n"
        "#endif\n"
        "} \n";

    // The fragment shader simply takes the texture index that we generated
    // in the vertex shader and does a texture lookup. In this case we're
    // just wholesale replacing the color, so if the map had any existing
    // imagery, this will overwrite it.

    const char* fragmentShader =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        "#pragma import_defines(OE_TRITON_MASK_SAMPLER);\n"

        "in float oe_triton_elev;\n"

        "#ifdef OE_TRITON_MASK_SAMPLER\n"
        "in vec2 maskCoords;\n"
        "uniform sampler2D OE_TRITON_MASK_SAMPLER;\n"
        "#endif\n"

        "out vec4 out_height; \n"

        "void oe_triton_drawHeightMap(inout vec4 unused) \n"
        "{ \n"
#ifdef DEBUG_HEIGHTMAP
          // Map to black = -500m, white = +500m
          "   float nHeight = clamp(oe_triton_elev / 1000.0 + 0.5, 0.0, 1.0);\n"
#else
          "   float nHeight = oe_triton_elev;\n"

          "#ifdef OE_TRITON_MASK_SAMPLER\n"
          "    float mask = texture(OE_TRITON_MASK_SAMPLER, maskCoords).a;\n"
          "    nHeight *= mask; \n"
          "#endif\n"

#endif
        "    out_height = vec4( nHeight, 0.0, 0.0, 1.0 ); \n"
        "} \n";

    struct TerrainDirtyCallback : public osgEarth::TerrainCallback
    {
        osg::observer_ptr<TritonHeightMap> _hm;
        TerrainDirtyCallback(TritonHeightMap* hm) : _hm(hm) { }
        void onTileAdded(const osgEarth::TileKey&, osg::Node*, osgEarth::TerrainCallbackContext&)
        {
            osg::ref_ptr<TritonHeightMap> hm;
            if (_hm.lock(hm))
                hm->dirty();
        }
    };
}

TritonHeightMap::TritonHeightMap() :
_texSize(0u),
_internalFormat((GLint)0),
_sourceFormat((GLenum)0)
{
    setCullingActive(false);
}

TritonHeightMap::~TritonHeightMap()
{
    osgEarth::TerrainEngineNode* t = dynamic_cast<osgEarth::TerrainEngineNode*>(_terrain.get());
    if (t)
    {
        t->getTerrain()->removeTerrainCallback(static_cast<TerrainDirtyCallback*>(_terrainCallback.get()));
        _terrainCallback = NULL;
    }
}

void
TritonHeightMap::setTerrain(osg::Node* node)
{
    _terrain = node;

    osgEarth::TerrainEngineNode* t = dynamic_cast<osgEarth::TerrainEngineNode*>(node);
    if (t)
    {
        TerrainDirtyCallback* cb = new TerrainDirtyCallback(this);
        t->getTerrain()->addTerrainCallback(cb);
        _terrainCallback = cb;
    }
}

void
TritonHeightMap::setMaskLayer(const osgEarth::ImageLayer* layer)
{
    _maskLayer = layer;
}

void
TritonHeightMap::SetDirty::operator()(CameraLocal& local)
{
    local._mvpw.makeIdentity();
}

void
TritonHeightMap::dirty()
{
    SetDirty setDirty;
    _local.forEach(setDirty);
}

bool
TritonHeightMap::configure(unsigned texSize, osg::State& state)
{
    bool result = true;

    if (_texSize == 0u)
    {
        // first time through, single-lane and set up FBO parameters.
        static Threading::Mutex s_mutex;
        s_mutex.lock();

        if (_texSize == 0u)
        {
            _texSize = texSize;
            if (!getBestFBOConfig(state, _internalFormat, _sourceFormat))
            {
                result = false;
            }
        }

        s_mutex.unlock();
    }
    return result;
}

namespace {
    struct Format {
        Format(GLint i, GLenum s, const std::string& n) :
            internalFormat(i), sourceFormat(s), name(n) { }
        GLint internalFormat;
        GLenum sourceFormat;
        std::string name;
    };
}

bool
TritonHeightMap::getBestFBOConfig(osg::State& state, GLint& out_internalFormat, GLenum& out_sourceFormat)
{
#ifdef GL_LUMINANCE_FLOAT16_ATI
#   define GL_LUMINANCE_FLOAT16_ATI 0x881E
#endif

    std::vector<Format> formats;

#ifdef GL_R16F
    formats.push_back(Format(GL_R16F, GL_RED, "GL_R16F"));
#endif
#ifdef GL_LUMINANCE16F_ARB
    formats.push_back(Format(GL_LUMINANCE16F_ARB, GL_LUMINANCE, "GL_LUMINANCE16F_ARB"));
#endif
#ifdef GL_LUMINANCE_FLOAT16_ATI
    formats.push_back(Format(GL_LUMINANCE_FLOAT16_ATI, GL_LUMINANCE, "GL_LUMINANCE_FLOAT16_ATI"));
#endif
#ifdef GL_R32F
    formats.push_back(Format(GL_R32F, GL_RED, "GL_R32F"));
#endif
#ifdef GL_LUMINANCE32F_ARB
    formats.push_back(Format(GL_LUMINANCE32F_ARB, GL_LUMINANCE, "GL_LUMINANCE32F_ARB"));
#endif
#ifdef GL_RGB16F_ARB
    formats.push_back(Format(GL_RGB16F_ARB, GL_RGB, "GL_RGB16F_ARB"));
#endif
#ifdef GL_RGBA16F_ARB
    formats.push_back(Format(GL_RGBA16F_ARB, GL_RGBA, "GL_RGBA16F_ARB"));
#endif
#ifdef GL_RGB32F_ARB
    formats.push_back(Format(GL_RGB32F_ARB, GL_RGB, "GL_RGB32F_ARB"));
#endif
#ifdef GL_RGBA32F_ARB
    formats.push_back(Format(GL_RGBA32F_ARB, GL_RGBA, "GL_RGBA32F_ARB"));
#endif
    

    osg::GLExtensions* ext = osg::GLExtensions::Get(state.getContextID(), true);

    osg::State::CheckForGLErrors check = state.getCheckForGLErrors();
    state.setCheckForGLErrors(state.NEVER_CHECK_GL_ERRORS);

    bool found = false;

    for(int i=0; i<formats.size() && !found; ++i)
    {
        const Format& format = formats[i];

        osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D();
        tex->setTextureSize(1, 1);
        tex->setInternalFormat( format.internalFormat );
        tex->setSourceFormat  ( format.sourceFormat );

        osg::ref_ptr<osg::FrameBufferObject> fbo = new osg::FrameBufferObject();
        fbo->setAttachment( osg::Camera::COLOR_BUFFER0, osg::FrameBufferAttachment(tex.get()) );

        fbo->apply( state );

        GLenum status = ext->glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);

        fbo->releaseGLObjects( &state );
        tex->releaseGLObjects( &state );

        if ( status == GL_FRAMEBUFFER_COMPLETE_EXT )
        {
            out_internalFormat = format.internalFormat;
            out_sourceFormat   = format.sourceFormat;
            OE_INFO << LC << "Height map format = " << format.name << std::endl;
            found = true;
        }
    }

    state.setCheckForGLErrors(check);

    return found;
}

bool
TritonHeightMap::isConfigurationComplete() const
{
    return
        _texSize > 0u &&
        _internalFormat != (GLint)0 &&
        _sourceFormat != (GLenum)0;
}

void
TritonHeightMap::setup(CameraLocal& local, const std::string& name)
{
    // make sure the FBO params are configured:
    if (!isConfigurationComplete())
        return;

    local._frameNum = 0u;

    local._tex = new osg::Texture2D();
    local._tex->setName(Stringify() << "Triton HM (" << name << ")");
    local._tex->setTextureSize(_texSize, _texSize);
    local._tex->setInternalFormat( _internalFormat );
    local._tex->setSourceFormat( _sourceFormat );
    local._tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
    local._tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);

    // Triton prob doesn't need this but it's good practice
    if (_sourceFormat == GL_RED)
    {
        local._tex->setSwizzle(osg::Vec4i(GL_RED, GL_RED, GL_RED, GL_ONE));
    }
    
    local._rtt = new osg::Camera();
    local._rtt->setName(local._tex->getName());
    local._rtt->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    local._rtt->setClearMask(GL_COLOR_BUFFER_BIT); //GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    local._rtt->setClearColor(osg::Vec4(-1000.0, -1000.0, -1000.0, 1.0f));
    local._rtt->setViewport(0, 0, _texSize, _texSize);
    local._rtt->setRenderOrder(osg::Camera::PRE_RENDER);
    local._rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    local._rtt->setImplicitBufferAttachmentMask(0, 0);
    local._rtt->attach(osg::Camera::COLOR_BUFFER0, local._tex.get());
    //local._rtt->setCullMask( ~TRITON_OCEAN_MASK );
    local._rtt->setAllowEventFocus(false);
    local._rtt->setDrawBuffer(GL_FRONT);
    local._rtt->setReadBuffer(GL_FRONT);
    local._rtt->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    
    // TODO: create this once and just re-use it for all RTT cameras
    osg::StateSet* rttSS = local._rtt->getOrCreateStateSet();
    
    osgEarth::VirtualProgram* rttVP = osgEarth::VirtualProgram::getOrCreate(rttSS);
    rttVP->setName("Triton Height Map");
    rttVP->setFunction( "oe_triton_setupHeightMap", vertexShader,   ShaderComp::LOCATION_VERTEX_MODEL);
    rttVP->setFunction( "oe_triton_drawHeightMap",  fragmentShader, ShaderComp::LOCATION_FRAGMENT_OUTPUT);
    rttVP->setIsAbstract(true);
    rttVP->setInheritShaders(false);

    osg::StateAttribute::OverrideValue off = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE;
    rttSS->setDefine("OE_IS_DEPTH_CAMERA");
    rttSS->setDefine("OE_TERRAIN_RENDER_IMAGERY", off);
    rttSS->setDefine("OE_TERRAIN_RENDER_NORMAL_MAP", off);
    rttSS->setDefine("OE_TERRAIN_BLEND_IMAGERY", off);
    rttSS->setDefine("OE_TERRAIN_MORPH_GEOMETRY", off);

    osg::ref_ptr<const osgEarth::ImageLayer> maskLayer;
    if (_maskLayer.lock(maskLayer))
    {
        rttSS->setDefine("OE_TRITON_MASK_SAMPLER", maskLayer->shareTexUniformName().get());
        rttSS->setDefine("OE_TRITON_MASK_MATRIX", maskLayer->shareTexMatUniformName().get());
        OE_INFO << LC << "Using mask layer \"" << maskLayer->getName() << "\", sampler=" << maskLayer->shareTexUniformName().get() << ", matrix=" << maskLayer->shareTexMatUniformName().get() << std::endl;
    }

    if (_terrain.valid())
    {
        local._rtt->addChild(_terrain.get());
    }
    else
    {
        OE_WARN << LC << "Illegal: no terrain set (must call setTerrain)" << std::endl;
    }
}

#define MAXABS4(A,B,C,D) \
    osg::maximum(fabs(A), osg::maximum(fabs(B), osg::maximum(fabs(C),fabs(D))))

void
TritonHeightMap::update(CameraLocal& local, const osg::Camera* cam, osgEarth::Horizon* horizon)
{
    osg::Vec3d eye = osg::Vec3d(0,0,0) * cam->getInverseViewMatrix();

    double hd = horizon->getDistanceToVisibleHorizon();

    local._rtt->setProjectionMatrix(osg::Matrix::ortho(-hd, hd, -hd, hd, 1.0, eye.length()));
    local._rtt->setViewMatrixAsLookAt(eye, osg::Vec3d(0.0,0.0,0.0), osg::Vec3d(0.0,0.0,1.0));

    static const osg::Matrixd scaleBias(
        0.5, 0.0, 0.0, 0.0,
        0.0, 0.5, 0.0, 0.0,
        0.0, 0.0, 0.5, 0.0,
        0.5, 0.5, 0.5, 1.0);

    // Matrix that Triton will use to position the heightmap for sampling.
    local._texMatrix = local._rtt->getViewMatrix() * local._rtt->getProjectionMatrix() * scaleBias;
}

void
TritonHeightMap::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = osgEarth::Culling::asCullVisitor(nv);
        const osg::Camera* camera = cv->getCurrentCamera();
        if (camera)
        {
            CameraLocal& local = _local.get(camera);

            if (isConfigurationComplete())
            {
                // create the RTT for this camera on first encounter:
                if (!local._rtt.valid())
                {
                    setup(local, camera->getName());
                }

                // only update when the MVPW changes.
                if (local._mvpw != *cv->getMVPW())
                {
                    // update the RTT based on the current camera:
                    osgEarth::Horizon* horizon = osgEarth::Horizon::get(nv);
                    update(local, camera, horizon);

                    // finally, traverse the camera to build the height map.
                    local._rtt->accept(nv);
                    
                    local._frameNum = nv.getFrameStamp()->getFrameNumber();
                    local._mvpw = *cv->getMVPW();
                }
           }
           else
           {
               OE_DEBUG << LC << "Configuration not yet complete..." << std::endl;
           }
        }
    }
}

bool
TritonHeightMap::getTextureAndMatrix(osg::RenderInfo& ri, GLint& out_texName, osg::Matrix& out_matrix)
{
    if (!isConfigurationComplete())
        return false;

    CameraLocal& local = _local.get(ri.getCurrentCamera());
    if (!local._tex.valid())
        return false;

    // did the texture change?
    OE_DEBUG << "FN=" << ri.getState()->getFrameStamp()->getFrameNumber() << "; localFN=" << local._frameNum << std::endl;

    if (ri.getState()->getFrameStamp()->getFrameNumber() > local._frameNum)
        return false;

    osg::Texture::TextureObject* obj = local._tex->getTextureObject(ri.getContextID());
    if (!obj)
        return false;

    out_texName = obj->id();
    out_matrix = local._texMatrix;
    return true;
}
