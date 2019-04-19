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
#include <osgEarth/TileRasterizer>
#include <osgEarth/NodeUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/GLUtils>

#define LC "[TileRasterizer] "

#ifndef GL_ANY_SAMPLES_PASSED
#define GL_ANY_SAMPLES_PASSED 0x8C2F
#endif

using namespace osgEarth;

namespace
{
    template<typename T>
    struct PreDrawRouter : public osg::Camera::DrawCallback
    {
        T* _object;
        PreDrawRouter(T* object) : _object(object) { }
        void operator()(osg::RenderInfo& renderInfo) const {
            _object->preDraw(renderInfo);
        }
    };

    template<typename T>
    struct PostDrawRouter : public osg::Camera::DrawCallback
    {
        T* _object;
        PostDrawRouter(T* object) : _object(object) { }
        void operator()(osg::RenderInfo& renderInfo) const {
            _object->postDraw(renderInfo);
        }
    };
}

namespace
{
    const char* distort =
        "#version " GLSL_VERSION_STR "\n"
        "uniform float oe_rasterizer_f; \n"
        "void oe_rasterizer_clip(inout vec4 vert) { \n"
        "    float h = (vert.y + vert.w)/(2.0*vert.w); \n"
        "    float d = 1.0; \n" //1.2299; \n" //mix(1.0, oe_rasterizer_f, h); \n"
        "    vert.x *= d; \n"
        "} \n";
}

TileRasterizer::TileRasterizer() :
osg::Camera()
{
    // active an update traversal.
    ADJUST_EVENT_TRAV_COUNT(this, +1);
    setCullingActive(false);

    // set up the RTT camera.
    setClearColor(osg::Vec4(0,0,0,0));
    setClearMask(GL_COLOR_BUFFER_BIT);
    setReferenceFrame(ABSOLUTE_RF);
    //setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    setRenderOrder(PRE_RENDER);
    setRenderTargetImplementation(FRAME_BUFFER_OBJECT);
    setImplicitBufferAttachmentMask(0, 0);
    setSmallFeatureCullingPixelSize(0.0f);
    setViewMatrix(osg::Matrix::identity());

    osg::StateSet* ss = getOrCreateStateSet();

    ss->setMode(GL_BLEND, 1);
    ss->setMode(GL_CULL_FACE, 0);
    GLUtils::setLighting(ss, 0);
    
    this->setPreDrawCallback(new PreDrawRouter<TileRasterizer>(this));
    this->setPostDrawCallback(new PostDrawRouter<TileRasterizer>(this));

#if 0 // works in OE, not in VRV :(
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits();
    traits->sharedContext = 0L;
    traits->doubleBuffer = false;
    traits->x = 0, traits->y = 0, traits->width = 256, traits->height = 256;
    traits->format = GL_RGBA;
    traits->red = 8;
    traits->green = 8;
    traits->blue = 8;
    traits->alpha = 8;
    traits->depth = 0;
    osg::GraphicsContext* gc = osg::GraphicsContext::createGraphicsContext(traits);
    setGraphicsContext(gc);
#endif
    setDrawBuffer(GL_FRONT);
    setReadBuffer(GL_FRONT);

    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->setName("TileRasterizer");
    vp->setInheritShaders(false);

    // Someday we might need this to undistort rasterizer cells. We'll see
#if 0
    vp->setFunction("oe_rasterizer_clip", distort, ShaderComp::LOCATION_VERTEX_CLIP);
    _distortionU = new osg::Uniform("oe_rasterizer_f", 1.0f);
    ss->addUniform(_distortionU.get());
#endif

    _samplesQuery.resize(64u);
    _samplesQuery.setAllElementsTo(INT_MAX);
}

TileRasterizer::~TileRasterizer()
{
    OE_DEBUG << LC << "~TileRasterizer\n";
}

void
TileRasterizer::push(osg::Node* node, osg::Texture* texture, const GeoExtent& extent)
{
    Threading::ScopedMutexLock lock(_mutex);

    _pendingJobs.push(Job());
    Job& job = _pendingJobs.back();
    job._node = node;
    job._texture = texture;
    job._extent = extent;
}

void
TileRasterizer::ReadbackImage::readPixels(
    int x, int y, int width, int height,
    GLenum pixelFormat, GLenum type, int packing)
{
    OE_DEBUG << LC << "ReadPixels in context " << _ri->getContextID() << std::endl;

    glPixelStorei(GL_PACK_ALIGNMENT, _packing);
    glPixelStorei(GL_PACK_ROW_LENGTH, _rowLength);

    if (getPixelBufferObject())
    {
        _ri->getState()->bindPixelBufferObject(getPixelBufferObject()->getOrCreateGLBufferObject(_ri->getContextID()));
        glReadPixels(x, y, width, height, getPixelFormat(), getDataType(), 0L);
    }
    else
    {
        // synchronous:
        glReadPixels(x, y, width, height, getPixelFormat(), getDataType(), _data);
    }
}


Threading::Future<osg::Image>
TileRasterizer::push(osg::Node* node, unsigned size, const GeoExtent& extent)
{    
    Threading::ScopedMutexLock lock(_mutex);

    _pendingJobs.push(Job());
    Job& job = _pendingJobs.back();

    job._node = node;
    job._extent = extent;
    job._image = new ReadbackImage();
    job._image->allocateImage(size, size, 1, GL_RGBA, GL_UNSIGNED_BYTE);

    //job._imagePBO = new osg::PixelBufferObject(job._image.get());
    //job._imagePBO->setTarget(GL_PIXEL_PACK_BUFFER);
    //job._imagePBO->setUsage(GL_STREAM_READ);
    //job._image->setPixelBufferObject(job._imagePBO.get());

    return job._imagePromise.getFuture();
}

void
TileRasterizer::accept(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR && getBufferAttachmentMap().empty())
    {
        return;
    }
    else
    {
        osg::Camera::accept(nv);
    }
}


void
TileRasterizer::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.EVENT_VISITOR)
    {
        Threading::ScopedMutexLock lock(_mutex);

        if (!_finishedJobs.empty())
        {
            Job& job = _finishedJobs.front();
            removeChild(job._node.get());

            // If the job didn't write any fragments, return a NULL image.
            if (job._fragmentsWritten > 0)
                job._imagePromise.resolve(job._image.get());
            else
                job._imagePromise.resolve(0L);

            _finishedJobs.pop(); 
            detach(osg::Camera::COLOR_BUFFER);
            dirtyAttachmentMap();
        }

        if (!_pendingJobs.empty() && _readbackJobs.empty() && _finishedJobs.empty())
        {
            Job& job = _pendingJobs.front();

            // Configure a top-down orothographic camera:
            setProjectionMatrixAsOrtho2D(
                job._extent.xMin(), job._extent.xMax(),
                job._extent.yMin(), job._extent.yMax());

            // Job includes a texture to populate:
            if (job._texture.valid())
            {
                // Setup the viewport and attach to the new texture
                setViewport(0, 0, job._texture->getTextureWidth(), job._texture->getTextureHeight());
                attach(COLOR_BUFFER, job._texture.get(), 0u, 0u, /*mipmap=*/false);
                dirtyAttachmentMap();
            }

            // Job includes an image to populate, so use the built-in FBO target texture:
            else if (job._image.valid())
            {
                setViewport(0, 0, job._image->s(), job._image->t());
                attach(COLOR_BUFFER, job._image.get(), 0u, 0u);
                dirtyAttachmentMap();
            }

            // Add the node to the scene graph so it'll get rendered.
            addChild(job._node.get());

            // If this job has a readback image, push the job to the next queue
            // where it will be picked up for readback.
            if (job._image.valid())
            {
                _readbackJobs.push(job);
            }

            // Remove the texture from the queue.
            _pendingJobs.pop();
            //OE_INFO << LC
            //    << "P=" << _pendingJobs.size()
            //    << ", R=" << _readbackJobs.size()
            //    << ", F=" << _finishedJobs.size()
            //    << std::endl;
        }
    }

    //if (!getBufferAttachmentMap().empty())
    else if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        if (!getBufferAttachmentMap().empty())
        {
            osg::Camera::traverse(nv);
        }
    }
}

void
TileRasterizer::preDraw(osg::RenderInfo& ri) const
{
    if (!_readbackJobs.empty())
    {
        Threading::ScopedMutexLock lock(_mutex);

        if (!_readbackJobs.empty()) // double check!
        {
            Job& job = _readbackJobs.front();
            if (job._image.valid())
            {
                job._image.get()->_ri = &ri;
            }

            // allocate a query on demand for this GC:
            osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(),true);
            GLuint& query = _samplesQuery[ri.getContextID()];
            if (query == INT_MAX)
            {
                ext->glGenQueries(1, &query);
            }

            // initiate a query for samples passing the fragment shader
            // to see whether we drew anything.
            ext->glBeginQuery(GL_ANY_SAMPLES_PASSED, query);
        }
    }
}

void
TileRasterizer::postDraw(osg::RenderInfo& ri) const
{
    if (!_readbackJobs.empty())
    {
        Threading::ScopedMutexLock lock(_mutex);

        if (!_readbackJobs.empty()) // double check!
        {
            Job& job = _readbackJobs.front();

            // get the results of the query and store the
            // # of fragments generated in the job.
            osg::GLExtensions* ext = osg::GLExtensions::Get(ri.getContextID(), true);
            GLuint query = _samplesQuery[ri.getContextID()];
            ext->glEndQuery(GL_ANY_SAMPLES_PASSED);
            ext->glGetQueryObjectuiv(query, GL_QUERY_RESULT, &job._fragmentsWritten);

            _finishedJobs.push(job);
            _readbackJobs.pop();
        }
    }
}
