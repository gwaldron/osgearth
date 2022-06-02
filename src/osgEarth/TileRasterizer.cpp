/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/Metrics>
#include <osgEarth/CameraUtils>
#include <osgViewer/Renderer>
#include <osgViewer/Viewer>
#include <osg/BlendFunc>
#include <osg/CullFace>
#include <osgDB/WriteFile>

#ifndef GL_COPY_WRITE_BUFFER
#define GL_COPY_WRITE_BUFFER 0x8F37
#endif

#define LC "[TileRasterizer] "

// Set this to use an NVIDIA Fermi+ Copy Buffer Object for fast readback
// https://community.khronos.org/t/nvidia-dual-copy-engines/62499/19
// See Page 22 of this deck:
// https://on-demand.gputechconf.com/gtc/2012/presentations/S0356-GTC2012-Texture-Transfers.pdf
// Note: we may need to disable this for AMD
// NOTE: THIS IS COMMENTED OUT B/C IT WAS CAUSING ISSUES IN NSIGHT.
// Furthermore, calling glGetError just after glCopyBufferSubData causes
// the code the break. Why? No one knows.
//#define USE_CBO

// Set to use GPU sample queries to determine whether any data was rendered.
// This will not work here because of threading overlap and multiple query objects;
// i.e. you cannot start one query which another query of the same target is
// already running.
//#define USE_QUERY

using namespace osgEarth;
using namespace osgEarth::Util;

// This is the number of renders to use. This number is chosen 
// to accomodate a round robin setup in multi-threaded OSG mode.
#define NUM_RENDERERS_PER_GC 3

TileRasterizer::Renderer::Renderer(unsigned width, unsigned height)
{
    _tex = new osg::Texture2D();
    _tex->setTextureSize(width, height);
    _tex->setSourceFormat(GL_RGBA);
    _tex->setInternalFormat(GL_RGBA8);
    _tex->setSourceType(GL_UNSIGNED_BYTE);
    _dataSize = width * height * 4u; // for RGBA

    // set up the FBO camera
    _rtt = new osg::Camera();
    _rtt->setCullingActive(false);
    _rtt->setClearColor(osg::Vec4(0, 0, 0, 0));
    _rtt->setClearMask(GL_COLOR_BUFFER_BIT);
    _rtt->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    _rtt->setRenderOrder(osg::Camera::POST_RENDER);
    _rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    _rtt->setImplicitBufferAttachmentMask(0, 0);
    _rtt->setSmallFeatureCullingPixelSize(0.0f);
    _rtt->setViewMatrix(osg::Matrix::identity());
    _rtt->setViewport(0, 0, width, height);
    _rtt->attach(osg::Camera::COLOR_BUFFER0, _tex.get());
}

osg::Image*
TileRasterizer::Renderer::createImage() const
{
    osg::Image* image = new osg::Image();

    image->allocateImage(
        _tex->getTextureWidth(),
        _tex->getTextureHeight(),
        1, // depth
        _tex->getSourceFormat(),
        _tex->getSourceType());

    image->setInternalTextureFormat(
        _tex->getInternalFormat());

    return image;
}

void
TileRasterizer::Renderer::releaseGLObjects(osg::State* state) const
{
    if (_rtt.valid())
        _rtt->releaseGLObjects(state);

    if (_tex.valid())
        _tex->releaseGLObjects(state);

    _query = nullptr;
    _pbo = nullptr;
    _cbo = nullptr;
}

void
TileRasterizer::Renderer::resizeGLObjectBuffers(unsigned size)
{
    if (_rtt.valid())
        _rtt->resizeGLObjectBuffers(size);

    if (_tex.valid())
        _tex->resizeGLObjectBuffers(size);
}

void
TileRasterizer::Job::useRenderer(TileRasterizer::Renderer::Ptr renderer)
{
    // Assign a rendering context to this job and
    // configure the RTT camera with this job's node and extent.

    _renderer = renderer;

    renderer->_rtt->setProjectionMatrixAsOrtho2D(
        _extent.xMin(), _extent.xMax(),
        _extent.yMin(), _extent.yMax());

    renderer->_rtt->removeChildren(0, renderer->_rtt->getNumChildren());

    renderer->_rtt->addChild(_node);

    renderer->_phase = Renderer::RENDER;
}

TileRasterizer::TileRasterizer(unsigned width, unsigned height) :
    _width(width),
    _height(height)
{
    setCullingActive(false);

    auto blend = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    auto cullface = new osg::CullFace();

    // default no-op shader
    auto vp = new VirtualProgram();
    vp->setName(typeid(*this).name());
    vp->setInheritShaders(false);

    _rttStateSet = new osg::StateSet();
    GLUtils::setLighting(_rttStateSet.get(), 0);
    _rttStateSet->setAttributeAndModes(blend, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
    _rttStateSet->setAttributeAndModes(cullface, 0);
    _rttStateSet->setAttribute(vp);
}


TileRasterizer::~TileRasterizer()
{
    //nop
}

void
TileRasterizer::install(GLObjects::Ptr gc)
{
    for (unsigned i = 0; i < NUM_RENDERERS_PER_GC; ++i)
    {
        Renderer::Ptr r = std::make_shared<Renderer>(_width, _height);

        r->_uid = osgEarth::createUID();
        r->_rtt->getOrCreateStateSet()->merge(*_rttStateSet.get());

        r->_rtt->setPostDrawCallback(new DrawCallback(
            [this](osg::RenderInfo& ri) {this->postDraw(ri); }));

        gc->_renderers.add(r);
    }
}

Future<TileRasterizer::Job::Result>
TileRasterizer::render(osg::Node* node, const GeoExtent& extent)
{
    // make a new job and push it on the queue.
    Job::Ptr job = std::make_shared<Job>();
    job->_uid = osgEarth::createUID();
    job->_node = node;
    job->_extent = extent;

    // retrieve the future so we can return it to the caller:
    Future<Job::Result> result = job->_promise.getFuture();

    // put it on the queue:
    _jobQ.push(job);

    // all set.
    return result;
}

void
TileRasterizer::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        if (!_jobQ.empty())
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
            if (!cv)
                return;

            const osg::Camera* camera = cv->getCurrentCamera();

            if (camera == nullptr ||
                camera->getGraphicsContext() == nullptr ||
                CameraUtils::isShadowCamera(camera) ||
                CameraUtils::isDepthCamera(camera) ||
                CameraUtils::isPickCamera(camera))
            {
                return;
            }

            // find an initialize (if necessary) the GC-specific state for this camera
            GLObjects::Ptr& gc = GLObjects::get(_globjects, *camera->getGraphicsContext()->getState());
            if (gc == nullptr)
            {
                gc = std::make_shared<GLObjects>();
                install(gc);
            }

            // Find the next unused renderer. If it is in use, just bail out
            // and come back next time.
            Renderer::Ptr renderer = gc->_renderers.next();
            if (renderer.use_count() != 2)
                return;

            // Ready the next job:
            Job::Ptr job = _jobQ.take_front();
            if (job)
            {
                // assign a context to this job:
                job->useRenderer(renderer);

                // cull the RTT:
                renderer->_rtt->accept(nv);

                // queue for rendering.
                gc->_renderQ.push(job);
            }
        }
    }

    osg::Node::traverse(nv);
}

#define OE_TEST OE_DEBUG

void
TileRasterizer::Renderer::allocate(osg::State& state)
{
    if (_pbo == nullptr || !_pbo->valid())
    {
        _pbo = GLBuffer::create(GL_PIXEL_PACK_BUFFER_ARB, state);
        _pbo->bind();
        _pbo->debugLabel("TileRasterizer");
        _pbo->unbind();

#ifdef USE_CBO
        // dedicated copy buffer to take advantage of the Fermi+
        // copy engines. Rumor has it this slows things down on AMD
        // so we might want to disable it for them
        _cbo = GLBuffer::create(GL_COPY_WRITE_BUFFER, state);
        _cbo->bind();
        _cbo->debugLabel("TileRasterizer");
        _cbo->unbind();
#endif
    }

    if (_pbo->size() < _dataSize)
    {
        if (_cbo)
        {
            // when using a copy engine, GL_STATIC_COPY ensures a GPU->GPU
            // transfer to the copy buffer for later fast readback.
            _cbo->bind();
            _cbo->bufferData(_dataSize, nullptr, GL_STATIC_COPY);
            _cbo->unbind();

            _cbo->bind();
            _cbo->bufferData(_dataSize, nullptr, GL_STREAM_READ);
            _cbo->unbind();
        }
        else
        {
            _pbo->bind();
            _pbo->bufferData(_dataSize, nullptr, GL_STREAM_READ);
            _pbo->unbind();
        }
    }
}

GLuint
TileRasterizer::Renderer::query(osg::State& state)
{
    GLuint samples = 1;

    if (_query)
        _query->getResult(&samples);

    if (samples > 0)
    {
        // make the target texture current so we can read it back.
        _tex->apply(state);

        // begin the asynchronous transfer from texture to PBO
        _pbo->bind();

        // should return immediately
        glGetTexImage(
            _tex->getTextureTarget(),
            0, // mip level
            _tex->getSourceFormat(),
            _tex->getSourceType(),
            nullptr);

        if (_cbo)
        {
            // two-stage copy speeds things up on fermi allegedly
            _cbo->bind();
            _pbo->copyBufferSubData(_cbo, 0, 0, _dataSize);
            _cbo->unbind();
        }

        _pbo->unbind();
    }

    return samples;
}

osg::ref_ptr<osg::Image>
TileRasterizer::Renderer::readback(osg::State& state)
{
    osg::ref_ptr<osg::Image> result = createImage();

    GLBuffer::Ptr readback_buf =
        _cbo ? _cbo :
        _pbo;

    readback_buf->bind();

    void* ptr = readback_buf->map(GL_READ_ONLY_ARB);
    if (ptr)
    {
        ::memcpy(
            result->data(),
            ptr,
            _dataSize);

        readback_buf->unmap();
    }
    else
    {
        // Error. Unable to map a pointer to the PBO. Massive fail.
        OE_SOFT_ASSERT(ptr != nullptr, "glMapBuffer failed to map to PBO");
    }

    readback_buf->unbind();

    return result;
}

#define INV_QUERY 0
#define INV_READBACK 1

void
TileRasterizer::postDraw(osg::RenderInfo& ri)
{
    osg::State& state = *ri.getState();
    GLObjects::Ptr& gc = GLObjects::get(_globjects, state);
    Job::Ptr job = gc->_renderQ.take_front();
    OE_HARD_ASSERT(job != nullptr);

    // Check to see if the client still wants the result:
    if (job->_promise.isCanceled())
        return;

    job->_renderer->allocate(state);

    // GPU task delegate:
    auto gpu_task = [job](osg::State& state, Promise<Job::Result>& promise, int invocation)
    {
        if (promise.isAbandoned())
        {
            OE_DEBUG << "Job " << job << " canceled" << std::endl;
            return false; // done
        }

        // is the query still running?
        if (invocation == INV_QUERY)
        {
            OE_GL_ZONE_NAMED("TileRasterizer/QUERY");
            GLuint samples = job->_renderer->query(state);

            if (samples > 0)
            {
                return true; // come back later when the results are ready.
            }
            else
            {
                // no samples drawn? we are done with an empty result.
                promise.resolve(nullptr);
                return false;
            }
        }

        else if (invocation == INV_READBACK)
        {
            OE_GL_ZONE_NAMED("TileRasterizer/READBACK");

            Job::Result result = job->_renderer->readback(state);

            // all done:
            promise.resolve(result);
            return false;
        }

        // bad dates
        OE_HARD_ASSERT(false, "Bad dates.");
    };


#if 0
    if (gpu_task(state, job->_promise, INV_QUERY))
        gpu_task(state, job->_promise, INV_READBACK);
#else
    // Queue up our GPU job (using our existing Promise object)
    GLPipeline::get(state)->dispatch<Job::Result>(gpu_task, job->_promise);
#endif
}

void
TileRasterizer::releaseGLObjects(osg::State* state) const
{
    osg::Node::releaseGLObjects(state);

    for(unsigned i=0; i< _globjects.size(); ++i)
    {
        if (_globjects[i])
        {
            for (auto& r : _globjects[i]->_renderers)
            {
                r->releaseGLObjects(state);
            }
        }
    }
}

void
TileRasterizer::resizeGLObjectBuffers(unsigned size)
{
    osg::Node::resizeGLObjectBuffers(size);

    for (unsigned i = 0; i < _globjects.size(); ++i)
    {
        if (_globjects[i])
        {
            for (auto& r : _globjects[i]->_renderers)
            {
                r->resizeGLObjectBuffers(size);
            }
        }
    }

    if (size > _globjects.size())
    {
        _globjects.resize(size);
    }
}
