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

// Set this to use a Pixel Buffer Object for DMA readback.
#define USE_PBO

// Set this to use an NVIDIA Copy Buffer Object for fast readback
// Note, this will only be effective with a separate HW transfer thread
// See Page 22 of this deck:
// https://on-demand.gputechconf.com/gtc/2012/presentations/S0356-GTC2012-Texture-Transfers.pdf
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
#define NUM_RENDERERS 4

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

#if 0
void
TileRasterizer::Renderer::preDraw(osg::State& state)
{
    GCState& gs = _gs[state.getContextID()];

#ifdef USE_QUERY
    if (gs.query == nullptr)
    {
        gs.query = GLQuery::create(
            GL_SAMPLES_PASSED_ARB, state, typeid(*this).name());
    }
    gs.query->begin();

    // transition to query phase.
    _phase = QUERY;
#else

    // transition to render phase.
    _phase = RENDER;
#endif
}
#endif

void
TileRasterizer::Renderer::releaseGLObjects(osg::State* state) const
{
    if (_rtt.valid())
        _rtt->releaseGLObjects(state);

    if (_tex.valid())
        _tex->releaseGLObjects(state);

    if (state)
    {
        GCState& gs = _gs[state->getContextID()];
        gs.pbo = nullptr;
        gs.query = nullptr;
    }
    else
    {
        _gs.setAllElementsTo(GCState());
    }
}

void
TileRasterizer::Renderer::resizeGLObjectBuffers(unsigned size)
{
    if (_rtt.valid())
        _rtt->resizeGLObjectBuffers(size);

    if (_tex.valid())
        _tex->resizeGLObjectBuffers(size);

    _gs.resize(size);
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

TileRasterizer::TileRasterizer(unsigned width, unsigned height)
{
    setCullingActive(false);

    auto blend = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    auto cullface = new osg::CullFace();

    // default no-op shader
    auto vp = new VirtualProgram();
    vp->setName(typeid(*this).name());
    vp->setInheritShaders(false);

    for (unsigned i = 0; i < NUM_RENDERERS; ++i)
    {
        Renderer::Ptr r = std::make_shared<Renderer>(width, height);

        r->_uid = osgEarth::createUID();

        osg::StateSet* ss = r->_rtt->getOrCreateStateSet();
        GLUtils::setLighting(ss, 0);
        ss->setAttributeAndModes(blend, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
        ss->setAttributeAndModes(cullface, 0);
        ss->setAttribute(vp);

        //r->_rtt->setPreDrawCallback(new DrawCallback(
        //    [this](osg::RenderInfo& ri) {this->preDraw(ri); }));

        r->_rtt->setPostDrawCallback(new DrawCallback(
            [this](osg::RenderInfo& ri) {this->postDraw(ri); }));

        _renderers.add(r);
    }
}


TileRasterizer::~TileRasterizer()
{
    //nop
}

Future<osg::ref_ptr<osg::Image>>
TileRasterizer::render(osg::Node* node, const GeoExtent& extent)
{
    // make a new job and push it on the queue.
    Job::Ptr job = std::make_shared<Job>();
    job->_uid = osgEarth::createUID();
    job->_node = node;
    job->_extent = extent;
    Future<osg::ref_ptr<osg::Image>> result = job->_promise.getFuture();

    _jobQ.push(job);

    // return a Future result.
    return result;
}

#define OE_SERIALIZE_SCOPE \
    static osgEarth::Threading::Mutex _oe_s_serialized_scope_mutex; \
    osgEarth::Threading::ScopedMutexLock _oe_s_serialized_scope_lock(_oe_s_serialized_scope_mutex)

void
TileRasterizer::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        if (!_jobQ.unsafe_empty())
        {
            const osg::Camera* camera = static_cast<osgUtil::CullVisitor*>(&nv)->getCurrentCamera();
            if (CameraUtils::isShadowCamera(camera) ||
                CameraUtils::isDepthCamera(camera) ||
                CameraUtils::isPickCamera(camera))
            {
                return;
            }

            OE_SERIALIZE_SCOPE;

            Job::Ptr job = _jobQ.take_front();
            if (job)
            {
                auto renderer = _renderers.next();
                OE_SOFT_ASSERT(renderer.use_count() == 2);

                // assign a context to this job:
                job->useRenderer(renderer);

                // cull the RTT:
                job->_renderer->_rtt->accept(nv);

                // queue for rendering.
                _renderQ.push(job);
            }
        }
    }

    osg::Node::traverse(nv);
}

#if 0
void
TileRasterizer::preDraw(osg::RenderInfo& ri)
{
    // do NOT remove the job yet - do that in postDraw
    Job::Ptr job = _renderQ.front();
    if (job)
    {
        job->_renderer->preDraw(*ri.getState());
    }
}
#endif

#define OE_TEST OE_DEBUG

void
TileRasterizer::postDraw(osg::RenderInfo& ri)
{
    Job::Ptr job = _renderQ.take_front();
    OE_HARD_ASSERT(job != nullptr);

    // Check to see if the client still wants the result:
    if (job->_promise.isAbandoned())
        return;

    osg::State& state = *ri.getState();

    OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
        << " : scheduling job " << job->_uid
        << " with renderer " << job->_renderer->_uid
        << std::endl;

    auto operation = [job](osg::State& state)
    {
        // is the job still valid?
        if (job->_promise.isAbandoned())
        {
            OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                << " : job abandoned " << job->_uid
                << " with renderer " << job->_renderer->_uid
                << std::endl;

            return false;
        }

        // was the job already resolved? (This should never happen)
        if (job->_promise.isResolved())
        {
            OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                << " : job already resolved " << job->_uid
                << " with renderer " << job->_renderer->_uid
                << std::endl;

            return false;
        }

        Renderer::Ptr& renderer = job->_renderer;
        Renderer::GCState& gs = renderer->_gs[state.getContextID()];

#ifdef USE_PBO
        if (gs.pbo == nullptr)
        {
            gs.pbo = GLBuffer::create(
                GL_PIXEL_PACK_BUFFER_ARB, state, "TileRasterizer");
        }

        GLenum pbo_usage = GL_STREAM_READ;

#ifdef USE_CBO
        if (gs.cbo == nullptr)
        {
            gs.cbo = GLBuffer::create(
                GL_COPY_WRITE_BUFFER, state, "TileRasterizer");
        }

        if (gs.cbo->size() < renderer->_dataSize)
        {
            gs.cbo->bind();
            gs.cbo->bufferData(renderer->_dataSize, nullptr, GL_STREAM_READ);
            gs.cbo->unbind();
        }
        pbo_usage = GL_STREAM_COPY;
#endif // USE_CBO

        if (gs.pbo->size() < renderer->_dataSize)
        {
            gs.pbo->bind();
            gs.pbo->bufferData(renderer->_dataSize, nullptr, pbo_usage);
            gs.pbo->unbind();
        }

#endif // USE_PBO

        if (renderer->_phase == Renderer::RENDER)
        {
            // wait until the next frame.
            renderer->_phase = Renderer::QUERY;
            return true;
        }

        // is the query still running?
        else if (renderer->_phase == Renderer::QUERY)
        {
            OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                << " : running query " << job->_uid
                << " with renderer " << job->_renderer->_uid
                << std::endl;

            // is the result ready?
            if (gs.query == nullptr || gs.query->isReady())
            {
                OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                    << " : query " << job->_uid << " is ready!"
                    << std::endl;

                // yes, read the result.
                GLuint samples = 1;

                if (gs.query)
                    gs.query->getResult(&samples);

                if (samples > 0)
                {
                    OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                        << " : posting a readback for " << job->_uid
                        << std::endl;

                    // make the target texture current so we can read it back.
                    renderer->_tex->apply(state);

                    // begin the asynchronous transfer from texture to PBO
                    gs.pbo->bind();

                    // should return immediately
                    glGetTexImage(
                        renderer->_tex->getTextureTarget(),
                        0, // mip level
                        renderer->_tex->getSourceFormat(),
                        renderer->_tex->getSourceType(),
                        nullptr);

#ifdef USE_CBO
                    // queue a buffer-to-buffer copy on the GPU
                    // should return immediately
                    gs.pbo->copyBufferSubData(
                        gs.cbo,
                        0, 0, renderer->_dataSize);
#endif

                    gs.pbo->unbind();

                    renderer->_phase = Renderer::READBACK;
                    return true; // come back later when the results are ready.
                }
                else
                {
                    OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                        << " : no samples drawn for " << job->_uid
                        << std::endl;

                    // no samples drawn? we are done with an empty result.
                    job->_promise.resolve(nullptr);
                    return false;
                }
            }
            else
            {
                OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                    << " : query " << job->_uid << " isn't ready yet.."
                    << std::endl;

                // nope, wait another frame.
                return true;
            }
        }

        else if (renderer->_phase == Renderer::READBACK)
        {
            OE_TEST << "Frame " << state.getFrameStamp()->getFrameNumber()
                << " : reading back " << job->_uid
                << " with context " << job->_renderer->_uid
                << std::endl;

            osg::ref_ptr<osg::Image> result = renderer->createImage();

#ifdef USE_CBO

            gs.cbo->getBufferSubData(
                0, renderer->_dataSize,
                renderer->_result->data());

#else
            gs.pbo->bind();

            void* ptr = gs.pbo->map(GL_READ_ONLY_ARB);
            if (ptr)
            {
                ::memcpy(
                    result->data(),
                    ptr,
                    renderer->_dataSize);

                gs.pbo->unmap();
            }
            else
            {
                // Error. Unable to map a pointer to the PBO. Massive fail.
                OE_SOFT_ASSERT(ptr != nullptr, "glMapBuffer failed to map to PBO");
            }

            gs.pbo->unbind();
#endif

            if (ImageUtils::isEmptyImage(result.get()))
                result = nullptr;

            job->_promise.resolve(result);

            return false; // all done.
        }

        // bad dates
        OE_HARD_ASSERT(false, "Logic error, should never get here!");
    };

    // schedule a new GPU operation on this GC.
    state.getGraphicsContext()->add(new GPUOperation(operation));

    // Test: run it all inline
    //while (operation(state));
}

void
TileRasterizer::releaseGLObjects(osg::State* state) const
{
    osg::Node::releaseGLObjects(state);

    for (auto& r : _renderers)
    {
        r->releaseGLObjects(state);
    }
}

void
TileRasterizer::resizeGLObjectBuffers(unsigned size)
{
    osg::Node::resizeGLObjectBuffers(size);

    for (auto& r : _renderers)
    {
        r->resizeGLObjectBuffers(size);
    }
}
