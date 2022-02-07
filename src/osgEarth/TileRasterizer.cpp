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

    // retrieve the future so we can return it to the caller:
    Future<Job::Result> result = job->_promise.getFuture();

    // put it on the queue:
    _jobQ.push(job);

    // all set.
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
        if (!_jobQ.empty())
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

#define OE_TEST OE_DEBUG

void
TileRasterizer::postDraw(osg::RenderInfo& ri)
{
    Job::Ptr job = _renderQ.take_front();
    OE_HARD_ASSERT(job != nullptr);

    // Check to see if the client still wants the result:
    if (job->_promise.isCanceled())
        return;

    // GPU task delegate:
    auto gpu_task = [job](osg::State& state, Promise<Job::Result>& promise, int invocation)
    {
        // invocations:
        constexpr int RENDER = 0;
        constexpr int QUERY = 1;
        constexpr int READBACK = 2;

        if (promise.isAbandoned())
        {
            OE_DEBUG << "Job " << job << " canceled" << std::endl;
            return false; // done
        }

        Renderer::Ptr& renderer = job->_renderer;
        Renderer::GCState& gs = renderer->_gs[state.getContextID()];

        if (invocation == RENDER)
        {
            if (gs.pbo == nullptr)
            {
                gs.pbo = GLBuffer::create(
                    GL_PIXEL_PACK_BUFFER_ARB, state, "TileRasterizer");

#ifdef USE_CBO
                // dedicated copy buffer to take advantage of the Fermi+
                // copy engines. Rumor has it this slows things down on AMD
                // so we might want to disable it for them
                gs.cbo = GLBuffer::create(
                    GL_COPY_WRITE_BUFFER, state, "TileRasterizer");
#endif
            }

            if (gs.pbo->size() < renderer->_dataSize)
            {
                if (gs.cbo)
                {
                    // when using a copy engine, GL_STATIC_COPY ensures a GPU->GPU
                    // transfer to the copy buffer for later fast readback.
                    gs.pbo->bind();
                    gs.pbo->bufferData(renderer->_dataSize, nullptr, GL_STATIC_COPY);
                    gs.pbo->unbind();

                    gs.cbo->bind();
                    gs.cbo->bufferData(renderer->_dataSize, nullptr, GL_STREAM_READ);
                    gs.cbo->unbind();
                }
                else
                {
                    gs.pbo->bind();
                    gs.pbo->bufferData(renderer->_dataSize, nullptr, GL_STREAM_READ);
                    gs.pbo->unbind();
                }
            }

            return true; // wait until the next frame (for render/query to finish)
        }

        // is the query still running?
        else if (invocation == QUERY)
        {
            OE_GL_ZONE_NAMED("TileRasterizer/QUERY");

            GLuint samples = 1;
            if (gs.query)
                gs.query->getResult(&samples);

            if (samples > 0)
            {
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

                if (gs.cbo)
                {
                    // two-stage copy speeds things up on fermi allegedly
                    gs.cbo->bind();
                    gs.pbo->copyBufferSubData(gs.cbo, 0, 0, renderer->_dataSize);
                    gs.cbo->unbind();
                }

                gs.pbo->unbind();

                return true; // come back later when the results are ready.
            }
            else
            {
                // no samples drawn? we are done with an empty result.
                promise.resolve(nullptr);
                return false;
            }
        }

        else if (invocation == READBACK)
        {
            OE_GL_ZONE_NAMED("TileRasterizer/READBACK");

            osg::ref_ptr<osg::Image> result = renderer->createImage();

            GLBuffer::Ptr readback_buf =
                gs.cbo ? gs.cbo :
                gs.pbo;

            readback_buf->bind();

            void* ptr = readback_buf->map(GL_READ_ONLY_ARB);
            if (ptr)
            {
                ::memcpy(
                    result->data(),
                    ptr,
                    renderer->_dataSize);

                readback_buf->unmap();
            }
            else
            {
                // Error. Unable to map a pointer to the PBO. Massive fail.
                OE_SOFT_ASSERT(ptr != nullptr, "glMapBuffer failed to map to PBO");
            }

            readback_buf->unbind();

            // all done:
            promise.resolve(result);
            return false;
        }

        // bad dates
        OE_HARD_ASSERT(false, "Bad dates.");
    };


    // Queue up our GPU job (using our existing Promise object)
    GLPipeline::get()->dispatch<Job::Result>(gpu_task, job->_promise);
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
