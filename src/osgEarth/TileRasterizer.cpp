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

#define LC "[TileRasterizer] "

#ifndef GL_ANY_SAMPLES_PASSED
#define GL_ANY_SAMPLES_PASSED 0x8C2F
#endif

#ifndef GL_READ_ONLY
#define GL_READ_ONLY 0x88B8
#endif

#ifndef GL_COLOR_ATTACHMENT0
#define GL_COLOR_ATTACHMENT0 0x8CE0
#endif

// Set this to use a Pixel Buffer Object for DMA readback.
#define USE_PBO

// Set to use GPU sample queries to determine whether any data was rendered.
#define USE_QUERY

using namespace osgEarth;
using namespace osgEarth::Util;


TileRasterizer::TileRasterizer(unsigned width, unsigned height)
{
    setCullingActive(false);

    _cx = std::make_shared<RenderContext>();

    _cx->_width = width;
    _cx->_height = height;

    _cx->_tex = new osg::Texture2D();
    _cx->_tex->setTextureSize(_cx->_width, _cx->_height);
    _cx->_tex->setSourceFormat(GL_RGBA);
    _cx->_tex->setInternalFormat(GL_RGBA8);
    _cx->_tex->setSourceType(GL_UNSIGNED_BYTE);

    // set up the FBO camera
    _cx->_rtt = new osg::Camera();
    _cx->_rtt->setCullingActive(false);
    _cx->_rtt->setClearColor(osg::Vec4(0,0,0,0));
    _cx->_rtt->setClearMask(GL_COLOR_BUFFER_BIT);
    _cx->_rtt->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    _cx->_rtt->setRenderOrder(osg::Camera::POST_RENDER);
    _cx->_rtt->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    _cx->_rtt->setImplicitBufferAttachmentMask(0, 0);
    _cx->_rtt->setSmallFeatureCullingPixelSize(0.0f);
    _cx->_rtt->setViewMatrix(osg::Matrix::identity());
    _cx->_rtt->setViewport(0, 0, width, height);
    _cx->_rtt->attach(osg::Camera::COLOR_BUFFER0, _cx->_tex.get());

#if defined(USE_PBO) || defined(USE_QUERY)
    _cx->_rtt->setPreDrawCallback(new DrawCallback(
        [this](osg::RenderInfo& ri) {this->preDraw(ri); }));
#endif

    _cx->_rtt->setPostDrawCallback(new DrawCallback(
        [this](osg::RenderInfo& ri) {this->postDraw(ri); }));

    _cx->_activeJob = nullptr;
    _cx->_rttActive = false;

    // GL objects
    _cx->_samplesQuery = 0;
    _cx->_pbo = 0;

    osg::StateSet* ss = _cx->_rtt->getOrCreateStateSet();
    //ss->setMode(GL_BLEND, 1);
    ss->setMode(GL_CULL_FACE, 0);
    ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
    GLUtils::setLighting(ss, 0);

    // default no-op shader
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->setName("TileRasterizer");
    vp->setInheritShaders(false);
}

TileRasterizer::~TileRasterizer()
{
    //nop
}

Future<osg::ref_ptr<osg::Image>>
TileRasterizer::render(osg::Node* node, const GeoExtent& extent)
{
    RenderJob* job = new RenderJob();
    job->_node = node;
    job->_extent = extent;
    Future<osg::ref_ptr<osg::Image>> result = job->_promise.getFuture();

    ScopedMutexLock lock(_queue);
    _queue.push(job);

    return result;
}

void
TileRasterizer::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        // disallow shadow cameras or depth cameras
        if (CameraUtils::isShadowCamera(static_cast<osgUtil::CullVisitor*>(&nv)->getCurrentCamera()) ||
            CameraUtils::isDepthCamera(static_cast<osgUtil::CullVisitor*>(&nv)->getCurrentCamera()))
        {
            return;
        }

        // only enter if an RTT is NOT currently active:
        if (!_queue.empty())
        {
            if (!_cx->_rttActive.exchange(true))
            {
                OE_SOFT_ASSERT(_cx->_activeJob == nullptr);

                ScopedMutexLock lock(_queue);

                if (!_queue.empty()) // double check
                {
                    _cx->_activeJob = _queue.front();
                    _queue.pop();

                    OE_SOFT_ASSERT(_cx->_activeJob->_node.valid());

                    // This is OK, just means the client disappeared.
                    //OE_SOFT_ASSERT(!_cx->_activeJob->_promise.isAbandoned());

                    _cx->_rtt->setProjectionMatrixAsOrtho2D(
                        _cx->_activeJob->_extent.xMin(), _cx->_activeJob->_extent.xMax(),
                        _cx->_activeJob->_extent.yMin(), _cx->_activeJob->_extent.yMax());

                    _cx->_rtt->removeChildren(0, _cx->_rtt->getNumChildren());

                    _cx->_rtt->addChild(_cx->_activeJob->_node);

                    _cx->_rtt->accept(nv);
                }
                else
                {
                    // double check failed, reset active state.
                    _cx->_rttActive.exchange(false);
                }
            }
        }
    }

    osg::Node::traverse(nv);
}

void
TileRasterizer::preDraw(osg::RenderInfo& ri)
{
    OE_SOFT_ASSERT(_cx->_rttActive == true);
    OE_SOFT_ASSERT(_cx->_activeJob != nullptr);

    if (_cx->_activeJob->_promise.isAbandoned())
        return;

    unsigned id = ri.getContextID();
    osg::GLExtensions* ext = osg::GLExtensions::Get(id, true);

#ifdef USE_PBO
    if (_cx->_pbo == 0 && ext->isPBOSupported)
    {
        // Allocate a pixel buffer object for DMA readback
        unsigned size = _cx->_width * _cx->_height * 4u;
        ext->glGenBuffers(1, &_cx->_pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _cx->_pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, size, 0, GL_STREAM_READ);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
    }
#endif

#ifdef USE_QUERY
    if (_cx->_samplesQuery == 0)
    {
        // Allocate a sample-counting query
        ext->glGenQueries(1, &_cx->_samplesQuery);
    }

    _cx->_samples = 0u;
    ext->glBeginQuery(GL_ANY_SAMPLES_PASSED, _cx->_samplesQuery);
#endif
}

void
TileRasterizer::postDraw(osg::RenderInfo& ri)
{
    OE_SOFT_ASSERT(_cx->_rttActive == true);
    OE_SOFT_ASSERT(_cx->_activeJob != nullptr);

    osg::ref_ptr<osg::Image> image;
    osg::GLExtensions* ext = ri.getState()->get<osg::GLExtensions>();

    // finalize the samples query (if in use)
    if (_cx->_samplesQuery > 0)
    {
        OE_PROFILING_ZONE_NAMED("glEndQuery/glGet");
        ext->glEndQuery(GL_ANY_SAMPLES_PASSED);
        ext->glGetQueryObjectuiv(_cx->_samplesQuery, GL_QUERY_RESULT, &_cx->_samples);
    }

    if (!_cx->_activeJob->_promise.isAbandoned())
    {
        if (_cx->_samples > 0 || _cx->_samplesQuery == 0)
        {
            // create our new target image:
            image = new osg::Image();
            image->allocateImage(_cx->_width, _cx->_height, 1, _cx->_tex->getSourceFormat(), _cx->_tex->getSourceType());
            image->setInternalTextureFormat(_cx->_tex->getInternalFormat());

            OE_PROFILING_ZONE_NAMED("Readback");

            // make the target texture current so we can read it back.
            _cx->_tex->apply(*ri.getState());

            if (_cx->_pbo > 0)
            {
                // Use the PBO to perform a DMA transfer (faster than straight glReadPixels)
                ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _cx->_pbo);

                // begin the transfer from GPU to PBO
                glGetTexImage(
                    GL_TEXTURE_2D,
                    0,
                    _cx->_tex->getSourceFormat(),
                    _cx->_tex->getSourceType(),
                    nullptr);

                // blocks until the transfer is complete. Later we could break this
                // up so the map occurs on a subsequent frame.
                GLubyte* src = (GLubyte*)ext->glMapBuffer(
                    GL_PIXEL_PACK_BUFFER_ARB,
                    GL_READ_ONLY_ARB);

                OE_SOFT_ASSERT(src != nullptr);
                if (src)
                {
                    memcpy(image->data(), src, image->getTotalSizeInBytes());
                    ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
                }
                ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
            }
            else
            {
                // read the texture directly into the allocated memory
                glGetTexImage(
                    GL_TEXTURE_2D,
                    0,
                    _cx->_tex->getSourceFormat(),
                    _cx->_tex->getSourceType(),
                    image->data());
            }
        }

        if (_cx->_samplesQuery == 0)
        {
            // when not using sample queries, check the image to see if anything
            // was actually rendered
            if (ImageUtils::isEmptyImage(image.get()))
                image = nullptr;
        }

        _cx->_activeJob->_promise.resolve(image);
    }

    delete _cx->_activeJob;
    _cx->_activeJob = nullptr;
    _cx->_rttActive.exchange(false);
}

void
TileRasterizer::releaseGLObjects(osg::State* state) const
{
    osg::Node::releaseGLObjects(state);

    if (_cx != nullptr)
    {
        if (_cx->_rtt.valid())
            _cx->_rtt->releaseGLObjects(state);
        if (_cx->_tex.valid())
            _cx->_tex->releaseGLObjects(state);

        if (state)
        {
            osg::GLExtensions* ext = state->get<osg::GLExtensions>();
            if (_cx->_pbo > 0)
                ext->glDeleteBuffers(1, &_cx->_pbo);
            if (_cx->_samplesQuery > 0)
                ext->glDeleteQueries(1, &_cx->_samplesQuery);
        }

        _cx->_pbo = 0;
        _cx->_samplesQuery = 0;
    }
}

void
TileRasterizer::resizeGLObjectBuffers(unsigned size)
{
    osg::Node::resizeGLObjectBuffers(size);

    if (_cx != nullptr)
    {
        if (_cx->_rtt.valid())
            _cx->_rtt->resizeGLObjectBuffers(size);
        if (_cx->_tex.valid())
            _cx->_tex->resizeGLObjectBuffers(size);
    }
}
