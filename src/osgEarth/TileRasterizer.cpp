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
#include <osgViewer/Renderer>
#include <osgViewer/Viewer>

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

using namespace osgEarth;
using namespace osgEarth::Util;

TileRasterizer::RenderOperation::RenderOperation(
    osg::Node* node, 
    const GeoExtent& extent,
    std::shared_ptr<TileRasterizer::RenderData> renderData) :

    _node(node),
    _extent(extent),
    _rd(renderData)
{
    //nop
}

osg::ref_ptr<osg::Image>
TileRasterizer::RenderOperation::run(osg::State* state)
{
    OE_PROFILING_ZONE_NAMED("TileRasterizer:RenderOperation");

    // lock the weak pointer to make sure the TileRasterizer still exists
    // and didn't disappear out from under us
    std::shared_ptr<RenderData> rd = _rd.lock();
    if (rd == nullptr)
    {
        return nullptr;
    }

    osg::Camera* camera = rd->_sv->getCamera();

    //unsigned id = gc->getState()->getContextID();
    unsigned id = state->getContextID();
    osg::GLExtensions* ext = osg::GLExtensions::Get(id, true);

#ifdef USE_PBO
    if (rd->_pbo == 0 && ext->isPBOSupported)
    {
        // Allocate a pixel buffer object for DMA readback
        unsigned size = rd->_width * rd->_height * 4u;
        ext->glGenBuffers(1, &rd->_pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, rd->_pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, size, 0, GL_STREAM_READ);
    }
#endif

    // Set up the projection to match the geo extents
    camera->setProjectionMatrixAsOrtho2D(
        _extent.xMin(), _extent.xMax(),
        _extent.yMin(), _extent.yMax());

    rd->_sv->setSceneData(_node.get());

    if (rd->_samplesQuery == 0)
    {
        // Allocate a sample-counting query
        ext->glGenQueries(1, &rd->_samplesQuery);
    }

    rd->_sv->cull();

    // initiate a query for samples passing the fragment shader
    // to see whether we drew anything.
    GLuint samples = 0u;
    ext->glBeginQuery(GL_ANY_SAMPLES_PASSED, rd->_samplesQuery);

    rd->_sv->draw();

    {
        OE_PROFILING_ZONE_NAMED("glEndQuery/glGet");
        ext->glEndQuery(GL_ANY_SAMPLES_PASSED);
        ext->glGetQueryObjectuiv(rd->_samplesQuery, GL_QUERY_RESULT, &samples);
    }

    osg::ref_ptr<osg::Image> image;

    // if we rendered any samples, allocate an image and copy it over.
    if (samples > 0u)
    {
        // create our new target image:
        image = new osg::Image();
        image->allocateImage(rd->_width, rd->_height, 1, rd->_tex->getSourceFormat(), rd->_tex->getSourceType());
        image->setInternalTextureFormat(rd->_tex->getInternalFormat());

        OE_PROFILING_ZONE_NAMED("Readback");

        // make the target texture current so we can read it back.
        rd->_tex->apply(*state);

        if (rd->_pbo > 0)
        {
            // Use the PBO to perform a DMA transfer (faster than straight glReadPixels)
            ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, rd->_pbo);
            glGetTexImage(GL_TEXTURE_2D, 0, rd->_tex->getSourceFormat(), rd->_tex->getSourceType(), 0);
            GLubyte* src = (GLubyte*)ext->glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
            if (src)
            {
                memcpy(image->data(), src, image->getTotalSizeInBytes());
                ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
            }
            ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
        }
        else
        {
            image->readImageFromCurrentTexture(id, false);
        }
    }
    else
    {
        //OE_INFO << "No data" << std::endl;
    }

    return image;
}


TileRasterizer::TileRasterizer(unsigned width, unsigned height)
{
    _renderData = std::make_shared<RenderData>();

    _renderData->_initialized = false;
    _renderData->_width = width;
    _renderData->_height = height;

    _renderData->_tex = new osg::Texture2D();
    _renderData->_tex->setTextureSize(_renderData->_width, _renderData->_height);
    _renderData->_tex->setSourceFormat(GL_RGBA);
    _renderData->_tex->setInternalFormat(GL_RGBA8);
    _renderData->_tex->setSourceType(GL_UNSIGNED_BYTE);

    // set up the FBO camera
    osg::Camera* rtt = new osg::Camera();
    rtt->setCullingActive(false);
    rtt->setClearColor(osg::Vec4(0,0,0,0));
    rtt->setClearMask(GL_COLOR_BUFFER_BIT);
    rtt->setReferenceFrame(rtt->ABSOLUTE_RF);
    rtt->setRenderOrder(rtt->PRE_RENDER);
    rtt->setRenderTargetImplementation(rtt->FRAME_BUFFER_OBJECT);
    rtt->setImplicitBufferAttachmentMask(0, 0);
    rtt->setSmallFeatureCullingPixelSize(0.0f);
    rtt->setViewMatrix(osg::Matrix::identity());
    rtt->setViewport(0, 0, width, height);
    rtt->attach(rtt->COLOR_BUFFER, _renderData->_tex.get());

    osg::StateSet* ss = rtt->getOrCreateStateSet();
    ss->setMode(GL_BLEND, 1);
    ss->setMode(GL_CULL_FACE, 0);
    GLUtils::setLighting(ss, 0);

    // default no-op shader
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->setName("TileRasterizer");
    vp->setInheritShaders(false);

    // GL objects
    _renderData->_samplesQuery = 0;
    _renderData->_pbo = 0;

    // set up a sceneview to render the graph
    _renderData->_sv = new osgUtil::SceneView();
    _renderData->_sv->setAutomaticFlush(true);
    _renderData->_sv->setGlobalStateSet(rtt->getOrCreateStateSet());
    _renderData->_sv->setCamera(rtt, true);
    _renderData->_sv->setDefaults(0u);
    _renderData->_sv->getCullVisitor()->setIdentifier(new osgUtil::CullVisitor::Identifier());
    _renderData->_sv->setFrameStamp(new osg::FrameStamp());
}

bool
TileRasterizer::valid() const
{
    return _renderData->_sv.valid();
}

TileRasterizer::~TileRasterizer()
{
    //nop
}

Future<osg::ref_ptr<osg::Image>>
TileRasterizer::render(osg::Node* node, const GeoExtent& extent)
{
    using ImageJob = GPUJob<osg::ref_ptr<osg::Image>>;

    ImageJob::Result result;

    if (_renderData->_sv.valid())
    {
        std::shared_ptr<RenderOperation> op = std::make_shared<RenderOperation>(
            node, extent, _renderData);

        result = ImageJob::dispatch(
            [op](osg::State* state, Cancelable* progress)
            {
                return op->run(state);
            }
        );
    }

    return result;
}
