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
#include <osgViewer/Renderer>
#include <osgViewer/Viewer>

#define LC "[TileRasterizer] "

#ifndef GL_ANY_SAMPLES_PASSED
#define GL_ANY_SAMPLES_PASSED 0x8C2F
#endif

using namespace osgEarth;
using namespace osgEarth::Util;


TileRasterizer::RenderOperation::RenderOperation(osg::Node* node, const GeoExtent& extent, TileRasterizer::RenderData& renderData) :
    osg::GraphicsOperation("TileRasterizer", false),
    _node(node),
    _extent(extent),
    _renderData(renderData)
{
    //nop
}

Threading::Future<osg::Image>
TileRasterizer::RenderOperation::getFuture()
{
    return _promise.getFuture();
}

void 
TileRasterizer::RenderOperation::operator () (osg::GraphicsContext* context)
{
    osg::Camera* camera = _renderData._sv->getCamera();
    const osg::GraphicsContext* gc = camera->getGraphicsContext();

    _image = new osg::Image();
    _image->allocateImage(_renderData._width, _renderData._height, 1, GL_RGBA, GL_UNSIGNED_BYTE);

    camera->detach(camera->COLOR_BUFFER);
    camera->attach(camera->COLOR_BUFFER, _image.get());
    camera->dirtyAttachmentMap();

    camera->setProjectionMatrixAsOrtho2D(
        _extent.xMin(), _extent.xMax(),
        _extent.yMin(), _extent.yMax());

    _renderData._sv->setSceneData(_node.get());
           
    osg::GLExtensions* ext = osg::GLExtensions::Get(gc->getState()->getContextID(), true);

    // initiate a query for samples passing the fragment shader
    // to see whether we drew anything.
    GLuint samples = 0u;

    GLuint query;
    ext->glGenQueries(1, &query);
    ext->glBeginQuery(GL_ANY_SAMPLES_PASSED, query);

    _renderData._sv->cull();
    _renderData._sv->draw();

    ext->glEndQuery(GL_ANY_SAMPLES_PASSED);
    ext->glGetQueryObjectuiv(query, GL_QUERY_RESULT, &samples);
    ext->glDeleteQueries(1, &query);

    if (samples > 0u)
    {
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, _image->data());

        //TODO: we can use this method if we wish to compress or otherwise reformat
        // the data on the GPU
        //_image->readImageFromCurrentTexture(gc->getState()->getContextID(), false);
    }
    else
    {
        _image = NULL;
    }

    _promise.resolve(_image.release());
}


TileRasterizer::TileRasterizer(unsigned width, unsigned height)
{
    osg::Camera* rtt = new osg::Camera();

    rtt->setCullingActive(false);

    // set up the FBO camera
    rtt->setClearColor(osg::Vec4(0,0,0,0));
    rtt->setClearMask(GL_COLOR_BUFFER_BIT);
    rtt->setReferenceFrame(rtt->ABSOLUTE_RF);
    rtt->setRenderOrder(rtt->PRE_RENDER);
    rtt->setRenderTargetImplementation(rtt->FRAME_BUFFER_OBJECT);
    rtt->setImplicitBufferAttachmentMask(0, 0);
    rtt->setSmallFeatureCullingPixelSize(0.0f);
    rtt->setViewMatrix(osg::Matrix::identity());
    rtt->setViewport(0, 0, width, height);

    osg::StateSet* ss = rtt->getOrCreateStateSet();
    ss->setMode(GL_BLEND, 1);
    ss->setMode(GL_CULL_FACE, 0);
    GLUtils::setLighting(ss, 0);

    // set up our off-screen GC
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;  
    traits->alpha = 8;
    traits->depth = 0;
    traits->glContextVersion = osg::DisplaySettings::instance()->getGLContextVersion();
    traits->glContextProfileMask = osg::DisplaySettings::instance()->getGLContextProfileMask();

    osg::GraphicsContext* gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    if (gc)
    {
        // set up a background thread to do the actual rendering from a queue
        gc->createGraphicsThread();

        // must realize the GC before starting the thread, as start() is where
        // the osg::State gets set up for the GC:
        gc->realize();
        gc->getGraphicsThread()->start();

        // assign our new GC to our RTT camera:
        rtt->setGraphicsContext(gc);
    }
    else
    {
        OE_WARN << LC << "Failed to create GC!" << std::endl;
    }

    // default no-op shader
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->setName("TileRasterizer");
    vp->setInheritShaders(false);

    // set up a container for our GL query
    _renderData._samplesQuery.resize(128u);
    _renderData._samplesQuery.setAllElementsTo(INT_MAX);

    // set up a sceneview to render the graph
    _renderData._sv = new osgUtil::SceneView();
    _renderData._sv->setAutomaticFlush(true);
    _renderData._sv->setGlobalStateSet(rtt->getOrCreateStateSet());
    _renderData._sv->setCamera(rtt, true);
    _renderData._sv->setDefaults(0u);
    _renderData._sv->getCullVisitor()->setIdentifier(new osgUtil::CullVisitor::Identifier());
    _renderData._sv->setState(gc->getState());
    _renderData._sv->setFrameStamp(new osg::FrameStamp());

    // pass in the dimensions
    _renderData._width = width;
    _renderData._height = height;
}

bool
TileRasterizer::valid() const
{
    return _renderData._sv.valid();
}

TileRasterizer::~TileRasterizer()
{
    if (_renderData._sv.valid())
    {
        osg::Camera* camera = _renderData._sv->getCamera();
        camera->getGraphicsContext()->getGraphicsThread()->cancel();
        camera->getGraphicsContext()->getGraphicsThread()->join();
        OE_DEBUG << LC << "~TileRasterizer\n";
    }
}


Threading::Future<osg::Image>
TileRasterizer::render(osg::Node* node, const GeoExtent& extent)
{
    if (_renderData._sv.valid() == false)
        return Threading::Future<osg::Image>();

    RenderOperation* op = new RenderOperation(node, extent, _renderData);
    Threading::Future<osg::Image> result = op->getFuture();

    _renderData._sv->getCamera()->getGraphicsContext()->getGraphicsThread()->add( op );

    return result;
}
