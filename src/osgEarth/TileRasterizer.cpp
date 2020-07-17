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

using namespace osgEarth;
using namespace osgEarth::Util;


TileRasterizer::RenderInstaller::RenderInstaller(TileRasterizer::RenderData& renderData) : 
    _renderData(renderData)
{
    setCullingActive(false);
    setUseDisplayList(false);
}

void
TileRasterizer::RenderInstaller::drawImplementation(osg::RenderInfo& ri) const
{
    // capture the GC and save it as our graphics op queue.
    osg::ref_ptr<osg::GraphicsContext> gc = _renderData._gc.get();
    if (gc.valid() == false)
    {
        static Threading::Mutex s_mutex(OE_MUTEX_NAME);
        gc = _renderData._gc.get();
        if (gc.valid() == false)
        {
            _renderData._gc = ri.getState()->getGraphicsContext();
            _renderData._sv->setState(ri.getState());
            OE_WARN << LC << "Installed on GC " << _renderData._gc.get() << std::endl;
        }
    }
}

TileRasterizer::RenderOperation::RenderOperation(osg::Node* node, const GeoExtent& extent, TileRasterizer::RenderData& renderData) :
    osg::GraphicsOperation("TileRasterizer", false),
    _node(node),
    _extent(extent),
    _renderData(renderData),
    _pass(0)
{
    //nop
}

Future<osg::Image>
TileRasterizer::RenderOperation::getFuture()
{
    return _promise.getFuture();
}

void 
TileRasterizer::RenderOperation::operator () (osg::GraphicsContext* gc)
{
    osg::Camera* camera = _renderData._sv->getCamera();

    _image = new osg::Image();
    _image->allocateImage(_renderData._width, _renderData._height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    _image->setInternalTextureFormat(GL_RGBA8);

    // attaching an image tells OSG to automatically call glReadPixels at the 
    // end of the RenderStage. No opportunity for async readback with a PBO however.
    camera->detach(camera->COLOR_BUFFER0);
    camera->attach(camera->COLOR_BUFFER0, _image.get());
    camera->dirtyAttachmentMap();

    camera->setProjectionMatrixAsOrtho2D(
        _extent.xMin(), _extent.xMax(),
        _extent.yMin(), _extent.yMax());

    _renderData._sv->setSceneData(_node.get());
           
    unsigned id = gc->getState()->getContextID();
    osg::GLExtensions* ext = osg::GLExtensions::Get(id, true);

    if (_renderData._samplesQuery[id] == INT_MAX)
    {
        ext->glGenQueries(1, &_renderData._samplesQuery[id]);
    }

    _renderData._sv->cull();

    // initiate a query for samples passing the fragment shader
    // to see whether we drew anything.
    GLuint samples = 0u;
    ext->glBeginQuery(GL_ANY_SAMPLES_PASSED, _renderData._samplesQuery[id]);

    _renderData._sv->draw();

    ext->glEndQuery(GL_ANY_SAMPLES_PASSED);
    ext->glGetQueryObjectuiv(_renderData._samplesQuery[id], GL_QUERY_RESULT, &samples);

    // nothing rendered? return a NULL image.
    if (samples == 0u)
        _image = NULL;

    _promise.resolve(_image.release());
}


TileRasterizer::TileRasterizer(unsigned width, unsigned height)
{
    _renderData._initialized = false;
    _renderData._width = width;
    _renderData._height = height;

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

    osg::StateSet* ss = rtt->getOrCreateStateSet();
    ss->setMode(GL_BLEND, 1);
    ss->setMode(GL_CULL_FACE, 0);
    GLUtils::setLighting(ss, 0);

    // default no-op shader
    VirtualProgram* vp = VirtualProgram::getOrCreate(ss);
    vp->setName("TileRasterizer");
    vp->setInheritShaders(false);

    // set up a container for our GL query
    _renderData._samplesQuery.resize(128u);
    _renderData._samplesQuery.setAllElementsTo(INT_MAX);
    _renderData._pbo.resize(128u);
    _renderData._pbo.setAllElementsTo(INT_MAX);

    // set up a sceneview to render the graph
    _renderData._sv = new osgUtil::SceneView();
    _renderData._sv->setAutomaticFlush(true);
    _renderData._sv->setGlobalStateSet(rtt->getOrCreateStateSet());
    _renderData._sv->setCamera(rtt, true);
    _renderData._sv->setDefaults(0u);
    _renderData._sv->getCullVisitor()->setIdentifier(new osgUtil::CullVisitor::Identifier());
    _renderData._sv->setFrameStamp(new osg::FrameStamp());

    _installer = new RenderInstaller(_renderData);
}

bool
TileRasterizer::valid() const
{
    return _renderData._sv.valid();
}

TileRasterizer::~TileRasterizer()
{
    //nop
}

osg::Node*
TileRasterizer::getNode() const
{
    return _installer.get();
}

Future<osg::Image>
TileRasterizer::render(osg::Node* node, const GeoExtent& extent)
{
    if (_renderData._sv.valid())
    {
        osg::ref_ptr<osg::GraphicsContext> gc = _renderData._gc.get();
        if (gc.valid())
        {
            osg::ref_ptr<RenderOperation> op = new RenderOperation(node, extent, _renderData);
            Future<osg::Image> result = op->getFuture();   
            gc->add(op.get());
            return result;
        }
    }

    return Future<osg::Image>();
}
