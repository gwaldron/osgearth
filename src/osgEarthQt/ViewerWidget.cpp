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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthQt/ViewerWidget>

#include <osgEarthUtil/EarthManipulator>

#include <osgGA/StateSetManipulator>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <QtGui>
#include <QtCore/QTimer>
#include <QWidget>

using namespace osgEarth;
using namespace osgEarth::QtGui;


ViewerWidget::ViewerWidget(osg::Node* scene)
{
    // create a new viewer (a simple osgViewer::Viewer)
    createViewer();

    // attach the scene graph provided by the user
    if (scene)
    {
        dynamic_cast<osgViewer::Viewer*>(_viewer.get())->setSceneData( scene );
    }

    // start up the paint event timer.
    installFrameTimer();
}

ViewerWidget::ViewerWidget(osgViewer::ViewerBase* viewer) :
_viewer( viewer )
{
    if ( !_viewer.valid() )
    {
        // create a viewer if the user passed in NULL
        createViewer();
    }

    else
    {
        // reconfigure all the viewer's views to use a Qt graphics context.
        osgViewer::ViewerBase::Views views;
        getViews( views );
        for( osgViewer::ViewerBase::Views::iterator v = views.begin(); v != views.end(); ++v )
        {
            reconfigure( *v );
        }

        // disable event setting on the viewer.
        viewer->setKeyEventSetsDone(0);
        viewer->setQuitEventSetsDone(false);
    }

    // start up the paint event timer.
    installFrameTimer();
}

ViewerWidget::~ViewerWidget()
{
    _timer.stop();
    if ( _viewer.valid() )
    {
        _viewer->stopThreading();
        _viewer = 0L;
    }

    OE_DEBUG << "ViewerWidget::DTOR" << std::endl;
}


void
ViewerWidget::setTimerInterval(int milliseconds)
{
    if ( _timer.interval() != milliseconds )
    {
        _timer.start( milliseconds );
    }
}


void ViewerWidget::installFrameTimer()
{    
    // start the frame timer.
    connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
    _timer.start(20);
}


void ViewerWidget::createViewer()
{
    // creates a simple basic viewer.
    osgViewer::Viewer* viewer = new osgViewer::Viewer();

    viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer->setCameraManipulator(new osgEarth::Util::EarthManipulator());

    viewer->addEventHandler(new osgViewer::StatsHandler());
    viewer->addEventHandler(new osgGA::StateSetManipulator());
    viewer->addEventHandler(new osgViewer::ThreadingHandler());

    viewer->setKeyEventSetsDone(0);
    viewer->setQuitEventSetsDone(false);

    reconfigure( viewer );

    _viewer = viewer;
}

void ViewerWidget::reconfigure( osgViewer::View* view )
{
    if ( !_gc.valid() )
    {
        // create the Qt graphics context if necessary; it will be shared across all views.
        osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds);

        traits->readDISPLAY();
        if (traits->displayNum<0) traits->displayNum = 0;

        traits->windowName = "osgEarthViewerQt";
        traits->windowDecoration = false;
        traits->x = x();
        traits->y = y();
        traits->width = width();
        traits->height = height();
        traits->doubleBuffer = true;
        traits->inheritedWindowData = new osgQt::GraphicsWindowQt::WindowData(this);

        _gc = new osgQt::GraphicsWindowQt( traits.get() );

        // Update Qt OpenGL context to use the version of OpenGL requested 
        // via the Qt graphics context setup.
        unsigned int major, minor;
        traits->getContextVersion(major, minor);
        QGLFormat newFormat(format());
        newFormat.setVersion(major, minor);
        setFormat(newFormat);
        makeCurrent();
    }

    // reconfigure this view's camera to use the Qt GC if necessary.
    osg::Camera* camera = view->getCamera();
    if ( camera->getGraphicsContext() != _gc.get() )
    {
        camera->setGraphicsContext( _gc.get() );
        if ( !camera->getViewport() )
        {
            camera->setViewport(new osg::Viewport(0, 0, _gc->getTraits()->width, _gc->getTraits()->height));
        }
        camera->setProjectionMatrixAsPerspective(
            30.0f, camera->getViewport()->width()/camera->getViewport()->height(), 1.0f, 10000.0f );
    }
    
    camera->setDrawBuffer(_gc->getTraits()->doubleBuffer? GL_BACK : GL_FRONT);
    camera->setReadBuffer(_gc->getTraits()->doubleBuffer? GL_BACK : GL_FRONT);
}

      
void ViewerWidget::paintEvent(QPaintEvent* e)
{
    if (_viewer->getRunFrameScheme() == osgViewer::ViewerBase::CONTINUOUS || 
        _viewer->checkNeedToDoFrame() )
    {
        _viewer->frame();
    }
}
