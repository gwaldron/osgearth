/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthQt/ViewerWidget>

#include <osgEarthUtil/EarthManipulator>

#include <osgGA/StateSetManipulator>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <QtGui>
#include <QtCore/QTimer>
#include <QtGui/QWidget>

using namespace osgEarth;
using namespace osgEarth::QtGui;


ViewerWidget::ViewerWidget(osg::Node* scene)
{
  initialize();

  if (scene) setSceneData(scene);

  connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
  _timer.start(15);
}

void ViewerWidget::initialize()
{
  setThreadingModel(osgViewer::Viewer::DrawThreadPerContext);
  setCamera(createCamera());
  setCameraManipulator(new osgEarth::Util::EarthManipulator());

  addEventHandler(new osgViewer::StatsHandler());
  addEventHandler(new osgGA::StateSetManipulator());
  addEventHandler(new osgViewer::ThreadingHandler());
}

osg::Camera* ViewerWidget::createCamera()
{
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
  //traits->alpha = ds->getMinimumNumAlphaBits();
  //traits->stencil = ds->getMinimumNumStencilBits();
  //traits->sampleBuffers = ds->getMultiSamples();
  //traits->samples = ds->getNumMultiSamples();

  //if (ds->getStereo())
  //{
  //  switch(ds->getStereoMode())
  //  {
  //  case(osg::DisplaySettings::QUAD_BUFFER): traits->quadBufferStereo = true; break;
  //  case(osg::DisplaySettings::VERTICAL_INTERLACE):
  //  case(osg::DisplaySettings::CHECKERBOARD):
  //  case(osg::DisplaySettings::HORIZONTAL_INTERLACE): traits->stencil = 8; break;
  //  default: break;
  //  }
  //}
  
  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  camera->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

  //camera->setClearColor( osg::Vec4(0.0, 0.0, 0.0, 1.0) );
  camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
  camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );
  
  return camera.release();
}
      
void ViewerWidget::paintEvent(QPaintEvent* e)
{
    if ( getRunFrameScheme() == CONTINUOUS || checkNeedToDoFrame() )
    {
        frame();
    }
}
