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
#include <osgEarthQt/CompositeViewerWidget>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>

#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/EarthManipulator>

#include <osgGA/StateSetManipulator>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/ViewerBase>
#include <osgViewer/ViewerEventHandlers>

#include <QtCore/QTimer>
#include <QtGui/QGridLayout>
#include <QtGui/QWidget>

using namespace osgEarth;
using namespace osgEarth::QtGui;


CompositeViewerWidget::CompositeViewerWidget(osg::Node* scene, DataManager* manager)
: _manager(manager)
{
  initialize();

  connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
  _timer.start(15);
}

void CompositeViewerWidget::initialize()
{
  setThreadingModel(osgViewer::Viewer::SingleThreaded);
}

osgViewer::View* CompositeViewerWidget::createViewWidget(osg::Node* scene, osgViewer::View* shared)
{
  osgViewer::View* view = new osgViewer::View();
  view->setCamera(createCamera(0, 0, 100, 100, (shared ? shared->getCamera()->getGraphicsContext() : 0L)));
  view->setCameraManipulator(new osgEarth::Util::EarthManipulator());
  
  if (_manager.valid())
    view->getCamera()->addCullCallback(new osgEarth::Util::AutoClipPlaneCullCallback(_manager->map()));

  view->addEventHandler(new osgViewer::StatsHandler());
  view->addEventHandler(new osgGA::StateSetManipulator());
  view->addEventHandler(new osgViewer::ThreadingHandler());

  if (scene)
    view->setSceneData(scene);

  addView(view);

  layoutWidgets();

  return view;
}

osg::Camera* CompositeViewerWidget::createCamera(int x, int y, int width, int height, osg::GraphicsContext* shared)
{
  osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
  osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds);

  traits->readDISPLAY();
  if (traits->displayNum<0) traits->displayNum = 0;

  traits->windowName = "";
  traits->windowDecoration = false;
  traits->x = x;
  traits->y = y;
  traits->width = width;
  traits->height = height;
  traits->doubleBuffer = true;
  traits->sharedContext = shared;
  //traits->inheritedWindowData = inherited;
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

void CompositeViewerWidget::layoutWidgets()
{
  QGridLayout* grid = new QGridLayout;

  osgViewer::ViewerBase::Windows windows;
  getWindows(windows);

  int viewCount = 0;
  for (osgViewer::ViewerBase::Windows::iterator it = windows.begin(); it != windows.end(); ++it)
  {
    osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>(*it);
    if (gw)
      viewCount++;
  }
  
  int cols = ceil(sqrt((float)viewCount));
  int col = 0, row = 0;

  for (osgViewer::ViewerBase::Windows::iterator it = windows.begin(); it != windows.end(); ++it)
  {
    osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>(*it);
    if (gw)
    {
      grid->addWidget(gw->getGLWidget(), row, col, 1, row * cols + col + 1 == viewCount ? cols - col : 1);
      col++;

      if (col == cols)
      {
        row++;
        col = 0;
      }
    }
  }

  delete layout();
  setLayout( grid );
}