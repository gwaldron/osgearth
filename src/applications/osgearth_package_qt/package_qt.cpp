/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include <osg/Notify>
#include <osgDB/FileUtils>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/ImageUtils>
#include <osgEarth/MapNode>
#include <osgEarthQt/ViewerWidget>
#include <osgEarthQt/LayerManagerWidget>
#include <osgEarthQt/MapCatalogWidget>
#include <osgEarthQt/DataManager>
#include <osgEarthUtil/EarthManipulator>

#include <QApplication>

#include "PackageQtMainWindow"
#include "SceneController.h"
#include "TMSExporter.h"

#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif

using namespace PackageQt;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;


/** Finds an argument with the specified extension. */
std::string
findArgumentWithExtension( osg::ArgumentParser& args, const std::string& ext )
{
    for( int i=0; i<args.argc(); ++i )
    {
        std::string arg( args.argv()[i] );
        if ( endsWith( toLower(trim(arg)), ".earth" ) )
            return arg;
    }
    return "";
}

int main(int argc, char** argv)
{
  HTTPClient::setUserAgent( "osgearth_package_qt/1.0" );

  //setup log file
#ifdef _WIN32
  const char *appData = getenv("APPDATA");
#else
  const char *appData = "/tmp";
#endif

  std::string logDir = std::string(appData) + "/osgEarthPackageQt";
  if (!osgDB::fileExists(logDir))
    osgDB::makeDirectory(logDir);

  /*
  std::string logPath = logDir + "/log.txt";
  std::ofstream* log = new std::ofstream( logPath.c_str() );
  std::cout.rdbuf( log->rdbuf() );
  std::cerr.rdbuf( log->rdbuf() );


  if (getenv("OSGEARTH_PACKAGE_LOGGING") != 0)
  {
    std::string level( getenv("OSGEARTH_PACKAGE_LOGGING") );
    if ( level == "INFO" )
      osgEarth::setNotifyLevel( osg::INFO );
    else if ( level == "DEBUG" )
      osgEarth::setNotifyLevel( osg::DEBUG_INFO );
  }
  else
  {
    osgEarth::setNotifyLevel( osg::INFO );
  }
  */

  osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);

  #ifdef Q_WS_X11
  XInitThreads();
  #endif

  QApplication app(argc, argv);

  osg::ref_ptr<osg::Group> root = new osg::Group();

  //create ViewWidget and get views collection
  osgEarth::QtGui::ViewerWidget* viewerWidget = new osgEarth::QtGui::ViewerWidget( root );
  osgEarth::QtGui::ViewVector views;
  viewerWidget->getViews( views );

  osg::ref_ptr<osgViewer::View> mainView;
  if (views.size() > 0)
    mainView = views[0];

  if (mainView.valid())
  {
    mainView->getCamera()->setNearFarRatio(0.00002);
    mainView->addEventHandler( new osgGA::StateSetManipulator(mainView->getCamera()->getOrCreateStateSet()) );
    mainView->addEventHandler( new osgViewer::StatsHandler() );

    osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(mainView.get());
    if(viewer)
      viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

  }

  //create the SceneController, if no earth file is specified a blank
  //globe will be loaded
  osg::ArgumentParser args(&argc,argv);
  std::string earthFile = findArgumentWithExtension(args, ".earth");
  SceneController controller(root, mainView, earthFile);

  //create the TMSExporter and main window
  TMSExporter exporter;
  PackageQtMainWindow appWin(viewerWidget, &controller, &exporter);
  appWin.setGeometry(100, 100, 1280, 800);
  appWin.show();

  //return app.exec();
  int ret = app.exec();

  //TODO: move somewhere smarter
  /*
  if (log)
  {
      log->close();
      delete log;
  }
  */

  return ret;
}
