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

#include <osg/Notify>
#include <osgEarth/MapNode>
#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/ScaleDecoration>
#include <osgEarthQt/ViewerWidget>
#include <osgEarthQt/CompositeViewerWidget>
#include <osgEarthQt/LayerManagerWidget>
#include <osgEarthQt/MapCatalogWidget>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/AnnotationListWidget>
#include <osgEarthUtil/AnnotationEvents>
#include <osgEarthUtil/SkyNode>
#include <osgEarthDrivers/ocean_surface/OceanSurface>

#include <QAction>
#include <QDockWidget>
#include <QMainWindow>
#include <QToolBar>
#include <QtGui/QApplication>

#include "DemoMainWindow"

#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif


using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static osg::ref_ptr<osg::Group> s_annoGroup;
static osgEarth::Util::SkyNode* s_sky=0L;
static osgEarth::Drivers::OceanSurfaceNode* s_ocean=0L;


int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << std::endl;
    OE_NOTICE << "USAGE: osgearth_qt [options] file.earth" << std::endl;
    OE_NOTICE << "   --composite n           : use a composite viewer with n initial views" << std::endl;
    OE_NOTICE << "   --stylesheet filename   : optional Qt stylesheet" << std::endl;
        
    return -1;
}

//------------------------------------------------------------------

/**
 * Event handler that processes events fired from the
 * AnnotationEventCallback
 */
struct MyAnnoEventHandler : public AnnotationEventHandler
{
  MyAnnoEventHandler(osgEarth::QtGui::DataManager* manager) : _manager(manager) {}

  void onClick( AnnotationNode* node, const EventArgs& details )
  {
    if (_manager.valid() && details.buttons == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
      if (details.modkeys & osgGA::GUIEventAdapter::MODKEY_CTRL)
      {
        if (_manager->isSelected(node))
          _manager->removeSelectedAnnotation(node);
        else
          _manager->addSelectedAnnotation(node);
      }
      else
      {
        _manager->clearSelectedAnnotations();
        _manager->addSelectedAnnotation(node);
      }
    }   
  }

  //void onHoverEnter( AnnotationNode* anno, const EventArgs& args )
  //{
  //  anno->setDecoration( "hover" );
  //}

  //void onHoverLeave( AnnotationNode* anno, const EventArgs& args )
  //{
  //  anno->clearDecoration();
  //}

  osg::ref_ptr<osgEarth::QtGui::DataManager> _manager;
};

//------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);

    std::string compNum;
    bool composite = arguments.read("--composite", compNum);
    int numViews = composite ? osgEarth::as<int>(compNum, 4) : 1;

    std::string stylesheet;
    bool styled = arguments.read("--stylesheet", stylesheet);

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );
 

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    s_annoGroup = new osg::Group();
    root->addChild( s_annoGroup );

  #ifdef Q_WS_X11
    XInitThreads();
  #endif

    QApplication app(argc, argv);

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    osg::ref_ptr<osgEarth::QtGui::DataManager> dataManager = new osgEarth::QtGui::DataManager(mapNode);
    DemoMainWindow appWin(dataManager.get(), mapNode, s_annoGroup);

    // install an event handler for picking and selection
    AnnotationEventCallback* cb = new AnnotationEventCallback();
    cb->addHandler(new MyAnnoEventHandler(dataManager.get()));
    s_annoGroup->addEventCallback(cb);

    // add an annotation for demo purposes
    if (mapNode)
    {
      osgEarth::Annotation::AnnotationNode* annotation = new osgEarth::Annotation::PlaceNode(
        mapNode, 
        osg::Vec3d(-77.04, 38.85, 0),
        osgDB::readImageFile("../data/placemark32.png"),
        "Washington, DC");

      osgEarth::Annotation::AnnotationData* data = new osgEarth::Annotation::AnnotationData();
      data->setName("Washington, DC");
      data->setViewpoint(osgEarth::Viewpoint(osg::Vec3d(-77.04, 38.85, 0), 0.0, -90.0, 1e5));
      annotation->setAnnotationData(data);

      annotation->installDecoration("selected", new osgEarth::Annotation::ScaleDecoration(2.0f));

      dataManager->addAnnotation(annotation, s_annoGroup);
    }
    
    osgEarth::QtGui::ViewVector views;

    // create viewer widget
    if (composite)
    {
      osgEarth::QtGui::CompositeViewerWidget* viewer = new osgEarth::QtGui::CompositeViewerWidget(root, dataManager.get());

      osgViewer::View* primary = viewer->createViewWidget(root);
      views.push_back(primary);

      for (int i=0; i < numViews - 1; i++)
        views.push_back(viewer->createViewWidget(root, primary));

      viewer->setGeometry(100, 100, 800, 600);
      appWin.setViewerWidget(viewer, views);
    }
    else
    {
      osgEarth::QtGui::ViewerWidget* viewer = new osgEarth::QtGui::ViewerWidget(root, dataManager.get());
      viewer->setGeometry(100, 100, 800, 600);
      appWin.setViewerWidget(viewer);
      views.push_back(viewer);

      if (mapNode)
      {
        const Config& externals = mapNode->externalConfig();

        if (mapNode->getMap()->isGeocentric())
        {
          // Sky model.
          Config skyConf = externals.child("sky");

          double hours = skyConf.value("hours", 12.0);
          s_sky = new osgEarth::Util::SkyNode(mapNode->getMap());
          s_sky->setDateTime(2011, 3, 6, hours);
          s_sky->attach(viewer);
          root->addChild(s_sky);

          // Ocean surface.
          if (externals.hasChild("ocean"))
          {
            s_ocean = new osgEarth::Drivers::OceanSurfaceNode(mapNode, externals.child("ocean"));
            if (s_ocean)
              root->addChild(s_ocean);
          }
        }
      }
        
    }


    // create catalog widget and add as a docked widget to the main window
    QDockWidget *catalogDock = new QDockWidget(QWidget::tr("Layers"));
    catalogDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::MapCatalogWidget* layerCatalog = new osgEarth::QtGui::MapCatalogWidget(dataManager.get(), osgEarth::QtGui::MapCatalogWidget::ALL_LAYERS);
    layerCatalog->setActiveViews(views);
    layerCatalog->setHideEmptyGroups(true);
	  catalogDock->setWidget(layerCatalog);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, catalogDock);


    // create and dock an annotation list widget
    QDockWidget *annoDock = new QDockWidget(QWidget::tr("Annotations"));
    annoDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::AnnotationListWidget* annoList = new osgEarth::QtGui::AnnotationListWidget(dataManager.get());
    annoList->setActiveViews(views);
    annoDock->setWidget(annoList);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, annoDock);


    // create a second catalog widget for viewpoints
    QDockWidget *vpDock = new QDockWidget(QWidget::tr("Viewpoints"));
    vpDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::MapCatalogWidget* vpCatalog = new osgEarth::QtGui::MapCatalogWidget(dataManager.get(), osgEarth::QtGui::MapCatalogWidget::VIEWPOINTS);
    vpCatalog->setActiveViews(views);
	  vpDock->setWidget(vpCatalog);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, vpDock);


    // create layer manager widget and add as a docked widget on the right
    QDockWidget *layersDock = new QDockWidget(QWidget::tr("Image Layers"));
    layersDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::LayerManagerWidget* layerManager = new osgEarth::QtGui::LayerManagerWidget(dataManager.get(), osgEarth::QtGui::LayerManagerWidget::IMAGE_LAYERS);
    layerManager->setActiveViews(views);
	  layersDock->setWidget(layerManager);
	  appWin.addDockWidget(Qt::RightDockWidgetArea, layersDock);

    // attempt to load .qss stylesheet if one was provided
    if (styled)
    {
      QFile file(QString(stylesheet.c_str()));
      if (file.exists())
      {
        file.open(QFile::ReadOnly);
        QString qstylesheet = QLatin1String(file.readAll());
        app.setStyleSheet(qstylesheet);
        layerManager->setStyleSheet(qstylesheet);
        annoList->setStyleSheet(qstylesheet);
      }
    }

    appWin.setGeometry(100, 100, 800, 600);
    appWin.show();

    return app.exec();
}
