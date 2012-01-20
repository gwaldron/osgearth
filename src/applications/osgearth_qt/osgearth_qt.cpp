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
#include <osgEarth/ImageUtils>
#include <osgEarth/MapNode>
#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/ScaleDecoration>
#include <osgEarthAnnotation/TrackNode>
#include <osgEarthQt/ViewerWidget>
#include <osgEarthQt/CompositeViewerWidget>
#include <osgEarthQt/LayerManagerWidget>
#include <osgEarthQt/MapCatalogWidget>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/AnnotationListWidget>
#include <osgEarthQt/LOSControlWidget>
#include <osgEarthUtil/AnnotationEvents>
#include <osgEarthUtil/AutoClipPlaneHandler>
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

#define TRACK_ICON_URL    "../data/m2525_air.png"
#define TRACK_ICON_SIZE   24
#define TRACK_FIELD_NAME  "name"

static osg::ref_ptr<osg::Group> s_annoGroup;
static osgEarth::Util::SkyNode* s_sky=0L;
static osgEarth::Drivers::OceanSurfaceNode* s_ocean=0L;

//------------------------------------------------------------------

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
// Methods for demo track simulation

struct TrackSim : public osg::Referenced
{
  TrackSim(TrackNode* track, const osg::Vec3d center, float radius, double time, osgEarth::MapNode* mapNode)
    : _track(track), _mapNode(mapNode), _radius(radius), _time(time)
  {
    //Get the center point in geocentric
    mapNode->getMap()->mapPointToWorldPoint( center, _center );

    _up = _center;
    _up.normalize();

    //Get the "side" vector
    _side = _up ^ osg::Vec3d(0,0,1);
  }

  void update(double time)
  {
    double angle = (time / _time);
    angle = (angle - (int)angle) * osg::PI * 2.0;

    osg::Quat quat(angle, _up );
    osg::Vec3d spoke = quat * (_side * _radius);
    osg::Vec3d end = _center + spoke;

    osg::Vec3d pos;
    _mapNode->getMap()->worldPointToMapPoint(end, pos);

    _track->setPosition(pos);
  }

  TrackNode* _track;
  osgEarth::MapNode* _mapNode;
  osg::Vec3d _center, _side, _up;
  float _radius;
  double _time;
};
typedef std::vector< osg::ref_ptr<TrackSim> > TrackSimVector;

/** Update operation that runs the simulators. */
struct TrackSimUpdate : public osg::Operation
{
  TrackSimUpdate(TrackSimVector& sims) : osg::Operation("tracksim", true), _sims(sims) { }

  void operator()(osg::Object* obj)
  {
    osg::View* view = dynamic_cast<osg::View*>(obj);
    double t = view->getFrameStamp()->getSimulationTime();
    for(TrackSimVector::iterator i = _sims.begin(); i != _sims.end(); ++i)
      i->get()->update(t);
  }

  TrackSimVector& _sims;
};

TrackNode* createTrack(TrackNodeFieldSchema& schema, osg::Image* image, const std::string& name, MapNode* mapNode, const osg::Vec3d& center, double radius, double time, TrackSimVector& trackSims)
{
  TrackNode* track = new TrackNode(mapNode, center, image, schema);
  track->setFieldValue(TRACK_FIELD_NAME, name);

  AnnotationData* data = new AnnotationData();
  data->setName(name);
  data->setViewpoint(osgEarth::Viewpoint(center, 0.0, -90.0, 1e5));
  track->setAnnotationData( data );

  trackSims.push_back(new TrackSim(track, center, radius, time, mapNode));

  return track;
}

void createTrackSchema(TrackNodeFieldSchema& schema)
{
    // draw the track name above the icon:
    TextSymbol* nameSymbol = new TextSymbol();
    nameSymbol->pixelOffset()->set( 0, 2+TRACK_ICON_SIZE/2 );
    nameSymbol->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
    nameSymbol->halo()->color() = Color::Black;
    schema[TRACK_FIELD_NAME] = TrackNodeField(nameSymbol, false);
}

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

    osg::ref_ptr<osgEarth::MapNode> mapNode = osgEarth::MapNode::findMapNode( earthNode );
    osg::ref_ptr<osgEarth::QtGui::DataManager> dataManager = new osgEarth::QtGui::DataManager(mapNode.get());
    DemoMainWindow appWin(dataManager.get(), mapNode.get(), s_annoGroup);

    // install an event handler for picking and selection
    AnnotationEventCallback* cb = new AnnotationEventCallback();
    cb->addHandler(new MyAnnoEventHandler(dataManager.get()));
    s_annoGroup->addEventCallback(cb);
    
    osgEarth::QtGui::ViewVector views;
    osgViewer::ViewerBase* viewer;

    // create viewer widget
    if (composite)
    {
      osgEarth::QtGui::CompositeViewerWidget* viewerWidget = new osgEarth::QtGui::CompositeViewerWidget(root);

      osgViewer::View* primary = viewerWidget->createViewWidget(root);
      primary->getCamera()->addCullCallback(new osgEarth::Util::AutoClipPlaneCullCallback(mapNode->getMap()));
      views.push_back(primary);

      for (int i=0; i < numViews - 1; i++)
      {
        osgViewer::View* view = viewerWidget->createViewWidget(root, primary);
        view->getCamera()->addCullCallback(new osgEarth::Util::AutoClipPlaneCullCallback(mapNode->getMap()));
        views.push_back(view);
      }

      viewerWidget->setGeometry(100, 100, 800, 600);
      appWin.setViewerWidget(viewerWidget, views);

      viewer = viewerWidget;
    }
    else
    {
      osgEarth::QtGui::ViewerWidget* viewerWidget = new osgEarth::QtGui::ViewerWidget(root);
      viewerWidget->setGeometry(100, 100, 800, 600);
      viewerWidget->getCamera()->addCullCallback(new osgEarth::Util::AutoClipPlaneCullCallback(mapNode->getMap()));
      appWin.setViewerWidget(viewerWidget);
      views.push_back(viewerWidget);

      if (mapNode.valid())
      {
        const Config& externals = mapNode->externalConfig();

        if (mapNode->getMap()->isGeocentric())
        {
          // Sky model.
          Config skyConf = externals.child("sky");

          double hours = skyConf.value("hours", 12.0);
          s_sky = new osgEarth::Util::SkyNode(mapNode->getMap());
          s_sky->setDateTime(2011, 3, 6, hours);
          s_sky->attach(viewerWidget);
          root->addChild(s_sky);

          // Ocean surface.
          if (externals.hasChild("ocean"))
          {
            s_ocean = new osgEarth::Drivers::OceanSurfaceNode(mapNode.get(), externals.child("ocean"));
            if (s_ocean)
              root->addChild(s_ocean);
          }
        }
      }

      viewer = viewerWidget;
    }


    // create demo tracks
    TrackSimVector trackSims;

    osg::ref_ptr<osg::Image> srcImage = osgDB::readImageFile(TRACK_ICON_URL);
    osg::ref_ptr<osg::Image> image;
    ImageUtils::resizeImage(srcImage.get(), TRACK_ICON_SIZE, TRACK_ICON_SIZE, image);

    TrackNodeFieldSchema schema;
    createTrackSchema(schema);
    dataManager->addAnnotation(createTrack(schema, image, "Plane 1", mapNode.get(), osg::Vec3d(-121.463, 46.3548, 1348.71), 10000, 24, trackSims), s_annoGroup);
    dataManager->addAnnotation(createTrack(schema, image, "Plane 2", mapNode.get(), osg::Vec3d(-121.656, 46.0935, 4133.06), 10000, 8, trackSims), s_annoGroup);
    dataManager->addAnnotation(createTrack(schema, image, "Plane 3", mapNode.get(), osg::Vec3d(-121.321, 46.2589, 1390.09), 10000, 12, trackSims), s_annoGroup);

    viewer->addUpdateOperation(new TrackSimUpdate(trackSims));


    // create catalog widget and add as a docked widget to the main window
    QDockWidget *catalogDock = new QDockWidget(QWidget::tr("Layers"));
    catalogDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::MapCatalogWidget* layerCatalog = new osgEarth::QtGui::MapCatalogWidget(dataManager.get(), osgEarth::QtGui::MapCatalogWidget::ALL_LAYERS);
    layerCatalog->setActiveViews(views);
    layerCatalog->setHideEmptyGroups(true);
	  catalogDock->setWidget(layerCatalog);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, catalogDock);


    // create and dock an annotation list widget
    QDockWidget *annoDock = new QDockWidget;
    annoDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::AnnotationListWidget* annoList = new osgEarth::QtGui::AnnotationListWidget(dataManager.get());
    annoList->setActiveViews(views);
    annoDock->setWidget(annoList);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, annoDock);


    // create a second catalog widget for viewpoints
    QDockWidget *vpDock = new QDockWidget;
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


    // create and dock a LOSControlWidget
    QDockWidget *losDock = new QDockWidget;
    losDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::LOSControlWidget* losControl = new osgEarth::QtGui::LOSControlWidget(root, mapNode.get(), dataManager.get());
    losControl->setActiveViews(views);
	  losDock->setWidget(losControl);
	  appWin.addDockWidget(Qt::RightDockWidgetArea, losDock);


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
        losControl->setStyleSheet(qstylesheet);
      }
    }

    appWin.setGeometry(100, 100, 800, 600);
    appWin.show();

    return app.exec();
}
