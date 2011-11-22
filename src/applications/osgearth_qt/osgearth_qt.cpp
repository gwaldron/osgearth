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
//#include <osgGA/StateSetManipulator>
//#include <osgGA/GUIEventHandler>
//#include <osgViewer/Viewer>
//#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
//#include <osgEarth/XmlUtils>
//#include <osgEarthUtil/EarthManipulator>
//#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
//#include <osgEarthUtil/Graticule>
#include <osgEarthUtil/SkyNode>
//#include <osgEarthUtil/Viewpoint>
#include <osgEarthUtil/Formatters>
//#include <osgEarthUtil/Annotation>
//#include <osgEarthSymbology/Color>
#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthQt/ViewerWidget>
#include <osgEarthQt/CompositeViewerWidget>
#include <osgEarthQt/MapCatalogWidget>
#include <osgEarthQt/DataManager>

#include <QMainWindow>
#include <QDockWidget>
#include <QtGui/QApplication>


using namespace osgEarth::Util;
//using namespace osgEarth::Util::Annotation;
using namespace osgEarth::Util::Controls;
//using namespace osgEarth::Symbology;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << std::endl;
    OE_NOTICE << "USAGE: osgearth_qt [options] file.earth" << std::endl;
    OE_NOTICE << "   --composite     : use a composite viewer" << std::endl;
    OE_NOTICE << "   --sky           : activates the atmospheric model" << std::endl;
    //OE_NOTICE << "   --autoclip      : activates the auto clip-plane handler" << std::endl;
    OE_NOTICE << "   --dms           : format coordinates as degrees/minutes/seconds" << std::endl;
    OE_NOTICE << "   --mgrs          : format coordinates as MGRS" << std::endl;
    
        
    return -1;
}

static SkyNode*          s_sky           =0L;
static bool              s_dms           =false;
static bool              s_mgrs          =false;


struct MouseCoordsHandler : public osgGA::GUIEventHandler
{
    MouseCoordsHandler( LabelControl* label, osgEarth::MapNode* mapNode )
        : _label( label ),
          _mapNode( mapNode )
    {
        _mapNodePath.push_back( mapNode->getTerrainEngine() );
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
        if (ea.getEventType() == ea.MOVE || ea.getEventType() == ea.DRAG)
        {
            osgUtil::LineSegmentIntersector::Intersections results;
            if ( view->computeIntersections( ea.getX(), ea.getY(), _mapNodePath, results ) )
            {
                // find the first hit under the mouse:
                osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
                osg::Vec3d point = first.getWorldIntersectPoint();
                osg::Vec3d lla;

                // transform it to map coordinates:
                _mapNode->getMap()->worldPointToMapPoint(point, lla);

                std::stringstream ss;

                if ( s_mgrs )
                {
                    MGRSFormatter f( MGRSFormatter::PRECISION_1M );
                    ss << "MGRS: " << f.format(lla.y(), lla.x()) << "   ";
                }
                 // lat/long
                {
                    LatLongFormatter::AngularFormat fFormat = s_dms?
                        LatLongFormatter::FORMAT_DEGREES_MINUTES_SECONDS :
                        LatLongFormatter::FORMAT_DECIMAL_DEGREES;
                    
                    LatLongFormatter f( fFormat );

                    ss 
                        << "Lat: " << f.format( Angular(lla.y(),Units::DEGREES), 4 ) << "  "
                        << "Lon: " << f.format( Angular(lla.x(),Units::DEGREES), 5 );
                }

                _label->setText( ss.str() );
            }
            else
            {
                //Clear the text
                _label->setText( "" );
            }
        }
        return false;
    }

    osg::ref_ptr< LabelControl > _label;
    MapNode*                     _mapNode;
    osg::NodePath                _mapNodePath;
};

void addMouseCoords(osgViewer::View* view, osgEarth::MapNode* mapNode)
{
    if (!view || !mapNode)
      return;

    ControlCanvas* canvas = ControlCanvas::get( view );
    LabelControl* mouseCoords = new LabelControl();
    mouseCoords->setHorizAlign(Control::ALIGN_CENTER );
    mouseCoords->setVertAlign(Control::ALIGN_BOTTOM );
    mouseCoords->setBackColor(0,0,0,0.5);    
    mouseCoords->setSize(400,50);
    mouseCoords->setMargin( 10 );
    canvas->addControl( mouseCoords );

    view->addEventHandler( new MouseCoordsHandler(mouseCoords, mapNode ) );
}

struct TestHandler : public osgGA::GUIEventHandler
{
  TestHandler(osgEarth::Map* map) : _map(map) {}

  bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
  {
    if ( ea.getEventType() == ea.KEYDOWN )
    {
      int index = (int)ea.getKey() - (int)'0';
      if (index == 1)
      {
        osgEarth::Drivers::GDALOptions layerOpt;
        layerOpt.url() = "nyc-inset-wgs84.tif";
        _map->addImageLayer( new ImageLayer( ImageLayerOptions("ny_inset", layerOpt) ) );
      }
      //else if (index == 2)
      //{
      //}
      //else if ( ea.getKey() == 'v' )
      //{
      //    Viewpoint vp = s_manip->getViewpoint();
      //    XmlDocument xml( vp.getConfig() );
      //    xml.store( std::cout );
      //    std::cout << std::endl;
      //}
      //else if ( ea.getKey() == '?' )
      //{
      //    s_controlPanel->setVisible( !s_controlPanel->visible() );
      //}
    }
    return false;
  }

  osg::ref_ptr<osgEarth::Map> _map;
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
    //osgViewer::Viewer viewer(arguments);

    std::string compNum;
    bool composite = arguments.read( "--composite", compNum );
    int numViews = composite ? osgEarth::as<int>(compNum, 4) : 1;

    //bool useAutoClip  = arguments.read( "--autoclip" );
    //bool useSky       = arguments.read( "--sky" );
    s_dms             = arguments.read( "--dms" );
    s_mgrs            = arguments.read( "--mgrs" );

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );
 
    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );

    QApplication app(argc, argv);

    QMainWindow appWin;
    appWin.setWindowTitle(QWidget::tr("osgEarth Qt"));

    osg::ref_ptr<TestHandler> testHandler = new TestHandler(mapNode ? mapNode->getMap() : 0L);

    if (composite)
    {
      osgEarth::QtGui::CompositeViewerWidget* viewer = new osgEarth::QtGui::CompositeViewerWidget(root, mapNode ? mapNode->getMap() : 0L);

      osgViewer::View* primary = viewer->createViewWidget(root);
      primary->addEventHandler(testHandler.get());

      for (int i=0; i < numViews - 1; i++)
      {
        osgViewer::View* childView = viewer->createViewWidget(root, primary);
        childView->addEventHandler(testHandler.get());
      }

      viewer->setGeometry(100, 100, 800, 600);
      appWin.setCentralWidget(viewer);
    }
    else
    {
      osgEarth::QtGui::ViewerWidget* viewer = new osgEarth::QtGui::ViewerWidget(root, mapNode ? mapNode->getMap() : 0L);
      addMouseCoords( viewer, mapNode );

      viewer->addEventHandler(testHandler.get());

      viewer->setGeometry(100, 100, 800, 600);
      appWin.setCentralWidget(viewer);
    }

    QDockWidget *catalogDock = new QDockWidget(QWidget::tr("Catalog"));
    //catalogDock->setObjectName(tr("CATALOG_DOCK_WINDOW"));
    catalogDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::MapCatalogWidget* layerCatalog = new osgEarth::QtGui::MapCatalogWidget(new osgEarth::QtGui::DataManager(mapNode->getMap()), osgEarth::QtGui::MapCatalogWidget::LAYERS_AND_ANNOTATIONS | osgEarth::QtGui::MapCatalogWidget::MASK_LAYERS);
	  catalogDock->setWidget(layerCatalog);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, catalogDock);

    appWin.setGeometry(100, 100, 800, 600);

    appWin.show();

    return app.exec();
}
