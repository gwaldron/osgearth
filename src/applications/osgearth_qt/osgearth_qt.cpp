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
#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthQt/ViewerWidget>
#include <osgEarthQt/CompositeViewerWidget>
#include <osgEarthQt/MapCatalogWidget>
#include <osgEarthQt/DataManager>

#include <QAction>
#include <QDockWidget>
#include <QMainWindow>
#include <QToolBar>
#include <QtGui/QApplication>

#include "DemoMainWindow"


using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static osg::ref_ptr<osg::Group> s_annoGroup;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << std::endl;
    OE_NOTICE << "USAGE: osgearth_qt [options] file.earth" << std::endl;
    OE_NOTICE << "   --composite n   : use a composite viewer with n initial views" << std::endl;
        
    return -1;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    std::string compNum;
    bool composite = arguments.read( "--composite", compNum );
    int numViews = composite ? osgEarth::as<int>(compNum, 4) : 1;


    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );
 

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    s_annoGroup = new osg::Group();
    root->addChild( s_annoGroup );

    QApplication app(argc, argv);

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    osg::ref_ptr<osgEarth::QtGui::DataManager> dataManager = new osgEarth::QtGui::DataManager(mapNode);
    DemoMainWindow appWin(dataManager.get(), mapNode, s_annoGroup);


    // add an annotation for demo purposes
    osgEarth::Annotation::AnnotationNode* annotation = new osgEarth::Annotation::PlaceNode(
      mapNode, 
      osg::Vec3d(-77.04, 38.85, 0),
      osgDB::readImageFile("../data/placemark32.png"),
      "Washington, DC");

    osgEarth::Annotation::AnnotationData* data = new osgEarth::Annotation::AnnotationData();
    data->setName("Washington, DC");
    data->setViewpoint(osgEarth::Viewpoint(osg::Vec3d(-77.04, 38.85, 0), 0.0, -90.0, 1e5));
    annotation->setAnnotationData(data);

    dataManager->addAnnotation(annotation, s_annoGroup);
    

    // create viewer widget
    if (composite)
    {
      osgEarth::QtGui::CompositeViewerWidget* viewer = new osgEarth::QtGui::CompositeViewerWidget(root, dataManager.get());

      osgViewer::View* primary = viewer->createViewWidget(root);

      for (int i=0; i < numViews - 1; i++)
        osgViewer::View* childView = viewer->createViewWidget(root, primary);

      viewer->setGeometry(100, 100, 800, 600);
      appWin.setViewerWidget(viewer);
    }
    else
    {
      osgEarth::QtGui::ViewerWidget* viewer = new osgEarth::QtGui::ViewerWidget(root, dataManager.get());
      viewer->setGeometry(100, 100, 800, 600);
      appWin.setViewerWidget(viewer);
    }


    // create catalog widget and add as a docked widget to the main window
    QDockWidget *catalogDock = new QDockWidget(QWidget::tr("Catalog"));
    catalogDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::MapCatalogWidget* layerCatalog = new osgEarth::QtGui::MapCatalogWidget(dataManager.get(), osgEarth::QtGui::MapCatalogWidget::LAYERS_AND_ANNOTATIONS);
	  catalogDock->setWidget(layerCatalog);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, catalogDock);


    // create a second catalog widget for viewpoints
    QDockWidget *vpDock = new QDockWidget(QWidget::tr("Viewpoints"));
    vpDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    osgEarth::QtGui::MapCatalogWidget* vpCatalog = new osgEarth::QtGui::MapCatalogWidget(dataManager.get(), osgEarth::QtGui::MapCatalogWidget::VIEWPOINTS);
	  vpDock->setWidget(vpCatalog);
	  appWin.addDockWidget(Qt::LeftDockWidgetArea, vpDock);


    appWin.setGeometry(100, 100, 800, 600);
    appWin.show();

    return app.exec();
}
