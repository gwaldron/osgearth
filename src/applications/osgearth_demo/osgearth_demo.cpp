/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include <osg/DisplaySettings>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthQt/ViewerWidget>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <QtGui/QApplication>
#include <QtGui/QMainWindow>
#include <QtGui/QStatusBar>

#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::QtGui;

//------------------------------------------------------------------

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl << std::endl;
    OE_NOTICE << "USAGE: osgearth_demo file.earth [options]" << std::endl;
    return -1;
}

//------------------------------------------------------------------

int
main(int argc, char** argv)
{
    // allocate pager threads based on CPU configuration.
    int cores = Registry::capabilities().getNumProcessors();
    osg::DisplaySettings::instance()->setNumOfDatabaseThreadsHint( osg::clampAbove(cores, 2) );
    osg::DisplaySettings::instance()->setNumOfHttpDatabaseThreadsHint( osg::clampAbove(cores/2, 1) );

    // parse the command line.
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // set up the motion model.
    viewer.setCameraManipulator( new EarthManipulator() );

    // simple UI.
    VBox* vbox = new VBox();
    vbox->setMargin( 3 );
    vbox->setChildSpacing( 5 );
    vbox->setAbsorbEvents( true );
    vbox->setHorizAlign( Control::ALIGN_RIGHT );
    vbox->setVertAlign ( Control::ALIGN_BOTTOM );

    // Control instructions:
    Grid* grid = vbox->addControl( new Grid() );
    grid->setControl( 0, 0, new LabelControl("Pan:"));
    grid->setControl( 1, 0, new LabelControl("Left mouse"));
    grid->setControl( 0, 1, new LabelControl("Zoom:"));
    grid->setControl( 1, 1, new LabelControl("Right mouse / scroll"));
    grid->setControl( 0, 2, new LabelControl("Rotate:") );
    grid->setControl( 1, 2, new LabelControl("Middle mouse"));
    grid->setControl( 0, 3, new LabelControl("Go to:"));
    grid->setControl( 1, 3, new LabelControl("Double-click"));

    ControlCanvas::get(&viewer)->addControl(vbox);

    // Load an earth file
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( !node )
        return usage( "Failed to load earth file." );
    viewer.setSceneData( node );


#ifdef Q_WS_X11
    // required for multi-threaded viewer on linux:
    XInitThreads();
#endif


    QApplication app(argc, argv);

    QWidget* viewerWidget = new ViewerWidget( &viewer );
    
    QMainWindow win;
    win.setWindowTitle( "osgEarth -- The #1 Open Source Terrain SDK for Mission-Critical Applications" );
    win.setCentralWidget( viewerWidget );
    win.setGeometry(100, 100, 1024, 800);

    win.statusBar()->showMessage(QString("osgEarth.   Terrain on Demand.   Copyright 2013 Pelican Mapping.   Please visit http://osgearth.org"));

    win.show();
    app.exec();
}
