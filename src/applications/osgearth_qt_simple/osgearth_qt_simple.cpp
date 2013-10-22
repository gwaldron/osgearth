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
#include <osgViewer/CompositeViewer>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthQt/ViewerWidget>
#include <QtGui/QApplication>
#include <QtGui/QMainWindow>
#include <QtGui/QStatusBar>
#include <QtGui/QMdiArea>
#include <QtGui/QMdiSubWindow>

#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::QtGui;

//------------------------------------------------------------------

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl << std::endl;
    OE_NOTICE << "USAGE: osgearth_qt_simple file.earth" << std::endl;
        
    return -1;
}

//------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--stencil") )
        osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);


    osgViewer::Viewer viewer(arguments);
    viewer.setRunFrameScheme( viewer.ON_DEMAND );
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file
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
    win.setCentralWidget( viewerWidget );
    win.setGeometry(100, 100, 1024, 800);

    win.statusBar()->showMessage(QString("Quite possibly the world's simplest osgEarthQt app."));

    win.show();
    app.exec();
}
