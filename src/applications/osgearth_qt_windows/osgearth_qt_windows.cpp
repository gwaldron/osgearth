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


/**
 * This demo show how to create multiple Qt widgets/windows and to share
 * the map amongst them.
 */

#include <osg/Notify>
#include <osgViewer/CompositeViewer>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthQt/ViewWidget>
#include <osgEarth/Random>
#include <QApplication>
#include <QDialog>
#include <QMainWindow>
#include <QPushButton>
#include <QLayout>

#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::QtGui;

//------------------------------------------------------------------

int
usage( const std::string& msg, osg::ArgumentParser& args ) 
{
    OE_NOTICE << msg << std::endl << std::endl;
    OE_NOTICE << "USAGE: " << args[0] << " file.earth" << std::endl;
        
    return -1;
}

//------------------------------------------------------------------

struct ViewController 
{
    virtual void addView() =0;
};


struct MyMainWindow : public QMainWindow, public ViewController
{
    QTimer                     _timer;
    osgViewer::CompositeViewer _viewer;
    osg::ref_ptr<osg::Node>    _scene;

    MyMainWindow(osg::ArgumentParser& args, osg::Node* scene) 
        : _viewer(args), _scene(scene)
    {
        // we have to add a starting view, otherwise the CV will automatically
        // mark itself as done :/
        addView();

        // timer fires a paint event.
        connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
        _timer.start(20);
    }

    void paintEvent(QPaintEvent* e)
    {
        // refresh all the views.
        if (_viewer.getRunFrameScheme() == osgViewer::ViewerBase::CONTINUOUS || 
            _viewer.checkNeedToDoFrame() )
        {
            _viewer.frame();
        }
    }

    void addView()
    {
        // the new View we want to add:
        osgViewer::View* view = new osgViewer::View();

        // a widget to hold our view:
        QWidget* viewWidget = new osgEarth::QtGui::ViewWidget(view);

        // a dialog to hold the view widget:
        QDialog* win = new QDialog(this);
        win->setModal( false );
        win->setLayout( new QHBoxLayout() );
        win->layout()->addWidget( viewWidget );
        int x = osgEarth::Random().next( 1024 );
        int y = osgEarth::Random().next( 768 );
        win->setGeometry( x, y, 640, 480 );
        win->show();

        // set up the view
        view->setCameraManipulator( new osgEarth::Util::EarthManipulator );
        view->setSceneData( _scene.get() );  
        view->getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true,false);

        // add it to the composite viewer.
        _viewer.addView( view );
    }
};

//------------------------------------------------------------------

struct AddButton : public QPushButton
{
    ViewController* _vc;

    AddButton(ViewController* vc, const char* text)
        : QPushButton(text), _vc(vc) { }

    void mousePressEvent( QMouseEvent* e )
    {
        _vc->addView();
    }
};

//------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);
    if ( args.find("--help") >= 0)
        return usage("Help", args);

    // load something
    osg::Node* node = osgDB::readNodeFiles( args );
    if ( !node )
        return usage("Can't load a scene!", args);


#ifdef Q_WS_X11
    // required for multi-threaded viewer on linux:
    XInitThreads();
#endif

    // Qt setup:
    QApplication q(argc, argv);

    // fire up our controller:
    MyMainWindow win( args, node );

    // A button for adding new views:
    QPushButton* addButton = new AddButton( &win, "Add a view" );

    // Main window for the button.
    //QMainWindow win;
    win.setCentralWidget( addButton );
    win.setGeometry( 100, 100, 200, 100 );
    win.show();

    q.exec();
}
