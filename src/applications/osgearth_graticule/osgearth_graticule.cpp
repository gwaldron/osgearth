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
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/MGRSFormatter>
#include <osgEarthUtil/LatLongFormatter>

#include <osgEarthUtil/GeodeticGraticule>
#include <osgEarthUtil/MGRSGraticule>
#include <osgEarthUtil/UTMGraticule>

using namespace osgEarth::Util;

int
usage( const std::string& msg )
{
    OE_NOTICE 
        << msg << std::endl
        << "USAGE: osgearth_graticule [options] file.earth" << std::endl
        << "   --geodetic            : display a geodetic (lat/long) graticule" << std::endl
        << "   --utm                 : display a UTM graticule" << std::endl
        << "   --mgrs                : display an MGRS graticule" << std::endl;        
    return -1;
}

//------------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // parse command line:
    bool isUTM = arguments.read("--utm");
    bool isMGRS = arguments.read("--mgrs");
    bool isGeodetic = !isUTM && !isMGRS;

    // load the .earth file from the command line.
    MapNode* mapNode = MapNode::load( arguments );
    if ( !mapNode )
        return usage( "Failed to load a map from the .earth file" );

    // install our manipulator:
    viewer.setCameraManipulator( new EarthManipulator() );

    // root scene graph:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode );

    Formatter* formatter = 0L;
    if ( isUTM )
    {
        UTMGraticule* gr = new UTMGraticule( mapNode );
        root->addChild( gr );
        formatter = new MGRSFormatter();
    }
    else if ( isMGRS )
    {
        MGRSGraticule* gr = new MGRSGraticule( mapNode );
        root->addChild( gr );
        formatter = new MGRSFormatter();
    }
    else // if ( isGeodetic )
    {
        GeodeticGraticule* gr = new GeodeticGraticule( mapNode );
        GeodeticGraticuleOptions o = gr->getOptions();
        o.lineStyle()->getOrCreate<LineSymbol>()->stroke()->color().set(1,0,0,1);
        gr->setOptions( o );
        root->addChild( gr );
        formatter = new LatLongFormatter();
    }

    // mouse coordinate readout:
    LabelControl* readout = new LabelControl();
    ControlCanvas::get( &viewer, true )->addControl( readout );
    MouseCoordsTool* tool = new MouseCoordsTool( mapNode );
    tool->addCallback( new MouseCoordsLabelCallback(readout, formatter) );
    viewer.addEventHandler( tool );

    // finalize setup and run.
    viewer.setSceneData( root );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    return viewer.run();
}
