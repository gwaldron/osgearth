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
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthDrivers/kml/KML>

using namespace osgEarth::Util;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << std::endl;
    OE_NOTICE << "USAGE: osgearth_kml file.earth file.kml" << std::endl;
    return -1;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();

    // load the .earth file from the command line.
    MapNode* mapNode = MapNode::load( arguments );
    if (!mapNode)
        return usage( "Unable to load earth model." );

    root->addChild( mapNode );

    for( int a = 1; a < argc; ++a )
    {
        std::string kmlFile( argv[a] );
        if ( endsWith( kmlFile, ".kml" ) )
        {
            osg::ref_ptr<osgDB::Options> options = new osgDB::Options();
            options->setPluginData( "osgEarth::MapNode", mapNode );
            osg::Node* kml = osgDB::readNodeFile( kmlFile, options.get() );
            if ( kml )
                root->addChild( kml );
        }
    }

    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.setSceneData( root );
    viewer.getDatabasePager()->setDoPreCompile( true );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}
