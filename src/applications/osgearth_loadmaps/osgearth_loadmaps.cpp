/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthDrivers/tms/TMSOptions>

using namespace osgEarth;
using namespace osgEarth::Drivers;


osgEarth::MapNode* createSimpleMap()
{
    // Create a "Map" dynamically if no nodes were loaded
    // The "Map" is the data model object that we will be visualizing. It will be
    // geocentric by default, but you can specify a projected map in the constructor.
    osgEarth::Map* map = new osgEarth::Map();

    // Add an image layer to the map.
    {
        TMSOptions* tms = new TMSOptions();
        tms->url() = "http://demo.pelicanmapping.com/rmweb/data/bluemarble-tms/tms.xml";
        map->addMapLayer( new ImageMapLayer( "NASA", tms ) );
    }

    // Add a heightfield layer to the map. You can add any number of heightfields and
    // osgEarth will composite them automatically.
    {
        TMSOptions* tms = new TMSOptions();
        tms->url() = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
        map->addMapLayer( new HeightFieldMapLayer( "SRTM", tms ) );
    }

    // The MapNode will render the Map object in the scene graph.
    osgEarth::MapNode* mapNode = new osgEarth::MapNode( map );
    return mapNode;
}

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    viewer.setCameraManipulator( new osgGA::TrackballManipulator);

    osg::Group* root = new osg::Group;

    root->addChild( createSimpleMap() );

    viewer.setSceneData( root );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    unsigned int frameNum = 0;
    while (!viewer.done())
    {
        if (frameNum % 500 == 0)
        {
            osg::notify(osg::NOTICE) << "Loading new map..." << std::endl;

            //Remove the current map from the scene graph
            root->removeChildren( 0, root->getNumChildren() );
            //Cancel any pending requests from the database pager.
            viewer.getDatabasePager()->clear();
            
            //Create a new map and add it to the scene graph
            osg::Node* newMap = createSimpleMap();
            root->addChild( newMap );

            //Register the new map with the pager since we aren't assigning it via setSceneData.
            viewer.getDatabasePager()->registerPagedLODs( root );
        }
        viewer.frame();
        frameNum++;
    }
    return 0;
}
