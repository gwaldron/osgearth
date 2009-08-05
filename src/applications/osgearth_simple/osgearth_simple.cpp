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
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    viewer.setCameraManipulator( new osgEarthUtil::EarthManipulator() );

    // The "Map" is the data model object that we will be visualizing. It will be
    // geocentric by default, but you can specify a projected map in the constructor.
    osgEarth::Map* map = new osgEarth::Map();

    // Add an image layer to the map.
    {
        osgEarth::Properties conf;
        conf["url"] = "http://demo.pelicanmapping.com/rmweb/data/bluemarble-tms/tms.xml";
        osgEarth::MapLayer* layer = new osgEarth::MapLayer( "NASA", osgEarth::MapLayer::TYPE_IMAGE, "tms", conf );
        map->addMapLayer( layer );
    }

    // Add a heightfield layer to the map. You can add any number of heightfields and
    // osgEarth will composite them automatically.
    {
        osgEarth::Properties conf;
        conf["url"] = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
        osgEarth::MapLayer* layer = new osgEarth::MapLayer( "SRTM", osgEarth::MapLayer::TYPE_HEIGHTFIELD, "tms", conf );
        map->addMapLayer( layer );
    }

    // The MapNode will render the Map object in the scene graph.
    osgEarth::MapNode* mapNode = new osgEarth::MapNode( map );

    viewer.setSceneData( mapNode );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
