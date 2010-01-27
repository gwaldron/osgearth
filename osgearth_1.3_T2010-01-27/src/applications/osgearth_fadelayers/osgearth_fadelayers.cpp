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

#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

#include <osgEarth/MapNode>
#include <osgEarth/FindNode>
#include <osgEarthUtil/FadeLayerNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ElevationFadeCallback>

#include <iostream>

using namespace osg;
using namespace osgDB;
using namespace osgEarth;
using namespace osgEarthUtil;




int main(int argc, char** argv)
{
  osg::ArgumentParser arguments(&argc,argv);

  // construct the viewer.
  osgViewer::Viewer viewer(arguments);

  osg::Group* group = new osg::Group;

  osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);
  if (loadedModel.valid())
  {
    group->addChild(loadedModel.get());
  }
  else
  {
      //Create the map dynamically
	  MapEngineProperties props;
      osg::ref_ptr<MapNode> mapNode = new MapNode(props);

      //Add the yahoo satellite layer
      {
          osgEarth::Config driverProps;
          driverProps.add( "dataset", "satellite" );

          mapNode->getMap()->addMapLayer( new MapLayer(
              "yahoo_satellite",
              MapLayer::TYPE_IMAGE,
              "yahoo",
              driverProps ) );
      }

      //Add the yahoo maps layer
      {
          osgEarth::Config driverProps;
          driverProps.add( "dataset", "roads" );

          mapNode->getMap()->addMapLayer( new MapLayer(
              "yahoo_roads",
              MapLayer::TYPE_IMAGE,
              "yahoo",
              driverProps ) );
      }

      group->addChild(mapNode.get());
  }

  //Find the map
  MapNode* mapNode = findTopMostNodeOfType<MapNode>(group);

  Node* root = group;
  if (mapNode)
  {
      FadeLayerNode* fadeLayerNode = new FadeLayerNode( mapNode->getMap(), mapNode->getEngine()->getEngineProperties() );

      unsigned int numImageLayers = mapNode->getMap()->getImageMapLayers().size();

      //Set all of the layer's opacity to 0.0 except for the first one
      for (unsigned int i = 1; i < numImageLayers; ++i)
      {
          //fadeLayerNode->setOpacity(i, 0.0f);
		  mapNode->getMap()->getImageMapLayers()[i]->setOpacity(0.0f);
      }
      //fadeLayerNode->setOpacity(0, 1.0f);
	  mapNode->getMap()->getImageMapLayers()[0]->setOpacity(1.0f);

	  //Setup the ElevationFadeCallback
	  ElevationFadeCallback* callback = new ElevationFadeCallback();

	  //Set the up elevation fade points
	  double maxElevation = 4e6;
	  for (unsigned int i = 0; i < numImageLayers; ++i)
	  {
		  callback->setElevation( i, maxElevation );
		  maxElevation /= 2.0;
	  }

	  //Attach the callback as both an update and a cull callback
      //fadeLayerNode->setUpdateCallback(callback);
      //fadeLayerNode->setCullCallback(callback);
	  mapNode->setUpdateCallback( callback );
	  mapNode->setCullCallback( callback );

      root = fadeLayerNode;

      fadeLayerNode->addChild(group);
  }
  else
  {
      osg::notify(osg::NOTICE) << "Please load an osgEarth file" << std::endl;
      return 1;
  }
  
  viewer.addEventHandler(new osgViewer::StatsHandler());
  viewer.addEventHandler(new osgViewer::WindowSizeHandler());
  viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

  EarthManipulator* manip = new EarthManipulator();
  manip->getSettings()->setThrowingEnabled( true );
  viewer.setCameraManipulator( manip );

  // set the scene to render
  viewer.setSceneData(root);

  // run the viewers frame loop
  return viewer.run();
}
