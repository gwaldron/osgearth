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

#include <osgEarthDrivers/yahoo/YahooOptions>

#include <iostream>

using namespace osg;
using namespace osgDB;
using namespace osgEarth;
using namespace osgEarthUtil;
using namespace osgEarth::Drivers;


int main(int argc, char** argv)
{
  osg::ArgumentParser arguments(&argc,argv);

  // construct the viewer.
  osgViewer::Viewer viewer(arguments);

  std::string temp;

  float elevationInterval = 4e6;
  if ( arguments.read( "--interval", temp ) )
      sscanf( temp.c_str(), "%f", &elevationInterval );

  float fadeBuffer = 3e6;
  if ( arguments.read( "--buffer", temp ) )
      sscanf( temp.c_str(), "%f", &fadeBuffer );

  osg::ref_ptr<osg::Node> root = osgDB::readNodeFiles(arguments);

  //Find the map
  MapNode* mapNode = findTopMostNodeOfType<MapNode>(root.get());
  if (mapNode)
  {
      ImageLayerVector imageLayers;
      mapNode->getMap()->getImageLayers( imageLayers );

	  //Setup the ElevationFadeCallback
	  ElevationFadeCallback* callback = new ElevationFadeCallback();

	  //Set the up elevation fade points
      callback->setElevationRange( imageLayers[0].get(), FLT_MAX, 0, fadeBuffer ); //elevationInterval, fadeBuffer );
	  for (unsigned int i = 1; i < imageLayers.size(); ++i )
	  {
          callback->setElevationRange( imageLayers[i].get(), elevationInterval, 0.0f, elevationInterval/4.0f );
		  elevationInterval /= 2.0f;
	  }
      callback->setElevationRange( imageLayers[imageLayers.size()-1].get(), elevationInterval*2.0f, 0.0f );
      
	  //Attach the callback as both an update and a cull callback
	  mapNode->addUpdateCallback( callback );
	  mapNode->addCullCallback( callback );
  }
  else
  {
      OE_NOTICE << "Please load an osgEarth file" << std::endl;
      return 1;
  }
  
  viewer.addEventHandler(new osgViewer::StatsHandler());
  viewer.addEventHandler(new osgViewer::WindowSizeHandler());
  viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

  EarthManipulator* manip = new EarthManipulator();
  manip->getSettings()->setThrowingEnabled( true );
  viewer.setCameraManipulator( manip );

  // set the scene to render
  viewer.setSceneData( root.get() );

  // run the viewers frame loop
  return viewer.run();
}
