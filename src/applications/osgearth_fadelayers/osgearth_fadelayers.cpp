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

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Projection>
#include <osg/AutoTransform>
#include <osg/Geometry>
#include <osg/Image>
#include <osg/CullFace>

#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>

#include <osgDB/WriteFile>

#include <osgText/Text>

#include <osgEarth/Map>
#include <osgEarthUtil/FadeLayerNode>

#include <iostream>

using namespace osg;
using namespace osgDB;
using namespace osgEarth;
using namespace osgEarthUtil;



template<class T>
class FindTopMostNodeOfTypeVisitor : public osg::NodeVisitor
{
public:
    FindTopMostNodeOfTypeVisitor():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        _foundNode(0)
    {}
    
    void apply(osg::Node& node)
    {
        T* result = dynamic_cast<T*>(&node);
        if (result)
        {
            _foundNode = result;
        }
        else
        {
            traverse(node);
        }
    }
    
    T* _foundNode;
};

template<class T>
T* findTopMostNodeOfType(osg::Node* node)
{
    if (!node) return 0;

    FindTopMostNodeOfTypeVisitor<T> fnotv;
    node->accept(fnotv);
    
    return fnotv._foundNode;
}




class FadeLayerCallback : public osg::NodeCallback
{
public:
    typedef std::vector<double> Elevations;

    FadeLayerCallback(Map* map, FadeLayerNode* fadeLayerNode, const Elevations& elevations, float animationTime=2.0f):
      _map(map),
      _fadeLayerNode(fadeLayerNode),
      _firstFrame(true),
      _previousTime(0.0),
      _elevations(elevations),
      _animationTime(animationTime),
      _currentElevation(0)
      {
          //Find the coordinate system node
          _csn = findTopMostNodeOfType<osg::CoordinateSystemNode>(_map.get());
      }

      virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
      {
          if (nv->getVisitorType() == NodeVisitor::UPDATE_VISITOR)
          {
              if (!_firstFrame)
              {
                  double deltaTime = nv->getFrameStamp()->getReferenceTime() - _previousTime;

                  _previousTime = nv->getFrameStamp()->getReferenceTime();

                  double delta = osg::minimum(deltaTime / _animationTime, 1.0);

                  //Determine which layer should be active
                  unsigned int activeLayer = _map->getNumLayers()-1;
                  for (unsigned int i = 0; i < _elevations.size(); ++i)
                  {
                      if (_currentElevation > _elevations[i])
                      {
                          activeLayer = i;
                          break;
                      }
                  }

                  bool dirtyLayers = false;
                  for (unsigned int i = 0; i < _map->getNumLayers(); ++i)
                  {
                      //If the layer that we are looking at is greater than the active layer, we want to fade it out to 0.0
                      //Otherwise, we want the layers to go to 1.0
                      float goalOpacity = (i > activeLayer) ? 0.0f : 1.0f;
                      float currentOpacity = _fadeLayerNode->getOpacity(i);

                      if (goalOpacity != currentOpacity)
                      {
                          if (currentOpacity > goalOpacity) delta = -delta;
                          _fadeLayerNode->setOpacity(i, currentOpacity + delta);
                      }
                  }
              }
              _firstFrame = false;
          }
          else if (nv->getVisitorType() == NodeVisitor::CULL_VISITOR)
          {
              //Get the current elevation
              _currentElevation = nv->getViewPoint().z();
              if (_csn.valid())
              {
                  osg::EllipsoidModel* em = _csn->getEllipsoidModel();
                  if (em)
                  {
                      double x = nv->getViewPoint().x();
                      double y = nv->getViewPoint().y();
                      double z = nv->getViewPoint().z();
                      double latitude, longitude;
                      em->convertXYZToLatLongHeight(x, y, z, latitude, longitude, _currentElevation);
                  }
              }
          }
          //Continue traversal
          traverse(node, nv);
      }

private:
    osg::observer_ptr<FadeLayerNode> _fadeLayerNode;
    osg::observer_ptr<Map> _map;
    osg::observer_ptr<CoordinateSystemNode> _csn;
    double _currentElevation;
    Elevations _elevations;
    bool _firstFrame;
    double _previousTime;
    double _animationTime;
};


int main(int argc, char** argv)
{
  osg::ArgumentParser arguments(&argc,argv);

  // construct the viewer.
  osgViewer::Viewer viewer(arguments);

  osg::Group* group = new osg::Group;

  FadeLayerNode *fadeLayerNode = new FadeLayerNode;
  group->addChild(fadeLayerNode);

  osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);
  if (loadedModel.valid())
  {
    fadeLayerNode->addChild(loadedModel.get());
  }
  else
  {
      //Create the map dynamically
      MapConfig mapConfig;
      mapConfig.setCoordinateSystemType( MapConfig::CSTYPE_GEOCENTRIC );
      osg::ref_ptr<Map> map = new Map(mapConfig);

      //Add the yahoo satellite layer
      {
          SourceProperties props;
          props["dataset"] = "satellite";
          map->addLayer(new ImageLayer(map->createTileSource( SourceConfig("yahoo_sat", "yahoo", props) ) ) );
      }

      //Add the yahoo maps layer
      {
          SourceProperties props;
          props["dataset"] = "roads";
          map->addLayer(new ImageLayer(map->createTileSource( SourceConfig("yahoo_roads", "yahoo", props) ) ) );
      }
      fadeLayerNode->addChild(map.get());
  }

  //Find the map
  Map* map = findTopMostNodeOfType<Map>(group);
  if (map)
  {
      //Set the up elevation fade points
      FadeLayerCallback::Elevations elevations;
      double maxElevation = 4e6;
      for (unsigned int i = 0; i < map->getNumLayers(); ++i)
      {
          elevations.push_back(maxElevation);
          maxElevation /= 2.0;
      }

      //Set all of the layer's opacity to 0.0 except for the first one
      for (unsigned int i = 1; i < map->getNumLayers(); ++i)
      {
          fadeLayerNode->setOpacity(i, 0.0f);
      }
      fadeLayerNode->setOpacity(0, 1.0f);

      //Attach the callback as both a cull and update callback
      FadeLayerCallback* callback = new FadeLayerCallback(map, fadeLayerNode, elevations);
      fadeLayerNode->setUpdateCallback(callback);
      fadeLayerNode->setCullCallback(callback);
  }
  else
  {
      osg::notify(osg::NOTICE) << "Please load an osgEarth file" << std::endl;
  }

  // set the scene to render
  viewer.setSceneData(group);

  // run the viewers frame loop
  return viewer.run();
}
