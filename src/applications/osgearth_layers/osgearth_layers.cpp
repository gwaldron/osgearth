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

#include <iostream>

#include <osgEarth/Map>
#include <osgEarth/TileSourceFactory>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <osgViewer/ViewerEventHandlers>


using namespace osg;
using namespace osgDB;
using namespace osgTerrain;
using namespace osgEarth;

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

class LayerHandler : public osgGA::GUIEventHandler {
public: 

    LayerHandler(Map* map, osgViewer::View* view):
        _map(map),
        _view(view),
        _selectedLayer(0)
        {}
    
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
        switch(ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYDOWN):
            {   
                if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Left)
                {
                    osgEarth::Layer* layer = _map->getLayer( _selectedLayer );
                    if (layer)
                    {
                        layer->setOpacity( layer->getOpacity() - 0.05);
                    }
                    return true;
                }
                else if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Right)
                {
                    osgEarth::Layer* layer = _map->getLayer( _selectedLayer );
                    if (layer)
                    {
                        layer->setOpacity( layer->getOpacity() + 0.05);
                    }
                    return true;
                }
                //Select the next layer in the list
                else if (ea.getKey() == 'u')
                {
                    _selectedLayer++;
                    _selectedLayer = osg::clampBetween(_selectedLayer, 0u, _map->getNumLayers()-1u);
                    osg::notify(osg::NOTICE) << "Selected layer " << _selectedLayer << std::endl;
                }
                //Select the previous layer in the list
                else if (ea.getKey() == 'j')
                {
                    _selectedLayer--;
                    _selectedLayer = osg::clampBetween(_selectedLayer, 0u, _map->getNumLayers()-1u);
                    osg::notify(osg::NOTICE) << "Selected layer " << _selectedLayer << std::endl;
                }
                //Toggle the visibility/enabling of the selected layer
                else if (ea.getKey() == 'v')
                {
                    osgEarth::Layer* layer = _map->getLayer( _selectedLayer );
                    if (layer)
                    {
                        layer->setEnabled( !layer->getEnabled());
                    }
                    return true;
                }
                //Move the selected layer forward in the layer ordering
                else if (ea.getKey()=='f')
                {
                    _view->getDatabasePager()->clear();
                    osgEarth::Layer* layer = _map->getLayer( _selectedLayer );
                    if (layer)
                    {
                        unsigned int newPosition = osg::clampBetween(_selectedLayer +1, 0u, _map->getNumLayers()-1u);
                        _map->moveLayer( layer, newPosition );
                        osg::notify(osg::NOTICE) << "Moved layer from " << _selectedLayer << " to " << newPosition << std::endl;
                        _selectedLayer = newPosition;
                        osg::notify(osg::NOTICE) << "Selected layer " << _selectedLayer << std::endl;
                    }
                    return true;
                }
                //Move the selected layer back in the layer ordering
                else if (ea.getKey()=='b')
                {
                    _view->getDatabasePager()->clear();
                    osgEarth::Layer* layer = _map->getLayer( _selectedLayer );
                    if (layer)
                    {
                        unsigned int newPosition = osg::clampBetween(_selectedLayer -1, 0u, _map->getNumLayers()-1u);
                        _map->moveLayer( layer, newPosition );
                        osg::notify(osg::NOTICE) << "Moved layer from " << _selectedLayer << " to " << newPosition << std::endl;
                        _selectedLayer = newPosition;
                        osg::notify(osg::NOTICE) << "Selected layer " << _selectedLayer << std::endl;
                    }
                    return true;
                }
                else if (ea.getKey()=='r')
                {
                    osgEarth::Layer* layer = _map->getLayer( _selectedLayer );
                    _view->getDatabasePager()->clear();
                    if (layer)
                    {
                        _map->removeLayer( layer );
                        osg::notify(osg::NOTICE) <<"Removed layer, selected layer is now 0"<< std::endl;
                        _selectedLayer = 0;
                    }
                    return true;
                }
                //Adds an overlay
                else if (ea.getKey()=='a')
                {
                    _view->getDatabasePager()->clear();
                    SourceProperties props;
                    //NOTE:  This must be run from the osgearth/tests directory for the relative path to work
                    props["url"] = "../data/boston-inset.tif";
                    //props["url"] = "c:/dev/osgearth/data/boston-inset.tif";

                    osg::ref_ptr<TileSource> tileSource = _map->createTileSource( new SourceConfig("overlay", "gdal", props ) );
                    if (tileSource.valid())
                    {
                      _map->addLayer( new osgEarth::ImageLayer( tileSource.get() ) );
                    }
                    return true;
                }
                /*else if (ea.getKey()=='b')
                {
                    _view->getDatabasePager()->clear();
                    TileSourceFactory factory;
                    SourceProperties props;
                    props["url"] = "I:/data/land_shallow_topo.tif";

                    osg::ref_ptr<TileSource> tileSource = _map->createTileSource( new SourceConfig("bluemarble", "gdal", props ) );
                    if (tileSource.valid())
                    {
                      _map->addLayer( new osgEarth::ImageLayer( tileSource.get() ) );
                    }
                    return true;
                }
                else if (ea.getKey()=='e')
                {
                    ElevationLayerList elevationLayers;
                    _map->getElevationLayers( elevationLayers );
                    if (!elevationLayers.empty())
                    {
                        _map->removeLayer( elevationLayers.back().get() );
                    }
                    return true;
                }
                else if (ea.getKey() == 'q')
                {
                    _view->getDatabasePager()->clear();
                    TileSourceFactory factory;
                    SourceProperties props;
                    props["url"] = "c:/dev/osgearth/data/terrain/mt_rainier_90m.tif";
                    props["tile_size"] = "32";

                    osg::ref_ptr<TileSource> tileSource = _map->createTileSource( new SourceConfig("mt_rainier", "gdal", props ) );
                    if (tileSource.valid() )
                    {
                        _map->addLayer( new osgEarth::ElevationLayer( tileSource.get() ) );
                    }

                    return true;
                }*/
                return false;
            }    
            default:
                return false;
        }
    }
    
protected:

    ~LayerHandler() {}


    osgViewer::View* _view;
    Map*  _map;
    unsigned int _selectedLayer;
};

int main(int argc, char** argv)
{
  osg::ArgumentParser arguments(&argc,argv);

  // construct the viewer.
  osgViewer::Viewer viewer(arguments);

  // set up the camera manipulators.
  {
      osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

      keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
      keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
      keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
      keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );

      std::string pathfile;
      char keyForAnimationPath = '5';
      while (arguments.read("-p",pathfile))
      {
          osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
          if (apm || !apm->valid()) 
          {
              unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
              keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
              keyswitchManipulator->selectMatrixManipulator(num);
              ++keyForAnimationPath;
          }
      }

      viewer.setCameraManipulator( keyswitchManipulator.get() );
  }

  osg::Group* group = new osg::Group;

  osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);
  if (loadedModel.valid())
  {
    group->addChild(loadedModel.get());
  }

  Map* map = findTopMostNodeOfType<Map>(group);
  if (map)
  {
      viewer.addEventHandler( new LayerHandler( map, &viewer ) );
  }

  // add the state manipulator
  viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
  viewer.addEventHandler(new osgViewer::StatsHandler);


  
  // set the scene to render
  viewer.setSceneData(group);

  // run the viewers frame loop
  return viewer.run();
}
