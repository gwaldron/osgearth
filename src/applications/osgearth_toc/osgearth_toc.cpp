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

#include <osgWidget/Util>
#include <osgWidget/WindowManager>
#include <osgWidget/Box>
#include <osgWidget/Label>
#include <osgWidget/ViewerEventHandlers>

#include <osgViewer/ViewerEventHandlers>


using namespace osg;
using namespace osgDB;
using namespace osgTerrain;
using namespace osgEarth;
using namespace osgWidget;

Vec4 normalColor(1,1,1,1);
Vec4 hotColor(0.4,0.56,1,1);

const unsigned int MASK_2D = 0xF0000000;

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

osgWidget::Label* createLabel(const std::string& l, unsigned int size=13) {
    osgWidget::Label* label = new osgWidget::Label("", "");

    label->setFont("fonts/vera.ttf");
    label->setColor(1.0f, 1.0f, 1.0f, 0.0f);
    label->setFontSize(size);
    label->setFontColor(1.0f, 1.0f, 1.0f, 1.0f);
    label->setLabel(l);
    label->getText()->setBackdropType(osgText::Text::OUTLINE);

    return label;
}

//Simple hot tracking callback that changes the color of labels when the mouse enters and leaves
struct HotTrackingCallback: public osgWidget::Callback {
    HotTrackingCallback(const osg::Vec4 &normalColor, const osg::Vec4 &hotColor):
osgWidget::Callback(osgWidget::EventType(osgWidget::EVENT_MOUSE_ENTER | osgWidget::EVENT_MOUSE_LEAVE)),
_normalColor(normalColor),
_hotColor(hotColor)
{
}
    virtual bool operator()(osgWidget::Event& ev) {
        if (ev.type == osgWidget::EVENT_MOUSE_ENTER)
        {
            ((osgWidget::Label*)ev.getWidget())->setFontColor(_hotColor.r(), _hotColor.g(), _hotColor.b(), _hotColor.a());
        }
        else if (ev.type == osgWidget::EVENT_MOUSE_LEAVE)
        {
            ((osgWidget::Label*)ev.getWidget())->setFontColor(_normalColor.r(), _normalColor.g(), _normalColor.b(), _normalColor.a());
        }
        return false;
    }

    osg::Vec4 _normalColor;
    osg::Vec4 _hotColor;
};

//Callback that toggles the visibility of a layer
struct ToggleVisiblityCallback: public osgWidget::Callback {
    ToggleVisiblityCallback(Map* map, unsigned int layerIndex):
osgWidget::Callback(osgWidget::EVENT_MOUSE_PUSH),
_map(map),
_layerIndex(layerIndex)
{
}
    virtual bool operator()(osgWidget::Event& ev) {
        if (ev.type == osgWidget::EVENT_MOUSE_PUSH)
        {
            osgEarth::Layer* layer = _map->getLayer(_layerIndex);
            if (layer)
            {
                layer->setEnabled(!layer->getEnabled());
                _map->dirtyLayers();
            }
        }
        return true;
    }

    osg::ref_ptr<Map> _map;
    unsigned int _layerIndex;
};

//Callback that increases/decreases the opacity of a layer
struct OpacityCallback: public osgWidget::Callback {
    OpacityCallback(Map* map, unsigned int layerIndex, float opacityDelta):
osgWidget::Callback(osgWidget::EVENT_MOUSE_PUSH),
_map(map),
_layerIndex(layerIndex),
_opacityDelta(opacityDelta)
{
}
    virtual bool operator()(osgWidget::Event& ev) {
        if (ev.type == osgWidget::EVENT_MOUSE_PUSH)
        {
            osgEarth::Layer* layer = _map->getLayer(_layerIndex);
            if (layer)
            {
                layer->setOpacity(layer->getOpacity() + _opacityDelta);
                _map->dirtyLayers();
            }
        }
        return true;
    }

    osg::ref_ptr<Map> _map;
    unsigned int _layerIndex;
    float _opacityDelta;
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
      osg::notify(osg::NOTICE) << "Found Map" << std::endl;
  }

  //Setup the osgWidget interface
  osgWidget::WindowManager* wm = new osgWidget::WindowManager(
        &viewer,
        800.0f,
        800.0f,
        MASK_2D,
        0
    );

  ImageLayerList imageLayers;
  map->getImageLayers(imageLayers);

  for (unsigned int i = 0; i < imageLayers.size(); ++i)
  {
      float textSize = 25.0f;

      osgEarth::Layer* layer = imageLayers[i].get();
      std::stringstream ss;
      unsigned int index = (imageLayers.size() - i);
      ss << index << ") ";

      osgWidget::Box* line = new osgWidget::Box("HBOX", osgWidget::Box::HORIZONTAL);

      //Add a label with the layer #
      osgWidget::Label* lblNum = createLabel(ss.str(), textSize);
      lblNum->setEventMask(osgWidget::EVENT_ALL);  
      lblNum->setPadding(3.0f);
      lblNum->addCallback(new HotTrackingCallback(normalColor, hotColor));
      line->addWidget(lblNum);

      //Add a label to turn down the opacity
      osgWidget::Label* lblOpacityDown = createLabel("<", textSize);
      lblOpacityDown->setEventMask(osgWidget::EVENT_ALL);  
      lblOpacityDown->setPadding(3.0f);
      lblOpacityDown->addCallback(new HotTrackingCallback(normalColor, hotColor));
      lblOpacityDown->addCallback(new OpacityCallback(map, i, -0.1));
      line->addWidget(lblOpacityDown);

      //Add a label to turn the opacity up
      osgWidget::Label* lblOpacityUp = createLabel(">", textSize);
      lblOpacityUp->setEventMask(osgWidget::EVENT_ALL);  
      lblOpacityUp->setPadding(3);
      lblOpacityUp->addCallback( new HotTrackingCallback(normalColor, hotColor));
      lblOpacityUp->addCallback(new OpacityCallback(map, i, 0.1));
      line->addWidget(lblOpacityUp);

      //Add a label with the name of the layer, clicking on the layer 
      osgWidget::Label* lblName = createLabel(layer->getName(), textSize);
      lblName->setEventMask(osgWidget::EVENT_ALL);  
      lblName->setPadding(3.0f);
      lblName->addCallback(new HotTrackingCallback(normalColor, hotColor));
      lblName->addCallback(new ToggleVisiblityCallback(map, i));
      line->addWidget(lblName);

      line->getBackground()->setColor(0,0,0,0);

      //Push the row up
      line->setOrigin(0, (textSize + 10)*i);

      wm->addChild(line);
  }

   viewer.setUpViewInWindow(10, 10, 800,800);



    osg::Camera* camera = wm->createParentOrthoCamera();
    group->addChild(camera);

    viewer.addEventHandler(new osgWidget::MouseHandler(wm));
    viewer.addEventHandler(new osgWidget::KeyboardHandler(wm));
    viewer.addEventHandler(new osgWidget::ResizeHandler(wm, camera));
    viewer.addEventHandler(new osgWidget::CameraSwitchHandler(wm, camera));

    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );


    wm->resizeAllWindows();

  
  // set the scene to render
  viewer.setSceneData(group);

  // run the viewers frame loop
  return viewer.run();
}
