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
#include <osgEarth/FindNode>

#include <osgEarthUtil/FadeLayerNode>

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
using namespace osgEarthUtil;
using namespace osgWidget;

Vec4 normalColor(1,1,1,1);
Vec4 hotColor(0.4,0.56,1,1);
float textSize = 25.0f;
bool hudDirty = false;

const unsigned int MASK_2D = 0xF0000000;

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
    return true;
}

osg::Vec4 _normalColor;
osg::Vec4 _hotColor;
};

//Callback that toggles the visibility of a layer
struct ToggleVisiblityCallback: public osgWidget::Callback {
    ToggleVisiblityCallback(FadeLayerNode* fadeLayerNode, unsigned int layerIndex):
osgWidget::Callback(osgWidget::EVENT_MOUSE_PUSH),
_fadeLayerNode(fadeLayerNode),
_layerIndex(layerIndex)
{
}
virtual bool operator()(osgWidget::Event& ev) {
    if (ev.type == osgWidget::EVENT_MOUSE_PUSH)
    {
        _fadeLayerNode->setEnabled(_layerIndex, !_fadeLayerNode->getEnabled(_layerIndex));
    }
    return true;
}

osg::ref_ptr<FadeLayerNode> _fadeLayerNode;
unsigned int _layerIndex;
};

//Callback that increases/decreases the opacity of a layer
struct OpacityCallback: public osgWidget::Callback
{
    OpacityCallback(FadeLayerNode* fadeLayerNode, unsigned int layerIndex, float opacityDelta):
osgWidget::Callback(osgWidget::EVENT_MOUSE_PUSH),
_fadeLayerNode(fadeLayerNode),
_layerIndex(layerIndex),
_opacityDelta(opacityDelta)
{
}
virtual bool operator()(osgWidget::Event& ev) {
    if (ev.type == osgWidget::EVENT_MOUSE_PUSH)
    {
        _fadeLayerNode->setOpacity(_layerIndex, _fadeLayerNode->getOpacity(_layerIndex) + _opacityDelta);
    }
    return true;
}

osg::ref_ptr<FadeLayerNode> _fadeLayerNode;
unsigned int _layerIndex;
float _opacityDelta;
};

//Callback that removes a Layer
struct RemoveLayerCallback: public osgWidget::Callback
{
    RemoveLayerCallback(Map* map, osgViewer::View* view, unsigned int layerIndex):
osgWidget::Callback(osgWidget::EVENT_MOUSE_PUSH),
_map(map),
_view(view),
_layerIndex(layerIndex)
{
}
virtual bool operator()(osgWidget::Event& ev) {
    if (ev.type == osgWidget::EVENT_MOUSE_PUSH)
    {
        _view->getDatabasePager()->clear();
        osgEarth::Layer* layer = _map->getLayer(_layerIndex);
        if (layer)
        {
            _map->removeLayer(layer);
            hudDirty = true;
        }
    }
    return true;
}

osg::ref_ptr<Map> _map;
unsigned int _layerIndex;
osgViewer::View* _view;
};

//Callback that removes a Layer
struct MoveLayerCallback: public osgWidget::Callback
{
    MoveLayerCallback(Map* map, osgViewer::View* view, unsigned int layerIndex, bool up):
osgWidget::Callback(osgWidget::EVENT_MOUSE_PUSH),
_map(map),
_view(view),
_layerIndex(layerIndex),
_up(up)
{
}
virtual bool operator()(osgWidget::Event& ev) {
    if (ev.type == osgWidget::EVENT_MOUSE_PUSH)
    {
        _view->getDatabasePager()->clear();
        osgEarth::Layer* layer = _map->getLayer( _layerIndex );
        if (layer)
        {
            int dir = _up ? 1 : -1;
            unsigned int newPosition = osg::clampBetween(_layerIndex + dir, 0u, _map->getNumLayers()-1u);
            _map->moveLayer( layer, newPosition );
            hudDirty = true;
        }
    }
    return true;
}

osg::ref_ptr<Map> _map;
osgViewer::View* _view;
unsigned int _layerIndex;
bool _up;
};

class AddLayerButton : public osgWidget::Label
{
public:
    AddLayerButton(Map* map, osgViewer::View* view, SourceConfig sourceConfig):
      osgWidget::Label("",""),
          _map(map),
          _view(view),
          _sourceConfig(sourceConfig)
      {
          setEventMask(osgWidget::EVENT_ALL);
          setFont("fonts/vera.ttf");
          setColor(1.0f, 1.0f, 1.0f, 0.0f);
          setFontSize(textSize);
          setFontColor(1.0f, 1.0f, 1.0f, 1.0f);
          setLabel("Add " + sourceConfig.getName());
          addCallback(new HotTrackingCallback(normalColor, hotColor));
          getText()->setBackdropType(osgText::Text::OUTLINE);
      }

     virtual bool mousePush(double, double, osgWidget::WindowManager*) {
         _view->getDatabasePager()->clear();
         osg::ref_ptr<TileSource> tileSource = _map->createTileSource(_sourceConfig);
         if (tileSource.valid())
         {
             _map->addLayer( new osgEarth::ImageLayer( tileSource.get() ) );
             hudDirty = true;
         }      
        return true;
    }

     osg::ref_ptr<Map> _map;
     SourceConfig _sourceConfig;
     osgViewer::View* _view;
     osg::ref_ptr<FadeLayerNode> _fadeLayerNode;
};

void createAddLayersMenu(osgWidget::WindowManager* wm, FadeLayerNode* fadeLayerNode, Map* map, osgViewer::View* view)
{
     osgWidget::Box* addLayersBox = new osgWidget::Box("AddLayersBox", osgWidget::Box::VERTICAL);

    //Add Google Labels
    {
        SourceProperties props;
        props["dataset"] = "labels";
        addLayersBox->addWidget(new AddLayerButton(map, view, SourceConfig("Google Labels", "google", props)));
    }

    //Add Google Roads
    {
        SourceProperties props;
        props["dataset"] = "roads";
        addLayersBox->addWidget(new AddLayerButton(map, view, SourceConfig("Google Roads", "google", props)));
    }

    //Add Google Traffic
    {
        SourceProperties props;
        props["dataset"] = "traffic";
        addLayersBox->addWidget(new AddLayerButton(map, view, SourceConfig("Google Traffic", "google", props)));
    }

    //Add Google Imagery
    {
        SourceProperties props;
        props["dataset"] = "imagery";
        addLayersBox->addWidget(new AddLayerButton(map, view, SourceConfig("Google Imagery", "google", props)));
    }

    //Add Yahoo Maps
    {
        SourceProperties props;
        props["dataset"] = "roads";
        addLayersBox->addWidget(new AddLayerButton(map, view, SourceConfig("Yahoo Maps", "yahoo", props)));
    }

    //Add Yahoo Imagery
    {
        SourceProperties props;
        props["dataset"] = "satellite";
        addLayersBox->addWidget(new AddLayerButton(map, view, SourceConfig("Yahoo Imagery", "yahoo", props)));
    }

    //addLayersBox->attachMoveCallback();
    addLayersBox->getBackground()->setColor(1,0,0,0.3);
    addLayersBox->setAnchorHorizontal(osgWidget::Window::HA_RIGHT);
    wm->addChild(addLayersBox);
    addLayersBox->resize();
}

class Line : public osgWidget::Box
{
public:
    Line(Map* map, FadeLayerNode* fadeLayerNode, osgViewer::View* view, unsigned int layerIndex):
_map(map),
_fadeLayerNode(fadeLayerNode),
_layerIndex(layerIndex),
_view(view),
osgWidget::Box("", osgWidget::Box::HORIZONTAL)
{
    //Create the number string
    _lblNum = createLabel();
    addWidget(_lblNum.get());

    //Create a button to remove the layer
    _lblRemove = createLabel();
    _lblRemove->setLabel("X");
    _lblRemove->addCallback(new RemoveLayerCallback(map, view, _layerIndex));
    addWidget(_lblRemove.get());

    //Add a label to turn down the opacity
    _lblOpacityDown = createLabel();
    _lblOpacityDown->setLabel("<");        
    _lblOpacityDown->addCallback(new OpacityCallback(_fadeLayerNode, _layerIndex, -0.1));
    addWidget(_lblOpacityDown.get());

    //Add a label to turn the opacity up
    _lblOpacityUp = createLabel();
    _lblOpacityUp->setLabel(">");
    _lblOpacityUp->addCallback(new OpacityCallback(_fadeLayerNode, _layerIndex, 0.1));
    addWidget(_lblOpacityUp.get());

    //Create a button to move the layer up
    _lblMoveUp = createLabel();
    _lblMoveUp->setLabel("Up");
    _lblMoveUp->addCallback(new MoveLayerCallback(map, view, _layerIndex, true));
    addWidget(_lblMoveUp.get());

    //Create a button to move the layer down
    _lblMoveDown = createLabel();
    _lblMoveDown->setLabel("Down");
    _lblMoveDown->addCallback(new MoveLayerCallback(map, view, _layerIndex, false));
    addWidget(_lblMoveDown.get());

    //Add a label with the name of the layer, clicking on the layer 
    _lblName = createLabel();
    _lblName->addCallback(new ToggleVisiblityCallback(fadeLayerNode, _layerIndex));
    addWidget(_lblName.get());

    getBackground()->setColor(0,0,0,0);

    updateText();
}

void updateText()
{
    osgEarth::Layer* layer = _map->getLayer(_layerIndex);
    if (layer)
    {
        std::stringstream ss;
        unsigned int index = (_map->getNumLayers() - _layerIndex);
        ss << index << ") ";
        _lblNum->setLabel(ss.str());
        _lblName->setLabel( layer->getName());
    }
}

osgWidget::Label* createLabel()
{
    osgWidget::Label* label = new osgWidget::Label("", "");
    label->setFont("fonts/vera.ttf");
    label->setColor(1.0f, 1.0f, 1.0f, 0.0f);
    label->setFontSize(textSize);
    label->setFontColor(1.0f, 1.0f, 1.0f, 1.0f);
    label->getText()->setBackdropType(osgText::Text::OUTLINE);
    label->setEventMask(osgWidget::EVENT_ALL);  
    label->setPadding(3.0f);
    label->addCallback(new HotTrackingCallback(normalColor, hotColor));
    return label;
}

osg::ref_ptr<osgWidget::Label> _lblName;
osg::ref_ptr<osgWidget::Label> _lblNum;
osg::ref_ptr<osgWidget::Label> _lblMoveUp;
osg::ref_ptr<osgWidget::Label> _lblMoveDown;
osg::ref_ptr<osgWidget::Label> _lblOpacityUp;
osg::ref_ptr<osgWidget::Label> _lblOpacityDown;
osg::ref_ptr<osgWidget::Label> _lblRemove;
unsigned int _layerIndex;
osg::ref_ptr<Map> _map;
osg::ref_ptr<FadeLayerNode> _fadeLayerNode;
osgViewer::View* _view;
};

class TOC
{
public:
    TOC(osgWidget::WindowManager* wm, Map* map, FadeLayerNode* fadeLayerNode, osgViewer::View* view):
      _wm(wm),
          _map(map),
          _fadeLayerNode(fadeLayerNode),
          _view(view)
      {
          //Create the lines
          unsigned int maxLayers = 4;
          for (unsigned int i = 0; i < maxLayers; ++i)
          {
              Line* line = new Line(_map,fadeLayerNode, _view, i);
              _lines.push_back(line);
          }
          update();
      }

      void update()
      {
          //Remove the existing lines
          for (unsigned int i = 0; i < _lines.size(); ++i)
          {
              _wm->removeChild(_lines[i].get());
          }

          ImageLayerList imageLayers;
          _map->getImageLayers(imageLayers);
          for (unsigned int i = 0; i < imageLayers.size(); ++i)
          {
              Line* line = i < _lines.size() ? _lines[i] : NULL;
              if (line)
              {
                  line->updateText();
                  //Push the row up
                  line->setOrigin(0, (textSize + 10)*i);
                  _wm->addChild(line);
              }
          }
      }

      std::vector<osg::ref_ptr<Line> > _lines;

      osgWidget::WindowManager* _wm;
      Map* _map;
      FadeLayerNode* _fadeLayerNode;
      osgViewer::View* _view;
};

class TOCUpdateCallback : public osg::NodeCallback
{
public:
    TOCUpdateCallback(TOC* toc)
    {
        _toc = toc;
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if (hudDirty)
        {
            _toc->update();
            hudDirty = false;
        }
        traverse(node, nv);
    }

    TOC* _toc;
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
    if (!loadedModel.valid())
    {
        osg::notify(osg::NOTICE) << "Failed to load map" << std::endl;
        return 1;
    }

    Map* map = findTopMostNodeOfType<Map>(loadedModel.get());
    if (!map)
    {
        osg::notify(osg::NOTICE) << "Please load a .earth file" << std::endl;
        return 1;
    }

    FadeLayerNode* fadeLayerNode = new FadeLayerNode(map);
    fadeLayerNode->addChild(loadedModel.get());
    group->addChild(fadeLayerNode);


    for (unsigned int i = 0; i < map->getNumLayers(); ++i)
    {
        fadeLayerNode->setOpacity(i, 1.0f);
        fadeLayerNode->setEnabled(i, true);
    }


    //Setup the osgWidget interface
    osgWidget::WindowManager* wm = new osgWidget::WindowManager(
        &viewer,
        800.0f,
        800.0f,
        MASK_2D,
        0
        );

    TOC toc(wm, map, fadeLayerNode, &viewer);
    createAddLayersMenu(wm, fadeLayerNode, map, &viewer);
    group->setDataVariance(osg::Object::DYNAMIC);
    group->setUpdateCallback(new TOCUpdateCallback(&toc));


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
