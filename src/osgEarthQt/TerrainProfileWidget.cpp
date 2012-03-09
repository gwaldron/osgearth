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
#include <osgEarthQt/TerrainProfileWidget>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>
#include <osgEarthQt/TerrainProfileGraph>

#include <osgEarth/Map>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthFeatures/Feature>
#include <osgEarthUtil/TerrainProfile>

//#include <QCheckBox>
//#include <QDropEvent>
#include <QFrame>
//#include <QGraphicsScene>
//#include <QGraphicsView>
#include <QHBoxLayout>
//#include <QPoint>
#include <QPushButton>
//#include <QRect>
//#include <QScrollArea>
//#include <QSlider>
#include <QVBoxLayout>

using namespace osgEarth;
using namespace osgEarth::QtGui;

//---------------------------------------------------------------------------
namespace
{
  struct TerrainProfileMouseHandler : public osgGA::GUIEventHandler
  {
    TerrainProfileMouseHandler(osgEarth::MapNode* mapNode, osg::Group* root, osgEarth::Util::TerrainProfileCalculator* profileCalculator)
      : _mapNode(mapNode), _root(root), _calculator(profileCalculator), _mouseDown(false), _capturing(false), _startValid(false)
    {
      //Define a style for the line
      osgEarth::Symbology::LineSymbol* ls = _lineStyle.getOrCreateSymbol<osgEarth::Symbology::LineSymbol>();
      ls->stroke()->color() = osgEarth::Symbology::Color::Yellow;
      ls->stroke()->width() = 2.0f;
      ls->tessellation() = 20;
      _lineStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()->clamping() = osgEarth::Symbology::AltitudeSymbol::CLAMP_TO_TERRAIN;
    }

    void setCapturing(bool capturing)
    {
      _capturing = capturing;
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
      if (_capturing)
      {
        if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
        {
          if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
          {
            _mouseDown = true;
            _xDown = ea.getX();
            _yDown = ea.getY();
          }
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
        {
          if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
          {
            if (_mouseDown && _xDown == ea.getX() && _yDown == ea.getY())
            {
              osg::Vec3d world;
              if ( _mapNode->getTerrain()->getWorldCoordsUnderMouse( aa.asView(), ea.getX(), ea.getY(), world ))
              {
                osgEarth::GeoPoint mapPoint;
                _mapNode->getMap()->worldPointToMapPoint( world, mapPoint );

                if (!_startValid)
                {
                  _start = mapPoint.vec3d();
                  _end = mapPoint.vec3d();
                  _startValid = true;
                }
                else
                {
                  _end = mapPoint.vec3d();
                  _startValid = false;

                  _calculator->setStartEnd(GeoPoint(_mapNode->getMapSRS(), _start.x(), _start.y(), 0),
                                           GeoPoint(_mapNode->getMapSRS(), _end.x(), _end.y(), 0));
                }

                updateDisplay();
              }
            }

            _mouseDown = false;
          }
        }
        else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
        {
          if (_startValid)
          {
            osg::Vec3d world;
            if (_mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world))
            {
              osgEarth::GeoPoint mapPoint;
              _mapNode->getMap()->worldPointToMapPoint(world, mapPoint);
              _end = mapPoint.vec3d();

              updateDisplay();
            }
          }
        }
      }

      return false;
    }

    void updateDisplay()
    {
      if (!_startValid)
      {
        if (_featureNode.valid())
        {
          _root->removeChild( _featureNode.get() );
          _featureNode = 0L;
        }

        return;
      }

      osgEarth::Symbology::LineString* line = new osgEarth::Symbology::LineString();
      line->push_back( _start );
      line->push_back( _end );

      osgEarth::Features::Feature* feature = new osgEarth::Features::Feature(line, _mapNode->getMapSRS());
      feature->geoInterp() = osgEarth::GEOINTERP_GREAT_CIRCLE;    
      feature->style() = _lineStyle;

      if (!_featureNode.valid())
      {
        _featureNode = new osgEarth::Annotation::FeatureNode( _mapNode, feature );
        _featureNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        _root->addChild( _featureNode.get() );
      }
      else
      {
        _featureNode->setFeature(feature);
      }
    }

    
    osg::ref_ptr<osgEarth::MapNode>  _mapNode;
    osg::Group* _root;
    osgEarth::Util::TerrainProfileCalculator* _calculator;
    osg::ref_ptr<osgEarth::Annotation::FeatureNode> _featureNode;
    osgEarth::Symbology::Style _lineStyle;
    osg::Vec3d _start;
    osg::Vec3d _end;
    bool _capturing;
    bool _startValid;
    bool _mouseDown;
    float _xDown, _yDown;
  };
}

//---------------------------------------------------------------------------

TerrainProfileWidget::TerrainProfileWidget(osg::Group* root, osgEarth::MapNode* mapNode)
: _root(root), _mapNode(mapNode)
{
  _calculator = new osgEarth::Util::TerrainProfileCalculator(mapNode);
  _guiHandler = new TerrainProfileMouseHandler(_mapNode.get(), _root.get(), _calculator.get());

  initialize();
}

TerrainProfileWidget::~TerrainProfileWidget()
{
}

void TerrainProfileWidget::initialize()
{
  QVBoxLayout* vStack = new QVBoxLayout();
  vStack->setSpacing(0);
  vStack->setContentsMargins(0, 0, 0, 0);
  setLayout(vStack);

  QHBoxLayout* buttonStack = new QHBoxLayout();
  buttonStack->setSpacing(0);
  buttonStack->setContentsMargins(0, 0, 0, 0);

  buttonStack->addStretch();

  _captureButton = new QPushButton(QIcon(":/images/crosshair.png"), tr(""));
  _captureButton->setCheckable(true);
  _captureButton->setMaximumSize(24, 24);
  buttonStack->addWidget(_captureButton);

  vStack->addLayout(buttonStack);

  _graph = new TerrainProfileGraph(_calculator);
  vStack->addWidget(_graph);

  connect(_captureButton, SIGNAL(toggled(bool)), this, SLOT(onCaptureToggled(bool)));
}

void TerrainProfileWidget::setActiveView(osgViewer::View* view)
{
  removeViews();
  addView(view);
}

void TerrainProfileWidget::setActiveViews(const ViewVector& views)
{
  removeViews();

  for (ViewVector::const_iterator it = views.begin(); it != views.end(); ++it)
    addView(*it);
}

void TerrainProfileWidget::addView(osgViewer::View* view)
{
  view->addEventHandler(_guiHandler.get());
  _views.push_back(view);
}

void TerrainProfileWidget::removeViews()
{
  for (ViewVector::iterator it = _views.begin(); it != _views.end(); ++it)
    (*it)->removeEventHandler(_guiHandler.get());

  _views.clear();
}

void TerrainProfileWidget::onCaptureToggled(bool checked)
{
  ((TerrainProfileMouseHandler*)_guiHandler.get())->setCapturing(checked);
}