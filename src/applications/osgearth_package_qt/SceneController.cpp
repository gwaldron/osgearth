/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include "SceneController.h"

#include <osgEarth/Common>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>


using namespace PackageQt;

#define LC "[SceneController] "

namespace
{
  struct BoundingBoxMouseHandler : public osgGA::GUIEventHandler
  {
    BoundingBoxMouseHandler(SceneController* controller, bool startCapture=false)
      : _controller(controller), _mouseDown(false), _capturing(startCapture)
    {
    }

    void startCapture()
    {
      _capturing = true;
    }

    void stopCapture()
    {
      _capturing = false;
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
      if (!_capturing || !_controller)
        return false;

      osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

      if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH  && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
      {
        osg::Vec3d world;
        if (_controller->mapNode()->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world))
        {
          osgEarth::GeoPoint mapPoint;
          mapPoint.fromWorld( _controller->mapNode()->getMapSRS(), world );
          _startPoint = mapPoint;
        }

        std::cout << "BBox DOWN" << std::endl;

        _mouseDown = true;

        ea.setHandled(true);
        return true;
      }
      else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG && _mouseDown)
      {
        osg::Vec3d world;
        if (_controller->mapNode()->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world))
        {
          osgEarth::GeoPoint mapPoint;
          mapPoint.fromWorld( _controller->mapNode()->getMapSRS(), world );
          _controller->setBounds(_startPoint, mapPoint);
        }

        std::cout << "BBox MOVE" << std::endl;

        ea.setHandled(true);
        return true;
      }
      else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON && _mouseDown)
      {
        _mouseDown = false;
        _capturing = false;
        _controller->endBoundsCapture();

        std::cout << "BBox UP" << std::endl;

        ea.setHandled(true);
        return true;
      }

      return false;
    }


    SceneController* _controller;
    bool _capturing;
    bool _mouseDown;
    osgEarth::GeoPoint _startPoint;
  };
}

SceneController::SceneController(osg::Group* root, osgViewer::View* view, const std::string& url)
: _root(root), _view(view)
{
  if (_root.valid() && _view.valid())
  {
    //install a canvas for any UI controls we plan to create
    _canvas = osgEarth::Util::Controls::ControlCanvas::getOrCreate(_view);

    _controlContainer = _canvas->addControl( new osgEarth::Util::Controls::VBox() );
    _controlContainer->setBackColor( osgEarth::Util::Controls::Color(osgEarth::Util::Controls::Color::Black, 0.8) );
    _controlContainer->setHorizAlign( osgEarth::Util::Controls::Control::ALIGN_LEFT );
    _controlContainer->setVertAlign( osgEarth::Util::Controls::Control::ALIGN_BOTTOM );

    root->addChild(_canvas);

    //create the root group for annotations
    _annoRoot = new osg::Group();
    root->addChild(_annoRoot);
  }

  //Setup bounding box style
  osgEarth::Symbology::LineSymbol* ls = _boundsStyle.getOrCreate<osgEarth::Symbology::LineSymbol>();
  ls->stroke()->color() = osgEarth::Symbology::Color::Red;
  ls->stroke()->width() = 3.0f;
  ls->stroke()->stipple() = 0x0F0F;
  //ls->tessellation() = 20;
  _boundsStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()->clamping() = osgEarth::Symbology::AltitudeSymbol::CLAMP_TO_TERRAIN;
  _boundsStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()->technique() = osgEarth::Symbology::AltitudeSymbol::TECHNIQUE_GPU;


  loadEarthFile(url);
}

osg::Node* SceneController::loadEarthFile(const std::string& url)
{
  if (!_root.valid() || !_view.valid())
    return 0L;

  if (_earthNode.valid())
  {
    _root->removeChild(_earthNode);
    _earthNode = 0L;
  }

  //if (_sky.valid())
  //{
  //  _root->removeChild(_sky);
  //  _sky = 0L;
  //}

  if (_controlContainer.valid())
    _controlContainer->clearControls();

  if (url.length() > 0)
    _earthNode = osgDB::readNodeFile( url );

  //Load a blank globe if needed
  if (!_earthNode.valid())
    _earthNode = new osgEarth::MapNode(new osgEarth::Map());

  if (_earthNode.valid())
  {
    _mapNode = osgEarth::MapNode::findMapNode( _earthNode );
    if (_mapNode.valid())
    {        
      _map = _mapNode->getMap();
      _earthFilePath = url;
      OE_NOTICE << "Set earth file path to " << _earthFilePath << std::endl;

      //const osgEarth::Config& externals = _mapNode->externalConfig();

      //if (_map->isGeocentric())
      //{
      //  // Sky model.
      //  osgEarth::Config skyConf = externals.child("sky");

      //  double hours = skyConf.value("hours", 12.0);

      //  _sky = new osgEarth::Util::SkyNode(_map);
      //  _sky->setAutoAmbience( true );
      //  _sky->setDateTime(2011, 3, 6, hours);
      //  _sky->attach(_view);

      //  _root->addChild(_sky);

      //  if (_controlContainer.valid())
      //  {
      //    osgEarth::Util::Controls::Control* c = osgEarth::Util::SkyControlFactory().create(_sky, _view);
      //    if ( c )
      //        _controlContainer->addControl( c );
      //  }
      //}
    }

    _root->addChild(_earthNode);

    //Create new EarthManipulator
    _view->setCameraManipulator(new osgEarth::Util::EarthManipulator());
  }

  return _earthNode.get();
}

void SceneController::captureBounds(BoundsSetCallback* callback)
{
  _boundsCallback = callback;

  if (!_guiHandler.valid())
  {
    _guiHandler = new BoundingBoxMouseHandler(this, true);
    _view->addEventHandler(_guiHandler.get());
  }
  else
  {
    dynamic_cast<BoundingBoxMouseHandler *>(_guiHandler.get())->startCapture();
  }
}

void SceneController::endBoundsCapture()
{
  if (_guiHandler.valid())
    dynamic_cast<BoundingBoxMouseHandler *>(_guiHandler.get())->stopCapture();

  if (_boundsCallback.valid())
    _boundsCallback->boundsSet(_boundsLL, _boundsUR);
}

void SceneController::clearBounds()
{
  if (_bboxNode.valid())
  {
    _annoRoot->removeChild(_bboxNode.get());
    _bboxNode = 0L;
  }

  _boundsLL.set(0., 0.);
  _boundsUR.set(0., 0.);
}

void SceneController::setBounds(const osgEarth::GeoPoint& p1, const osgEarth::GeoPoint& p2)
{
  _boundsLL.set(osg::minimum(p1.x(), p2.x()), osg::minimum(p1.y(), p2.y()));
  _boundsUR.set(osg::maximum(p1.x(), p2.x()), osg::maximum(p1.y(), p2.y()));

  if (_annoRoot.valid())
  {
    //TODO: use correct coords here
    osgEarth::Symbology::Geometry* geom = new osgEarth::Symbology::Polygon();
    geom->push_back(_boundsLL.x(), _boundsLL.y());
    geom->push_back(_boundsUR.x(), _boundsLL.y());
    geom->push_back(_boundsUR.x(), _boundsUR.y());
    geom->push_back(_boundsLL.x(), _boundsUR.y());

    osgEarth::Features::Feature* feature = new osgEarth::Features::Feature(geom, _mapNode->getMapSRS()->getGeographicSRS(), _boundsStyle);

    if (!_bboxNode.valid())
    {
      _bboxNode = new osgEarth::Annotation::FeatureNode(_mapNode, feature);
      _bboxNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
      _annoRoot->addChild( _bboxNode.get() );
    }
    else
    {
      _bboxNode->setFeature(feature);
    }
  }
}