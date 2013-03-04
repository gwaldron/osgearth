/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
//#include <osgEarth/StringUtils>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/SkyNode>



using namespace PackageQt;

#define LC "[SceneController] "

SceneController::SceneController(osg::Group* root, osgViewer::View* view)
: _root(root), _view(view)
{
  if (_root.valid() && _view.valid())
  {
    // install a canvas for any UI controls we plan to create
    _canvas = osgEarth::Util::Controls::ControlCanvas::get(_view, false);

    _controlContainer = _canvas->addControl( new osgEarth::Util::Controls::VBox() );
    _controlContainer->setBackColor( osgEarth::Util::Controls::Color(osgEarth::Util::Controls::Color::Black, 0.8) );
    _controlContainer->setHorizAlign( osgEarth::Util::Controls::Control::ALIGN_LEFT );
    _controlContainer->setVertAlign( osgEarth::Util::Controls::Control::ALIGN_BOTTOM );

    root->addChild(_canvas);
  }
}

osg::Node* SceneController::loadEarthFile(const std::string& url)
{
  if (!_root.valid() || !_view.valid())
    return 0L;

  if (_earthNode.valid())
    _root->removeChild(_earthNode);

  if (_sky.valid())
  {
    _root->removeChild(_sky);
    _sky = 0L;
  }

  if (_controlContainer.valid())
    _controlContainer->clearControls();

  _earthNode = osgDB::readNodeFile( url );

  if (_earthNode.valid())
  {
    _mapNode = osgEarth::MapNode::findMapNode( _earthNode );
    if (_mapNode.valid())
    {
      _map = _mapNode->getMap();

      const osgEarth::Config& externals = _mapNode->externalConfig();

      if (_map->isGeocentric())
      {
        // Sky model.
        osgEarth::Config skyConf = externals.child("sky");

        double hours = skyConf.value("hours", 12.0);

        _sky = new osgEarth::Util::SkyNode(_map);
        _sky->setAutoAmbience( true );
        _sky->setDateTime(2011, 3, 6, hours);
        _sky->attach(_view);

        _root->addChild(_sky);

        if (_controlContainer.valid())
        {
          osgEarth::Util::Controls::Control* c = osgEarth::Util::SkyControlFactory().create(_sky, _view);
          if ( c )
              _controlContainer->addControl( c );
        }
      }
    }

    _root->addChild(_earthNode);

    //Create new EarthManipulator
    _view->setCameraManipulator(new osgEarth::Util::EarthManipulator());
  }

  return _earthNode.get();
}
