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

#ifndef TILER_TOOL_SCENECONTROLLER_H
#define TILER_TOOL_SCENECONTROLLER_H 1

#include <vector>

#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/SkyNode>

namespace PackageQt
{
  class SceneController
  {
  public:

    SceneController(osg::Group* root, osgViewer::View* view, const std::string& url="");

    osg::Node* loadEarthFile(const std::string& url);

    osg::Node* earthNode() { return _earthNode.get(); }
    osgEarth::MapNode* mapNode() { return _mapNode.get(); }
    osgEarth::Map* map() { return _map.get(); }

  private:

    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osgViewer::View> _view;
    osg::ref_ptr<osgEarth::Util::Controls::ControlCanvas> _canvas;
    osg::ref_ptr<osgEarth::Util::Controls::Container> _controlContainer;

    osg::ref_ptr<osg::Node> _earthNode;
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osg::ref_ptr<osgEarth::Map> _map;
    //osg::ref_ptr<osgEarth::Util::SkyNode> _sky;
  };
}

#endif //TILER_TOOL_SCENECONTROLLER_H