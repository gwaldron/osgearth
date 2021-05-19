/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include <osgEarth/ImGui/ImGui>

using namespace osgEarth::GUI;

static osg::RefNodePath s_selectedNodePath;

osg::Node* osgEarth::GUI::getSelectedNode()
{
    if (s_selectedNodePath.empty())
    {
        return nullptr;
    }

    return s_selectedNodePath.back().get();
}

/*
void osgEarth::GUI::setSelectedNode(osg::Node* node)
{
    s_selectedNode = node;
}
*/



const osg::RefNodePath& osgEarth::GUI::getSelectedNodePath()
{
    return s_selectedNodePath;
}

void osgEarth::GUI::setSelectedNodePath(const osg::NodePath& nodePath)
{
    s_selectedNodePath.clear();

    for (auto itr = nodePath.begin(); itr != nodePath.end(); ++itr)
    {
        s_selectedNodePath.push_back(*itr);
    }
}