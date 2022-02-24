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
#ifndef OSGEARTH_IMGUI_VIEWPOINTS_GUI
#define OSGEARTH_IMGUI_VIEWPOINTS_GUI

#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>

namespace osgEarth {
    namespace GUI
    {
        using namespace osgEarth;
        using namespace osgEarth::Util;

        struct ViewpointsGUI : public BaseGUI
        {
            osg::observer_ptr<MapNode> _mapNode;
            float _duration;

            ViewpointsGUI() : 
                BaseGUI("Viewpoints"),
                _scannedForViewpoints(false) { }

            void draw(osg::RenderInfo& ri)
            {
                if (!isVisible())
                    return;

                if (!findNodeOrHide(_mapNode, ri))
                    return;

                if (!_scannedForViewpoints)
                {
                    Config conf = _mapNode->getConfig();
                    for (ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
                    {
                        if (i->key() == "viewpoints")
                        {
                            _duration = i->value("time", 1.0f);
                            const ConfigSet& children = i->children("viewpoint");
                            if (children.size() > 0)
                            {
                                for (ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i)
                                {
                                    _viewpoints.push_back(Viewpoint(*i));
                                }
                            }
                        }
                    }
                    _scannedForViewpoints = true;
                }

                ImGui::Begin(name(), visible());
                {
                    OE_SOFT_ASSERT_AND_RETURN(view(ri), void());

                    auto manip = dynamic_cast<EarthManipulator*>(view(ri)->getCameraManipulator());

                    if (!manip)
                        ImGui::TextColored(ImVec4(1,.9,.9,1), "(Viewpoints not active)");

                    if (_viewpoints.empty())
                        ImGui::TextColored(ImVec4(1, .9, .9, 1), "No viewpoints");

                    for (auto& vp : _viewpoints)
                    {
                        ImGui::PushID(&vp);
                        bool selected = false;
                        std::string name = vp.name().get();
                        if (name.empty())
                        {
                            name = "<no name>";
                        }
                        if (manip)
                        {
                            ImGui::Selectable(name.c_str(), &selected);
                            if (selected)
                                manip->setViewpoint(vp, _duration);
                        }
                        else
                        {
                            ImGui::TextColored(ImVec4(.5,.5,.5,1), name.c_str());
                        }
                        ImGui::PopID();
                        ImGui::Separator();
                    }
                }
                ImGui::End();
            }

            bool _scannedForViewpoints;
            std::vector<Viewpoint> _viewpoints;
        };
    }
}

#endif // OSGEARTH_IMGUI_VIEWPOINTS_GUI
