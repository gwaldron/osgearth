/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarth/ImGui/ImGui>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgViewer/Viewer>
#include <osgViewer/Renderer>
#include <osgDB/ReadFile>

#define LC "[imgui] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::GUI;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;
    return 0;
}

std::vector< osg::ref_ptr< EmbeddedViewer > > embeddedViewers;

class RTTViewerGUI : public BaseGUI
{
public:
    RTTViewerGUI(const std::string& name, EmbeddedViewer* view) :
        BaseGUI(name.c_str()),
        _view(view)
    {
    }

    void load(const Config& conf) override
    {
    }

    void save(Config& conf) override
    {
    }

protected:
    void draw(osg::RenderInfo& renderInfo) override
    {
        if (!isVisible()) return;

        _view->frame();

        ImGuiIO& io = ImGui::GetIO();

        ImGui::Begin(name(), visible());        
        auto size = ImVec2(_view->_colorTexture->getTextureWidth(), _view->_colorTexture->getTextureWidth());
        //auto size = ImGui::GetWindowSize();
        _view->setSize(size.x, size.y);

        ImVec2 screen_pos = ImGui::GetCursorScreenPos();        
        _view->origin.set(screen_pos.x, io.DisplaySize.y - screen_pos.y - size.y);

        auto texture = _view->_colorTexture.get();
        if (texture)
        {
            // Get the context id
            const unsigned int currentContextID = renderInfo.getState()->getContextID();
            const unsigned int contextID = _view->_graphicsWindowEmbedded->getState()->getContextID();

            unsigned int w = size.x;
            unsigned int h = size.y;

            // Apply the texture
            texture->apply(*renderInfo.getState());

            // Get the TextureObject.
            osg::Texture::TextureObject* textureObject = texture->getTextureObject(contextID);
            if (textureObject)
            {
                ImGui::Image((void*)(intptr_t)textureObject->_id, ImVec2(w, h), ImVec2(0, 1), ImVec2(1, 0), ImVec4(1, 1, 1, 1), ImVec4(1, 1, 0, 1));        
                if (ImGui::IsItemHovered())
                {
                    setFocusedView(_view.get());
                }
            }
        }

        ImGui::End();
    }
    osg::ref_ptr< EmbeddedViewer > _view;
};

class EmbeddedViewersGUI : public BaseGUI
{
public:
    float scale = 0.1f;

    EmbeddedViewersGUI() :
        BaseGUI("Embedded Viewers")
    {
    }

    void load(const Config& conf) override
    {
    }

    void save(Config& conf) override
    {
    }

protected:
    void draw(osg::RenderInfo& renderInfo) override
    {
        if (!isVisible()) return;

        ImGui::Begin(name(), visible());

        ImGui::SliderFloat("Scale", &scale, 0.1f, 1.0f);

        if (ImGui::Button("Add View"))
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(renderInfo.getView());

            EmbeddedViewer* rttViewer = new EmbeddedViewer(view->getSceneData(), view->getCamera()->getGraphicsContext());
            Viewpoint vp = static_cast<EarthManipulator*>(view->getCameraManipulator())->getViewpoint();
            static_cast<EarthManipulator*>(rttViewer->getCameraManipulator())->setViewpoint(vp);

            embeddedViewers.push_back(rttViewer);
        }        

        for (auto &view : embeddedViewers)
        {
            view->frame();

            ImGuiIO& io = ImGui::GetIO();

            auto size = ImVec2(scale * view->_colorTexture->getTextureWidth(), scale * view->_colorTexture->getTextureWidth());

            auto texture = view->_colorTexture.get();
            if (texture)
            {
                // Get the context id
                const unsigned int contextID = renderInfo.getState()->getContextID();

                unsigned int w = size.x;
                unsigned int h = size.y;

                // Apply the texture
                texture->apply(*renderInfo.getState());

                // Get the TextureObject.
                osg::Texture::TextureObject* textureObject = texture->getTextureObject(contextID);
                if (textureObject)
                {
                    ImGui::Image((void*)(intptr_t)textureObject->_id, ImVec2(w, h), ImVec2(0, 1), ImVec2(1, 0), ImVec4(1, 1, 1, 1), ImVec4(1, 1, 0, 1));
                    ImGui::SameLine();
                }
            }
        }

        ImGui::End();
    }  
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    if (arguments.read("--help"))
        return usage(argv[0]);

    osgEarth::initialize();


    osgViewer::Viewer viewer(arguments);
    viewer.setUpViewOnSingleScreen(0);
    viewer.setThreadingModel(viewer.SingleThreaded);
    viewer.setCameraManipulator(new EarthManipulator(arguments));

    // Call this to enable ImGui rendering.
    // If you use the MapNodeHelper, call this first.
    viewer.setRealizeOperation(new GUI::ApplicationGUI::RealizeOperation);

    std::vector< osg::ref_ptr< osg::Node > > nodes;
    std::string embed;
    while (arguments.read("--embed", embed))
    {
        auto node = osgDB::readNodeFile(embed);
        osgEarth::Registry::shaderGenerator().run(node);
        nodes.push_back(node);
    }

    //osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFiles(arguments);
    osg::ref_ptr<osg::Node> node = MapNodeHelper().loadWithoutControls(arguments, &viewer);
    if (node.valid())
    {
        auto guis = new GUI::ApplicationGUI(true);

        for (unsigned int i = 0; i < nodes.size(); ++i)
        {
            EmbeddedViewer* view = new EmbeddedViewer(nodes[i].get(), viewer.getCamera()->getGraphicsContext());
            std::stringstream buf;
            buf << "View " << i;
            guis->add(new RTTViewerGUI(buf.str(), view));
            embeddedViewers.push_back(view);
        }

        guis->add(new EmbeddedViewersGUI());

        viewer.getEventHandlers().push_front(guis);

        viewer.setSceneData(node.get());
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}