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
#include <osgEarth/GLUtils>

#include "imgui.h"
#include "backends/imgui_impl_opengl3.h"

using namespace osgEarth::GUI;


/***********************************************************************************/
using namespace osgEarth::GUI;


void OsgImGuiHandler::RealizeOperation::operator()(osg::Object* object)
{
    GlewInitOperation::operator()(object);
}

struct OsgImGuiHandler::ImGuiNewFrameCallback : public osg::Camera::DrawCallback
{
    ImGuiNewFrameCallback(OsgImGuiHandler& handler)
        : handler_(handler)
    {
    }

    void operator()(osg::RenderInfo& renderInfo) const override
    {
        handler_.newFrame(renderInfo);
    }

private:
    OsgImGuiHandler& handler_;
};

struct OsgImGuiHandler::ImGuiRenderCallback : public osg::Camera::DrawCallback
{
    ImGuiRenderCallback(OsgImGuiHandler& handler) :
        _handler(handler)
    {
    }

    void operator()(osg::RenderInfo& renderInfo) const override
    {
        _handler.render(renderInfo);
    }

private:
    OsgImGuiHandler& _handler;
};

OsgImGuiHandler::OsgImGuiHandler() :
    time_(0.0f),
    mousePressed_{ false },
    mouseWheel_(0.0f),
    initialized_(false),
    firstFrame_(true),
    show_(true)
{
    //nop
}

/**
 * Imporant Note: Dear ImGui expects the control Keys indices not to be
 * greater thant 511. It actually uses an array of 512 elements. However,
 * OSG has indices greater than that. So here I do a conversion for special
 * keys between ImGui and OSG.
 */
static int ConvertFromOSGKey(int key)
{
    using KEY = osgGA::GUIEventAdapter::KeySymbol;

    switch (key)
    {
    case KEY::KEY_Tab:
        return ImGuiKey_Tab;
    case KEY::KEY_Left:
        return ImGuiKey_LeftArrow;
    case KEY::KEY_Right:
        return ImGuiKey_RightArrow;
    case KEY::KEY_Up:
        return ImGuiKey_UpArrow;
    case KEY::KEY_Down:
        return ImGuiKey_DownArrow;
    case KEY::KEY_Page_Up:
        return ImGuiKey_PageUp;
    case KEY::KEY_Page_Down:
        return ImGuiKey_PageDown;
    case KEY::KEY_Home:
        return ImGuiKey_Home;
    case KEY::KEY_End:
        return ImGuiKey_End;
    case KEY::KEY_Delete:
        return ImGuiKey_Delete;
    case KEY::KEY_BackSpace:
        return ImGuiKey_Backspace;
    case KEY::KEY_Return:
        return ImGuiKey_Enter;
    case KEY::KEY_Escape:
        return ImGuiKey_Escape;
    case 22:
        return osgGA::GUIEventAdapter::KeySymbol::KEY_V;
    case 3:
        return osgGA::GUIEventAdapter::KeySymbol::KEY_C;
    default: // Not found
        return key;
    }
}

void OsgImGuiHandler::init()
{
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array.
    io.KeyMap[ImGuiKey_Tab] = ImGuiKey_Tab;
    io.KeyMap[ImGuiKey_LeftArrow] = ImGuiKey_LeftArrow;
    io.KeyMap[ImGuiKey_RightArrow] = ImGuiKey_RightArrow;
    io.KeyMap[ImGuiKey_UpArrow] = ImGuiKey_UpArrow;
    io.KeyMap[ImGuiKey_DownArrow] = ImGuiKey_DownArrow;
    io.KeyMap[ImGuiKey_PageUp] = ImGuiKey_PageUp;
    io.KeyMap[ImGuiKey_PageDown] = ImGuiKey_PageDown;
    io.KeyMap[ImGuiKey_Home] = ImGuiKey_Home;
    io.KeyMap[ImGuiKey_End] = ImGuiKey_End;
    io.KeyMap[ImGuiKey_Delete] = ImGuiKey_Delete;
    io.KeyMap[ImGuiKey_Backspace] = ImGuiKey_Backspace;
    io.KeyMap[ImGuiKey_Enter] = ImGuiKey_Enter;
    io.KeyMap[ImGuiKey_Escape] = ImGuiKey_Escape;
    io.KeyMap[ImGuiKey_A] = osgGA::GUIEventAdapter::KeySymbol::KEY_A;
    io.KeyMap[ImGuiKey_C] = osgGA::GUIEventAdapter::KeySymbol::KEY_C;
    io.KeyMap[ImGuiKey_V] = osgGA::GUIEventAdapter::KeySymbol::KEY_V;
    io.KeyMap[ImGuiKey_X] = osgGA::GUIEventAdapter::KeySymbol::KEY_X;
    io.KeyMap[ImGuiKey_Y] = osgGA::GUIEventAdapter::KeySymbol::KEY_Y;
    io.KeyMap[ImGuiKey_Z] = osgGA::GUIEventAdapter::KeySymbol::KEY_Z;


    ImGui_ImplOpenGL3_Init();

    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

void OsgImGuiHandler::setCameraCallbacks(osg::Camera* camera)
{
    camera->setPreDrawCallback(new ImGuiNewFrameCallback(*this));
    camera->setPostDrawCallback(new ImGuiRenderCallback(*this));
}

void OsgImGuiHandler::newFrame(osg::RenderInfo& renderInfo)
{
    if (firstFrame_ == true)
    {
        init();
    }

    ImGui_ImplOpenGL3_NewFrame();

    ImGuiIO& io = ImGui::GetIO();

    io.DisplaySize = ImVec2(renderInfo.getCurrentCamera()->getGraphicsContext()->getTraits()->width, renderInfo.getCurrentCamera()->getGraphicsContext()->getTraits()->height);

    double currentTime = renderInfo.getView()->getFrameStamp()->getSimulationTime();
    io.DeltaTime = currentTime - time_ + 0.0000001;
    time_ = currentTime;

    for (int i = 0; i < 3; i++)
    {
        io.MouseDown[i] = mousePressed_[i];
    }

    for (int i = 0; i < 3; i++)
    {
        io.MouseDoubleClicked[i] = mouseDoubleClicked_[i];
    }

    io.MouseWheel = mouseWheel_;
    mouseWheel_ = 0.0f;

    if (firstFrame_ == true)
    {
        installSettingsHandler();
        firstFrame_ = false;        
    }

    ImGui::NewFrame();
}

namespace
{
    static OsgImGuiHandler* s_guiHandler = nullptr;
}

void OsgImGuiHandler::handleReadSetting(
    ImGuiContext* ctx, ImGuiSettingsHandler* handler, void* entry, const char* line)
{
    std::vector<std::string> tokens;
    StringTokenizer(std::string(line), tokens, "=");
    if (tokens.size() == 2)
    {
        s_guiHandler->load(entry, tokens[0], tokens[1]);
    }
}

void* OsgImGuiHandler::handleStartEntry(
    ImGuiContext* ctx, ImGuiSettingsHandler* handler, const char* name)
{
    return s_guiHandler->findByName(name);
}

void OsgImGuiHandler::handleWriteSettings(
    ImGuiContext* ctx, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out_buf)
{
    OE_DEBUG << "Writing ini settings..." << std::endl;
    Config sections;
    s_guiHandler->save(sections);
    for (auto& section : sections.children())
    {
        std::string title = "[osgEarth][" + section.key() + "]\n";
        out_buf->append(title.c_str());

        for (auto& var : section.children())
        {
            std::string line(var.key() + "=" + var.value() + '\n');
            out_buf->append(line.c_str());
        }
    }
}

void OsgImGuiHandler::installSettingsHandler()
{
    OE_HARD_ASSERT(ImGui::GetCurrentContext() != nullptr);
    s_guiHandler = this;
    ImGuiSettingsHandler s;
    s.TypeName = "osgEarth";
    s.TypeHash = ImHashStr(s.TypeName);
    s.ReadOpenFn = handleStartEntry;
    s.ReadLineFn = handleReadSetting;
    s.WriteAllFn = handleWriteSettings;
    ImGui::GetCurrentContext()->SettingsHandlers.push_back(s);
}

void OsgImGuiHandler::render(osg::RenderInfo& ri)
{
    auto camera = ri.getCurrentCamera();
    auto viewport = camera->getViewport();

    if (show_)
    {
        constexpr ImGuiDockNodeFlags dockspace_flags =
            ImGuiDockNodeFlags_NoDockingInCentralNode | ImGuiDockNodeFlags_PassthruCentralNode;

        auto dockSpaceId = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), dockspace_flags);

        draw(ri);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        auto centralNode = ImGui::DockBuilderGetCentralNode(dockSpaceId);

        auto io = ImGui::GetIO();
        viewport->x() = centralNode->Pos.x;
        viewport->y() = io.DisplaySize.y - centralNode->Size.y - centralNode->Pos.y;
        viewport->width() = centralNode->Size.x;
        viewport->height() = centralNode->Size.y;
    }
    else
    {
        viewport->x() = 0;
        viewport->y() = 0;
        viewport->width() = camera->getGraphicsContext()->getTraits()->width;
        viewport->height() = camera->getGraphicsContext()->getTraits()->height;
    }

    if (autoAdjustProjectionMatrix)
    {
        const osg::Matrixd& proj = camera->getProjectionMatrix();
        bool isOrtho = osg::equivalent(proj(3, 3), 1.0);
        if (!isOrtho)
        {
            double fovy, ar, znear, zfar;
            camera->getProjectionMatrixAsPerspective(fovy, ar, znear, zfar);
            camera->setProjectionMatrixAsPerspective(fovy, viewport->width() / viewport->height(), znear, zfar);
        }
        else
        {
            double left, right, bottom, top, znear, zfar;
            camera->getProjectionMatrixAsOrtho(left, right, bottom, top, znear, zfar);
            camera->setProjectionMatrixAsOrtho(viewport->x(), viewport->x() + viewport->width(), viewport->y(), viewport->y() + viewport->height(), znear, zfar);
        }
    }
}

bool OsgImGuiHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (!initialized_)
    {
        auto view = aa.asView();
        if (view)
        {
            setCameraCallbacks(view->getCamera());
            initialized_ = true;
            return false;
        }
    }

    ImGuiIO& io = ImGui::GetIO();
    const bool wantCaptureMouse = io.WantCaptureMouse;
    const bool wantCaptureKeyboard = io.WantCaptureKeyboard;

    switch (ea.getEventType())
    {
    case osgGA::GUIEventAdapter::KEYDOWN:
    case osgGA::GUIEventAdapter::KEYUP:
    {
        const bool isKeyDown = ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN;
        const int c = ea.getKey();

        if (wantCaptureKeyboard)
        {
            // Always update the mod key status.
            io.KeyCtrl = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL;
            io.KeyShift = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT;
            io.KeyAlt = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_ALT;
            io.KeySuper = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SUPER;


            const int imgui_key = ConvertFromOSGKey(c);
            if (imgui_key > 0 && imgui_key < 512)
            {
                //assert((imgui_key >= 0 && imgui_key < 512) && "ImGui KeysMap is an array of 512");
                io.KeysDown[imgui_key] = isKeyDown;
            }

            // Not sure this < 512 is correct here....
            if (isKeyDown && imgui_key >= 32 && imgui_key < 512)
            {
                io.AddInputCharacter((unsigned int)c);
            }
        }
        else
        {
            if (isKeyDown && c == 'y')
            {
                show_ = !show_;
                return true;
            }
        }

        return wantCaptureKeyboard;
    }
    case (osgGA::GUIEventAdapter::RELEASE):
    {
        io.MousePos = ImVec2(ea.getX(), io.DisplaySize.y - ea.getY());
        mousePressed_[0] = ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
        mousePressed_[1] = ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON;
        mousePressed_[2] = ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;

        mouseDoubleClicked_[0] = ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
        mouseDoubleClicked_[1] = ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON;
        mouseDoubleClicked_[2] = ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
        return wantCaptureMouse;
    }
    case (osgGA::GUIEventAdapter::PUSH):
    {
        io.MousePos = ImVec2(ea.getX(), io.DisplaySize.y - ea.getY());
        mousePressed_[0] = ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
        mousePressed_[1] = ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON;
        mousePressed_[2] = ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
        return wantCaptureMouse;
    }
    case (osgGA::GUIEventAdapter::DOUBLECLICK):
    {
        io.MousePos = ImVec2(ea.getX(), io.DisplaySize.y - ea.getY());
        mouseDoubleClicked_[0] = ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
        mouseDoubleClicked_[1] = ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON;
        mouseDoubleClicked_[2] = ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
        return wantCaptureMouse;
    }
    case (osgGA::GUIEventAdapter::DRAG):
    case (osgGA::GUIEventAdapter::MOVE):
    {
        io.MousePos = ImVec2(ea.getX(), io.DisplaySize.y - ea.getY());
        return wantCaptureMouse;
    }
    case (osgGA::GUIEventAdapter::SCROLL):
    {
        mouseWheel_ = ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP ? 1.0 : -1.0;
        return wantCaptureMouse;
    }
    default:
    {
        return false;
    }
    }

    return false;
}

/***********************************************************************************/

static osg::RefNodePath s_selectedNodePath;

osg::Node* osgEarth::GUI::getSelectedNode()
{
    if (s_selectedNodePath.empty())
    {
        return nullptr;
    }

    return s_selectedNodePath.back().get();
}

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



