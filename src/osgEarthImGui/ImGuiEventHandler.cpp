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
#include "ImGuiEventHandler"
#include "imgui_internal.h"
#include "imgui_impl_opengl3.h"
#include <osgEarth/GLUtils>

using namespace osgEarth;

namespace
{
    struct PreDrawOp : public osg::Camera::DrawCallback
    {
        ImGuiEventHandler& _handler;
        PreDrawOp(ImGuiEventHandler& handler) : _handler(handler) {}
        void operator()(osg::RenderInfo& renderInfo) const override {
            _handler.newFrame(renderInfo);
        }
    };

    struct PostDrawOp : public osg::Camera::DrawCallback
    {
        ImGuiEventHandler& _handler;
        PostDrawOp(ImGuiEventHandler& handler) : _handler(handler) {}
        void operator()(osg::RenderInfo& renderInfo) const override {
            _handler.render(renderInfo);
        }
    };
}

ImGuiRealizeOperation::ImGuiRealizeOperation() :
    osg::Operation("ImGuiRealizeOperation", false)
{
    //nop
}

void
ImGuiRealizeOperation::operator()(osg::Object* object)
{
    osg::GraphicsContext* context = dynamic_cast<osg::GraphicsContext*>(object);
    if (context)
    {
        if (glewInit() != GLEW_OK)
        {
            OE_FATAL << "glewInit() failed" << std::endl;
        }
    }
}

void
ImGuiEventHandler::newFrame(osg::RenderInfo& renderInfo)
{
    if (_firstFrame)
    {
        ImGui::CreateContext();
        ImGui_ImplOpenGL3_Init();
        auto& io = ImGui::GetIO();
#ifdef IMGUI_HAS_DOCK
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif
    }

    ImGui_ImplOpenGL3_NewFrame();

    ImGuiIO& io = ImGui::GetIO();

    io.DisplaySize = ImVec2(
        renderInfo.getCurrentCamera()->getGraphicsContext()->getTraits()->width,
        renderInfo.getCurrentCamera()->getGraphicsContext()->getTraits()->height);

    double currentTime = renderInfo.getView()->getFrameStamp()->getSimulationTime();
    io.DeltaTime = currentTime - _time + 0.0000001;
    _time = currentTime;

    if (_firstFrame)
    {
        installSettingsHandler();
        _firstFrame = false;
    }

    ImGui::NewFrame();
}

namespace
{
    static ImGuiEventHandler* s_guiHandler = nullptr;
}

void ImGuiEventHandler::handleReadSetting(
    ImGuiContext* ctx, ImGuiSettingsHandler* handler, void* entry, const char* line)
{
    std::vector<std::string> tokens;
    StringTokenizer(std::string(line), tokens, "=");
    if (tokens.size() == 2)
    {
        s_guiHandler->load(entry, tokens[0], tokens[1]);
    }
}

void* ImGuiEventHandler::handleStartEntry(
    ImGuiContext* ctx, ImGuiSettingsHandler* handler, const char* name)
{
    return s_guiHandler->findByName(name);
}

void ImGuiEventHandler::handleWriteSettings(
    ImGuiContext* ctx, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out_buf)
{
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

void ImGuiEventHandler::installSettingsHandler()
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

void ImGuiEventHandler::render(osg::RenderInfo& ri)
{
    auto camera = ri.getCurrentCamera();
    auto viewport = camera->getViewport();

    if (_show)
    {
        constexpr ImGuiDockNodeFlags dockspace_flags =
            ImGuiDockNodeFlags_NoDockingInCentralNode | ImGuiDockNodeFlags_PassthruCentralNode;

        auto dockSpaceId = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), dockspace_flags);

        glDisable(GL_MULTISAMPLE);

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

    if (_autoAdjustProjectionMatrix)
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

bool ImGuiEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (!_initialized)
    {
        auto view = aa.asView();
        if (view)
        {
            view->getCamera()->setPreDrawCallback(new PreDrawOp(*this));
            view->getCamera()->setPostDrawCallback(new PostDrawOp(*this));
            _initialized = true;
            return false;
        }
    }
    if (_firstFrame)
    {
        // imgui is not initialized yet
        return false;
    }

    if (ea.getHandled())
    {
        return false;
    }

    ImGuiIO& io = ImGui::GetIO();

    switch (ea.getEventType())
    {
    case osgGA::GUIEventAdapter::KEYDOWN:
    case osgGA::GUIEventAdapter::KEYUP:
    {
        const bool isKeyDown = ea.getEventType() == ea.KEYDOWN;
        const int c = ea.getKey();

        if (io.WantCaptureKeyboard || c == ea.KEY_Return)
        {
            // first apply any modifiers:
            io.AddKeyEvent(ImGuiMod_Ctrl, (ea.getModKeyMask() & ea.MODKEY_CTRL) != 0);
            io.AddKeyEvent(ImGuiMod_Shift, (ea.getModKeyMask() & ea.MODKEY_SHIFT) != 0);
            io.AddKeyEvent(ImGuiMod_Alt, (ea.getModKeyMask() & ea.MODKEY_ALT) != 0);
            io.AddKeyEvent(ImGuiMod_Super, (ea.getModKeyMask() & ea.MODKEY_SUPER) != 0);

            // map the OSG key code to the ImGui key code and send to imgui:
            auto imgui_key = convertKey(c);
            io.AddKeyEvent(imgui_key, isKeyDown);

            // Send any raw ASCII characters to imgui as input
            if (isKeyDown && c >= 32 && c < 512)
            {
                io.AddInputCharacter((unsigned)c);
            }
        }
        else
        {
            // toggle imgui
            if (isKeyDown && c == 'y')
            {
                _show = !_show;
                return true;
            }
        }

        return io.WantCaptureKeyboard;
    }

    case (osgGA::GUIEventAdapter::PUSH):
    {
        if (io.WantCaptureMouse)
        {
            auto imgui_button = convertMouseButton(ea.getButtonMask());
            io.AddMousePosEvent(ea.getX(), io.DisplaySize.y - ea.getY());
            io.AddMouseButtonEvent(imgui_button, true); // true = press
        }
        return io.WantCaptureMouse;
    }

    case (osgGA::GUIEventAdapter::RELEASE):
    {
        if (io.WantCaptureMouse)
        {
            io.AddMousePosEvent(ea.getX(), io.DisplaySize.y - ea.getY());
        }
        auto imgui_button = convertMouseButton(ea.getButtonMask());
        io.AddMouseButtonEvent(imgui_button, false); // false = release

        return io.WantCaptureMouse;
    }

    case (osgGA::GUIEventAdapter::DRAG):
    case (osgGA::GUIEventAdapter::MOVE):
    {
        io.AddMousePosEvent(ea.getX(), io.DisplaySize.y - ea.getY());
        return io.WantCaptureMouse;
    }

    case (osgGA::GUIEventAdapter::SCROLL):
    {
        auto scrolling = ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP ? 1.0 : -1.0;
        io.AddMouseWheelEvent(0.0, io.MouseWheel += scrolling);
        return io.WantCaptureMouse;
    }
        }

    return false;
}

ImGuiKey
ImGuiEventHandler::convertKey(int c)
{
    // If you are holding CTRL, OSG remaps A-Z to 1-26. Undo that.
    if (c >= 1 && c <= 26)
    {
        return (ImGuiKey)((int)ImGuiKey_A + c - 1);
    }

    if (c >= osgGA::GUIEventAdapter::KEY_0 && c <= osgGA::GUIEventAdapter::KEY_0)
    {
        return (ImGuiKey)((int)ImGuiKey_0 + c - osgGA::GUIEventAdapter::KEY_0);
    }

    if (c >= osgGA::GUIEventAdapter::KEY_A && c <= osgGA::GUIEventAdapter::KEY_Z)
    {
        return (ImGuiKey)((int)ImGuiKey_A + c - osgGA::GUIEventAdapter::KEY_A);
    }

    switch (c)
    {
    case osgGA::GUIEventAdapter::KEY_Tab:
        return ImGuiKey_Tab;
    case osgGA::GUIEventAdapter::KEY_Left:
        return ImGuiKey_LeftArrow;
    case osgGA::GUIEventAdapter::KEY_Right:
        return ImGuiKey_RightArrow;
    case osgGA::GUIEventAdapter::KEY_Up:
        return ImGuiKey_UpArrow;
    case osgGA::GUIEventAdapter::KEY_Down:
        return ImGuiKey_DownArrow;
    case osgGA::GUIEventAdapter::KEY_Page_Up:
        return ImGuiKey_PageUp;
    case osgGA::GUIEventAdapter::KEY_Page_Down:
        return ImGuiKey_PageDown;
    case osgGA::GUIEventAdapter::KEY_Home:
        return ImGuiKey_Home;
    case osgGA::GUIEventAdapter::KEY_End:
        return ImGuiKey_End;
    case osgGA::GUIEventAdapter::KEY_Delete:
        return ImGuiKey_Delete;
    case osgGA::GUIEventAdapter::KEY_BackSpace:
        return ImGuiKey_Backspace;
    case osgGA::GUIEventAdapter::KEY_Return:
        return ImGuiKey_Enter;
    case osgGA::GUIEventAdapter::KEY_Escape:
        return ImGuiKey_Escape;                
    case osgGA::GUIEventAdapter::KEY_Space:
        return ImGuiKey_Space;
    }

    return ImGuiKey_None;
}

ImGuiButtonFlags
ImGuiEventHandler::convertMouseButton(int m)
{
    ImGuiButtonFlags flags = 0;
    if (m & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
        flags |= ImGuiMouseButton_Left;
    if (m & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        flags |= ImGuiMouseButton_Right;
    if (m & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
        flags |= ImGuiMouseButton_Middle;
    return flags;
}
