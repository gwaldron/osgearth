#pragma once

#include <osgEarth/ImGuiUtils>
#include <osgViewer/ViewerEventHandlers>

namespace osg {
class Camera;
}

class OsgImGuiHandler : public osgGA::GUIEventHandler
{
public:
    OsgImGuiHandler();

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;
    class RealizeOperation : public GlInitOperation
    {
        void operator()(osg::Object* object) override;
    };
protected:
    // Put your ImGui code inside this function
    virtual void drawUi(osg::RenderInfo&) = 0;
    friend class RealizeOperation;
private:
    static void init();

    void setCameraCallbacks(osg::Camera* camera);

    void newFrame(osg::RenderInfo& renderInfo);

    void render(osg::RenderInfo& renderInfo);

private:
    struct ImGuiNewFrameCallback;
    struct ImGuiRenderCallback;

    double time_;
    bool mousePressed_[3];
    float mouseWheel_;
    bool initialized_;
};
