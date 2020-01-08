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

#include "GL/glew.h"

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/NetworkMonitor>
#include <iostream>
#include <osgEarth/Metrics>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

#include "imgui.h"
#include "imgui_internal.h"

#include "OsgImGuiHandler.hpp"

struct Span
{
    Span(const std::string& _name) :
        name(_name),
        isComplete(false),
        startTime(osg::Timer::instance()->tick())
    {
    }

    Span() :
        isComplete(false),
        startTime(osg::Timer::instance()->tick())
    {
    }

    double getDuration()
    {
        if (isComplete)
        {
            return osg::Timer::instance()->delta_m(startTime, endTime);
        }
        return osg::Timer::instance()->delta_m(startTime, osg::Timer::instance()->tick());
    }

    bool isComplete;
    std::string name;
    osg::Timer_t startTime;
    osg::Timer_t endTime;
};

typedef std::map< int, std::vector< Span > > LevelSpans;

struct ThreadInfo
{
    ThreadInfo() :
        _level(-1)
    {
    }

    int _level;
    LevelSpans _levelSpans;
    std::stack< Span* > _spanStack;
};

class ImGuiMetricsBackend : public MetricsBackend
{
public:
    ImGuiMetricsBackend()
    {
    }

    virtual void begin(const std::string& name, const Config& args = Config())
    {
        Threading::ScopedMutexLock lock(_mutex);
        ThreadInfo& threadInfo = _threadInfo[osgEarth::Threading::getCurrentThreadId()];
        threadInfo._level++;
        std::vector<Span>& spans = threadInfo._levelSpans[threadInfo._level];
        Span span;
        span.name = name;
        spans.push_back(span);
        Span* activeSpan = &spans.back();
        threadInfo._spanStack.push(activeSpan);
    }

    virtual void end(const std::string& name, const Config& args = Config())
    {
        Threading::ScopedMutexLock lock(_mutex);
        ThreadInfo& threadInfo = _threadInfo[osgEarth::Threading::getCurrentThreadId()];
        if (!threadInfo._spanStack.empty())
        {
            Span* currentSpan = threadInfo._spanStack.top();
            if (currentSpan->name != name) {
                OE_WARN << "Last pushed metric is " << currentSpan->name << " but trying to end " << name << " on thread " << osgEarth::Threading::getCurrentThreadId() << std::endl;
            }
            currentSpan->endTime = osg::Timer::instance()->tick();
            currentSpan->isComplete = true;
            threadInfo._spanStack.pop();
            threadInfo._level--;
        }
    }

    virtual void counter(const std::string& graph,
        const std::string& name0, double value0,
        const std::string& name1, double value1,
        const std::string& name2, double value2)
    {
        //Threading::ScopedMutexLock lock(_mutex);
        //OE_NOTICE << "Counter" << std::endl;
    }

    void getThreadInfo(std::map< unsigned int, ThreadInfo>& out)
    {
        Threading::ScopedMutexLock lock(_mutex);
        out = _threadInfo;
    }

private:
    Threading::Mutex _mutex;
    std::map< unsigned int, ThreadInfo> _threadInfo;
};


class GlewInitOperation : public osg::Operation
{
public:
    GlewInitOperation()
        : osg::Operation("GlewInitCallback", false)
    {
    }

    void operator()(osg::Object* object) override
    {
        osg::GraphicsContext* context = dynamic_cast<osg::GraphicsContext*>(object);
        if (!context)
            return;

        if (glewInit() != GLEW_OK)
        {
            std::cout << "glewInit() failed\n";
        }
    }
};

class ImGuiDemo : public OsgImGuiHandler
{
public:
    ImGuiDemo() :
        timeScale(100.0f)
    {
    }
protected:
    void networkMonitor()
    {
        ImGui::Begin("Network Monitor");
        NetworkMonitor::Requests requests;
        NetworkMonitor::getRequests(requests);

        ImGui::Columns(3, "requests");
        ImGui::Separator();
        ImGui::Text("Path"); ImGui::NextColumn();
        ImGui::Text("Duration (ms)"); ImGui::NextColumn();
        ImGui::Text("Status"); ImGui::NextColumn();
        ImGui::Separator();
        for (auto itr = requests.begin(); itr != requests.end(); ++itr)
        {
            ImVec4 color = itr->second.isComplete ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 1.0, 1.0, 1.0);
            ImGui::TextColored(color, itr->second.uri.c_str());  ImGui::NextColumn();
            ImGui::Text("%f", itr->second.getDuration()); ImGui::NextColumn();
            ImGui::Text(itr->second.status.c_str()); ImGui::NextColumn();
        }
        ImGui::Columns(1);
        ImGui::Separator();


        ImGui::End();
    }

    void profiler()
    {
        bool showCustomDrawing = true;

        ImGuiMetricsBackend* metrics = dynamic_cast<ImGuiMetricsBackend*>(Metrics::getMetricsBackend());
        std::map< unsigned int, ThreadInfo > threadInfo;
        metrics->getThreadInfo(threadInfo);

        if (metrics)
        {
            ImGui::Begin("Profiler", &showCustomDrawing, ImGuiWindowFlags_HorizontalScrollbar);
            ImGui::SliderFloat("Scale", &timeScale, 0.1, 500.0);

            ImGui::BeginChild("scrolling", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);

            // Render each thread
            for (auto threadItr = threadInfo.begin(); threadItr != threadInfo.end(); ++threadItr)
            {
                std::stringstream buf;
                buf << "Thread " << threadItr->first;
            
                ImGui::Text("Thread %d", threadItr->first);                

                // Get all the levels that are in the thread.
                std::vector< unsigned int > levels;
                for (auto itr = threadItr->second._levelSpans.begin(); itr != threadItr->second._levelSpans.end(); ++itr)
                {
                    levels.push_back(itr->first);
                }

                float x = 0.0f;
                for (unsigned int i = 0; i < levels.size(); ++i)
                {                    
                    unsigned int level = levels[i];
                    ImGui::Text("Level %d", level);

                    //OE_NOTICE << "Rendering level " << level << std::endl;
                    std::vector< Span > &spans = threadItr->second._levelSpans[level];
                    x = 0.0f;
                    for (auto itr = spans.begin(); itr != spans.end(); ++itr)
                    {
                        double startTime = osg::Timer::instance()->delta_m(osg::Timer::instance()->getStartTick(), itr->startTime);
                        x = startTime * timeScale;
                        ImGui::SameLine(x);
                        //if (itr->isComplete)
                        {
                            float width = itr->getDuration() * timeScale;
                            //ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                            if (level == 0)
                            {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
                            }
                            else if (level == 1)
                            {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
                            }
                            else if (level == 2)
                            {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
                            }
                                                        
                            ImGui::ButtonEx(itr->name.c_str(), ImVec2(width, 0.0));
                            if (ImGui::IsItemHovered())
                                ImGui::SetTooltip("%s %fms", itr->name.c_str(), itr->getDuration());
                            ImGui::PopStyleColor();                        
                        }
                    }                    
                    ImGui::NewLine();
                }                
            }
            ImGui::EndChild();
            ImGui::End();
        }
    }

    void drawUi() override
    {
        // ImGui code goes here...
        ImGui::ShowDemoWindow();


        /*
        const ImVec2 p = ImGui::GetCursorScreenPos();
        float x = p.x;
        float y = p.y;
        float sz = 50.0f;
        float margin = 10.0f;
        ImVec4 colf = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
        for (unsigned int i = 0; i < 100; i++)
        {
            ImGui::RenderFrame(ImVec2(x, y), ImVec2(x + sz, y + sz), ImGui::GetColorU32(colf));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            ImGui::RenderTextClipped(ImVec2(x, y), ImVec2(x + sz, y + sz), "Hello", NULL, NULL);
            ImGui::PopStyleColor();
            x += (sz + margin);
        }
        ImGui::EndChild();
        */

        // Draw the first 
        /*
        float width = 100.0f;
        float margin = 30.0f;
        float x = 0.0f;
        for (unsigned int i = 0; i < 10; i++)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            ImGui::ButtonEx("Frame", ImVec2(width, 30.0f));
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
            x += (width + margin);
            ImGui::SameLine(x);
        }
        ImGui::NewLine();

        x = 0.0f;
        float childWidth = 50.0f;
        for (unsigned int i = 0; i < 10; i++)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            ImGui::ButtonEx("Update", ImVec2(25.0f, 30.0f));
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
            x += (width + margin);
            ImGui::SameLine(x);
        }
        ImGui::End();
        */

        profiler();
        networkMonitor();
    }

    float timeScale;
};

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, false);

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator(new EarthManipulator(arguments));

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    viewer.setRealizeOperation(new GlewInitOperation);
    viewer.addEventHandler(new ImGuiDemo);

    osg::ref_ptr< ImGuiMetricsBackend > metrics = new ImGuiMetricsBackend;

    Metrics::setMetricsBackend(metrics);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        viewer.setSceneData(node);

        return Metrics::run(viewer);

        /*
        while (!viewer.done())
        {
            viewer.frame();
        }
        return 0;
        */
    }
    else
    {
        return usage(argv[0]);
    }
}