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
#include <osgEarth/Registry>
#include <osgEarth/ObjectIndex>
#include <osgEarth/GLUtils>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/ObjectIDPicker>
#include <osgEarth/Feature>
#include <osgEarth/FeatureIndex>
#include <osgEarth/AnnotationNode>

#define LC "[rttpicker] "

using namespace osgEarth;
using namespace osgEarth::Util;

//-----------------------------------------------------------------------

//! Application-wide data.
struct App
{
    App(osg::ArgumentParser& args) : 
        viewer(args),
        previewStateSet(nullptr),
        previewTexture(nullptr),
        mapNode(nullptr),
        picker(nullptr),
        highlightUniform(nullptr)
    { }

    osgViewer::Viewer viewer;
    osg::StateSet* previewStateSet;
    osg::Texture2D* previewTexture;
    osgEarth::MapNode* mapNode;
    osgEarth::Util::ObjectIDPicker* picker;
    osg::Uniform* highlightUniform;

    osg::ref_ptr<Feature> _pickedFeature;
    osg::ref_ptr<AnnotationNode> _pickedAnno;
};

struct PickerGUI : public GUI::BaseGUI
{
    App& _app;
    bool _active;
    bool _preview;
    bool _installedTexture;

    PickerGUI(App& app) : BaseGUI("Picker"),
        _app(app),
        _active(true),
        _preview(false),
        _installedTexture(false) { }

    void load(const Config& conf) override {
        conf.get("ShowPreview", _preview);
    }

    void save(Config& conf) override {
        conf.set("ShowPreview", _preview);
    }

    void draw(osg::RenderInfo& ri) override
    {
        if (ImGui::Begin(name(), visible()))
        {
            if (ImGui::Checkbox("Picker active", &_active))
            {
                _app.picker->setNodeMask(_active ? ~0 : 0);
            }

            if (_active)
            {
                if (ImGui::Checkbox("RTT preview", &_preview))
                    dirtySettings();

                if (_preview && _app.previewTexture)
                {
                    osg::Texture2D* pickTex = _app.picker->getOrCreateTexture();
                    if (pickTex)
                    {
                        if (_app.previewStateSet->getTextureAttribute(0, osg::StateAttribute::TEXTURE) != pickTex)
                            _app.previewStateSet->setTextureAttribute(0, pickTex, 1);

                        ImGui::Text("Picker camera preview:");
                        ImGuiUtil::Texture(_app.previewTexture, ri);
                    }
                }

                if (_app._pickedFeature.valid())
                {
                    ImGui::Text("Picked Feature:");
                    ImGui::Indent();
                    {
                        ImGui::Text("FID = %ld", _app._pickedFeature->getFID());
                        ImGui::Separator();
                        for (auto& attr : _app._pickedFeature->getAttrs())
                        {
                            ImGui::Separator();
                            ImGui::TextWrapped("%s = %s", attr.first.c_str(), attr.second.getString().c_str());
                        }
                    }
                    ImGui::Unindent();
                }

                else if (_app._pickedAnno.valid())
                {
                    ImGui::Text("Picked Annotation:");
                    ImGui::Indent();
                    {
                        ImGui::Text("Object name = %s", _app._pickedAnno->getName().c_str());
                        ImGui::Text("Object type = %s", typeid(*_app._pickedAnno).name());
                    }
                    ImGui::Unindent();
                }
            }

            ImGui::End();
        }

    }
};

//-----------------------------------------------------------------------

// Shaders that will highlight the currently "picked" feature.

const char* highlightVert = R"(
    #version 330
    uniform uint objectid_to_highlight;
    uint oe_index_objectid;      // Stage global containing object id
    flat out int selected;
    void checkForHighlight(inout vec4 vertex)
    {
        selected = (objectid_to_highlight > 1u && objectid_to_highlight == oe_index_objectid) ? 1 : 0;
    }
)";
const char* highlightFrag = R"(
    #version 330
    flat in int selected;
    void highlightFragment(inout vec4 color)
    {
        if ( selected == 1 )
            color.rgb = mix(color.rgb, clamp(vec3(0.5,2.0,2.0)*(1.0-color.rgb), 0.0, 1.0), 0.5);
    }
)";

void installHighlighter(App& app)
{
    osg::StateSet* stateSet = app.mapNode->getOrCreateStateSet();
    int attrLocation = Registry::objectIndex()->getObjectIDAttribLocation();

    // This shader program will highlight the selected object.
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
    vp->setFunction( "checkForHighlight",  highlightVert, ShaderComp::LOCATION_VERTEX_CLIP );
    vp->setFunction( "highlightFragment",  highlightFrag, ShaderComp::LOCATION_FRAGMENT_COLORING );

    // Since we're accessing object IDs, we need to load the indexing shader as well:
    Registry::objectIndex()->loadShaders( vp );

    // A uniform that will tell the shader which object to highlight:
    app.highlightUniform = new osg::Uniform("objectid_to_highlight", 0u);
    stateSet->addUniform(app.highlightUniform );
}

//------------------------------------------------------------------------

// A lot of code just to re-color the picker's rtt camera into visible colors :)
// We are just taking the pick texture and re-rendering it to another quad
// with a new shader so we can amplify the colors.
void
setupPreviewCamera(App& app)
{
    // simple fragment shader to recolor a texture
    const char* recolor_vs = R"(
        #version 330
        out vec2 tc;
        void recolor_vs(inout vec4 clip) {
            if      (gl_VertexID==0) { clip = vec4(-1,-1,0,1); tc = vec2(0,0); }
            else if (gl_VertexID==1) { clip = vec4( 1, 1,0,1); tc = vec2(1,1); }
            else if (gl_VertexID==2) { clip = vec4(-1, 1,0,1); tc = vec2(0,1); }
            else if (gl_VertexID==3) { clip = vec4( 1, 1,0,1); tc = vec2(1,1); }
            else if (gl_VertexID==4) { clip = vec4(-1,-1,0,1); tc = vec2(0,0); }
            else if (gl_VertexID==5) { clip = vec4( 1,-1,0,1); tc = vec2(1,0); }
        }
    )";

    const char* recolor_fs = R"(
        #version 330
        in vec2 tc;
        out vec4 frag;
        uniform sampler2D tex;
        void recolor_fs(inout vec4 c) {
            c = texture(tex, tc);
            frag = c==vec4(0)? vec4(1) : vec4(vec3((c.r+c.g+c.b+c.a)/4.0),1);
        }
    )";

    osg::Geometry* geom = new osg::Geometry();
    app.previewStateSet = geom->getOrCreateStateSet();
    geom->setCullingActive(false);
    geom->setUseVertexBufferObjects(true);
    geom->setUseDisplayList(false);
    geom->setVertexArray(new osg::Vec3Array(6));
    geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, 6));
    app.previewStateSet->addUniform(new osg::Uniform("tex", 0));

    VirtualProgram* vp = VirtualProgram::getOrCreate(app.previewStateSet);
    vp->setFunction("recolor_vs", recolor_vs, ShaderComp::LOCATION_VERTEX_CLIP);
    vp->setFunction("recolor_fs", recolor_fs, ShaderComp::LOCATION_FRAGMENT_OUTPUT);

    app.previewTexture = new osg::Texture2D();
    app.previewTexture->setTextureSize(256, 256);
    app.previewTexture->setSourceFormat(GL_RGBA);
    app.previewTexture->setSourceType(GL_UNSIGNED_BYTE);
    app.previewTexture->setInternalFormat(GL_RGBA8);

    osg::Camera* cam = new osg::Camera();
    cam->addChild(geom);
    cam->setClearColor(osg::Vec4(1,0,0,1));
    cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    cam->setViewport(0, 0, 256, 256);
    cam->setRenderOrder(osg::Camera::POST_RENDER);
    cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    cam->setImplicitBufferAttachmentMask(0, 0);
    cam->attach(osg::Camera::COLOR_BUFFER, app.previewTexture);

    app.mapNode->addChild(cam);
}

//-----------------------------------------------------------------------

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
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0]);

    App app(arguments);

    app.viewer.setThreadingModel(app.viewer.SingleThreaded);
    app.viewer.setRealizeOperation(new GUI::ApplicationGUI::RealizeOperation);

    app.viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    app.viewer.setCameraManipulator( new EarthManipulator() );

    // Load up the earth file.
    osg::Node* node = MapNodeHelper().loadWithoutControls(arguments, &app.viewer);
    if ( node )
    {
        GUI::ApplicationGUI* gui = new GUI::ApplicationGUI(true);
        gui->add("Demo", new PickerGUI(app), true);

        app.viewer.setSceneData(node);

        app.mapNode = MapNode::get(node);

        app.picker = new ObjectIDPicker();
        app.picker->setView(&app.viewer);  // which view to pick?
        app.picker->setGraph(app.mapNode); // which graph to pick?
        app.mapNode->addChild(app.picker); // put it anywhere in the graph

        ObjectIDPicker::Function pick = [&](ObjectID id)
        {
            if (id > 0)
            {
                // Got a pick:
                FeatureIndex* index = Registry::objectIndex()->get<FeatureIndex>(id).get();
                Feature* feature = index ? index->getFeature(id) : 0L;
                app._pickedFeature = feature;
                app._pickedAnno = Registry::objectIndex()->get<AnnotationNode>(id).get();
                app.highlightUniform->set(id);
            }
            else
            {
                // No pick:
                app._pickedFeature = nullptr;
                app._pickedAnno = nullptr;
                app.highlightUniform->set(0u);
            }
        };
        // Call our handler when hovering over the map
        app.picker->onHover(pick);

        // Highlight features as we pick'em.
        installHighlighter(app);

        // To display the contents of the pick camera in the imgui panel
        setupPreviewCamera(app);

        // Install the imgui:
        app.viewer.getEventHandlers().push_front(gui);
        return app.viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
