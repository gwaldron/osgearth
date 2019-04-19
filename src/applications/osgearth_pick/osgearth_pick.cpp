/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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

#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/ObjectIndex>
#include <osgEarth/GLUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/RTTPicker>
#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/FeatureIndex>
#include <osgEarthAnnotation/AnnotationNode>

#include <osgEarth/IntersectionPicker>

#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osg/BlendFunc>

#define LC "[rttpicker] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Annotation;

namespace ui = osgEarth::Util::Controls;

//-----------------------------------------------------------------------

//! Application-wide data.
struct App
{
    App(osg::ArgumentParser& args) : viewer(args), mainView(NULL), rttView(NULL), mapNode(NULL), picker(NULL) { }

    osgViewer::CompositeViewer viewer;
    osgViewer::View* mainView;
    osgViewer::View* rttView;
    osgEarth::MapNode* mapNode;
    osgEarth::Util::RTTPicker* picker;

    ui::LabelControl* fidLabel;
    ui::LabelControl* nameLabel;
    osg::Uniform*     highlightUniform;
};


//! Callback that you install on the RTTPicker.
struct MyPickCallback : public RTTPicker::Callback
{
    App& _app;
    MyPickCallback(App& app) : _app(app) { }

    void onHit(ObjectID id)
    {
        // First see whether it's a feature:
        FeatureIndex* index = Registry::objectIndex()->get<FeatureIndex>(id).get();
        Feature* feature = index ? index->getFeature( id ) : 0L;

        if ( feature )
        {
            _app.fidLabel->setText( Stringify() << "Feature ID = " << feature->getFID() << " (oid = " << id << ")" );
            _app.nameLabel->setText( Stringify() << "Name = " << feature->getString("name") );
        }

        else
        {
            // Check whether it's an annotation:
            AnnotationNode* anno = Registry::objectIndex()->get<AnnotationNode>(id).get();
            if ( anno )
            {
                _app.fidLabel->setText( Stringify() << "ObjectID = " << id );
                _app.nameLabel->setName( Stringify() << "Name = " << anno->getName() );
            }

            // None of the above.. clear.
            else
            {
                _app.fidLabel->setText( Stringify() << "unknown oid = " << id );
                _app.nameLabel->setText( " " );
            }
        }

        _app.highlightUniform->set( id );
    }

    void onMiss()
    {
        _app.fidLabel->setText( "No pick." );
        _app.nameLabel->setText( " " );
        _app.highlightUniform->set( 0u );
    }

    // pick whenever the mouse moves.
    bool accept(const osgGA::GUIEventAdapter& ea, const osgGA::GUIActionAdapter& aa)
    {
        return ea.getEventType() == ea.MOVE;
    }
};

//-----------------------------------------------------------------------

// Shaders that will highlight the currently "picked" feature.

const char* highlightVert =
    "#version " GLSL_VERSION_STR "\n"
    "uniform uint objectid_to_highlight; \n"
    "uint oe_index_objectid;      // Stage global containing object id \n"
    "flat out int selected; \n"
    "void checkForHighlight(inout vec4 vertex) \n"
    "{ \n"
    "    selected = (objectid_to_highlight > 1u && objectid_to_highlight == oe_index_objectid) ? 1 : 0; \n"
    "} \n";

const char* highlightFrag =
    "#version " GLSL_VERSION_STR "\n"
    "flat in int selected; \n"
    "void highlightFragment(inout vec4 color) \n"
    "{ \n"
    "    if ( selected == 1 ) \n"
    "        color.rgb = mix(color.rgb, clamp(vec3(0.5,2.0,2.0)*(1.0-color.rgb), 0.0, 1.0), 0.5); \n"
    "} \n";

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

// Configures a window that lets you see what the RTT camera sees.
void
setupRTTView(osgViewer::View* view, osg::Texture* rttTex)
{
    view->setCameraManipulator(0L);
    view->getCamera()->setName( "osgearth_pick RTT view" );
    view->getCamera()->setViewport(0,0,256,256);
    view->getCamera()->setClearColor(osg::Vec4(1,1,1,1));
    view->getCamera()->setProjectionMatrixAsOrtho2D(-.5,.5,-.5,.5);
    view->getCamera()->setViewMatrixAsLookAt(osg::Vec3d(0,-1,0), osg::Vec3d(0,0,0), osg::Vec3d(0,0,1));
    view->getCamera()->setProjectionResizePolicy(osg::Camera::FIXED);

    osg::Vec3Array* v = new osg::Vec3Array(6);
    (*v)[0].set(-.5,0,-.5); (*v)[1].set(.5,0,-.5); (*v)[2].set(.5,0,.5); (*v)[3].set((*v)[2]); (*v)[4].set(-.5,0,.5);(*v)[5].set((*v)[0]);

    osg::Vec2Array* t = new osg::Vec2Array(6);
    (*t)[0].set(0,0); (*t)[1].set(1,0); (*t)[2].set(1,1); (*t)[3].set((*t)[2]); (*t)[4].set(0,1); (*t)[5].set((*t)[0]);

    osg::Geometry* g = new osg::Geometry();
    g->setUseVertexBufferObjects(true);
    g->setUseDisplayList(false);
    g->setVertexArray( v );
    g->setTexCoordArray( 0, t );
    g->addPrimitiveSet( new osg::DrawArrays(GL_TRIANGLES, 0, 6) );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( g );

    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setDataVariance(osg::Object::DYNAMIC);

    stateSet->setTextureAttributeAndModes(0, rttTex, 1);
    rttTex->setUnRefImageDataAfterApply( false );
    rttTex->setResizeNonPowerOfTwoHint(false);

    GLUtils::setLighting(stateSet, 0);
    stateSet->setMode(GL_CULL_FACE, 0);
    stateSet->setAttributeAndModes(new osg::BlendFunc(GL_ONE, GL_ZERO), 1);
    
    const char* fs =
    "#version " GLSL_VERSION_STR "\n"
    "void swap(inout vec4 c) { c.rgba = c==vec4(0)? vec4(1) : vec4(vec3((c.r+c.g+c.b+c.a)/4.0),1); }\n";
    osgEarth::Registry::shaderGenerator().run(geode);
    VirtualProgram::getOrCreate(geode->getOrCreateStateSet())->setFunction("swap", fs, ShaderComp::LOCATION_FRAGMENT_COLORING);

    view->setSceneData( geode );
}

void startPicker(App& app)
{
    // Note! Must stop and restart threading when removing the picker
    // because it changes the OSG View/Slave configuration.
    app.viewer.stopThreading();

    app.picker = new RTTPicker();
    app.mainView->addEventHandler(app.picker);

    // add the graph that will be picked.
    app.picker->addChild(app.mapNode);

    // install a callback that controls the picker and listens for hits.
    app.picker->setDefaultCallback(new MyPickCallback(app));

    // Make a view that lets us see what the picker sees.
    if (app.rttView == NULL)
    {
        app.rttView = new osgViewer::View();
        app.rttView->getCamera()->setGraphicsContext(app.mainView->getCamera()->getGraphicsContext());
        app.rttView->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
        app.viewer.addView(app.rttView);    
    }
    setupRTTView(app.rttView, app.picker->getOrCreateTexture(app.mainView));
    app.rttView->getCamera()->setNodeMask(~0);

    app.viewer.startThreading();
}

void stopPicker(App& app)
{
    // Note! Must stop and restart threading when removing the picker
    // because it changes the OSG View/Slave configuration.
    app.viewer.stopThreading();

    //app.viewer.removeView(app.rttView);
    app.rttView->getCamera()->setNodeMask(0);
    app.mainView->removeEventHandler(app.picker);
    app.picker = 0L;

    app.viewer.startThreading();
}

struct TogglePicker : public ui::ControlEventHandler
{
    App& _app;
    TogglePicker(App& app) : _app(app) { }
    void onClick(Control* button)
    {
        if (_app.picker == 0L)
            startPicker(_app);
        else
            stopPicker(_app);
    }
};

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
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0]);

    App app(arguments);

    app.mainView = new osgViewer::View();
    app.mainView->setUpViewInWindow(30, 30, 1024, 1024, 0);
    app.mainView->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    
    app.viewer.addView(app.mainView);

    app.mainView->getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );
    app.mainView->setCameraManipulator( new EarthManipulator() );    

    // Made some UI components:
    ui::VBox* uiContainer = new ui::VBox();
    uiContainer->setAlign( ui::Control::ALIGN_LEFT, ui::Control::ALIGN_TOP );
    uiContainer->setAbsorbEvents( true );
    uiContainer->setBackColor(0,0,0,0.8);

    uiContainer->addControl( new ui::LabelControl("RTT Picker Test", osg::Vec4(1,1,0,1)) );
    uiContainer->addControl( new ui::ButtonControl("Toggle picker", new TogglePicker(app)) );
    app.fidLabel = new ui::LabelControl("---");
    uiContainer->addControl( app.fidLabel );
    app.nameLabel = uiContainer->addControl( new ui::LabelControl( "---" ) );

    // Load up the earth file.
    osg::Node* node = MapNodeHelper().load( arguments, &app.viewer, uiContainer );
    if ( node )
    {
        app.mainView->setSceneData( node );

        app.mapNode = MapNode::get(node);

        // start with a picker running
        startPicker(app);

        // Highlight features as we pick'em.
        installHighlighter(app);

        app.mainView->getCamera()->setName( "Main view" );

        return app.viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
