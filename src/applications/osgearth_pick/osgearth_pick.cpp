/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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

static ui::LabelControl* s_fidLabel;
static ui::LabelControl* s_nameLabel;
static osg::Uniform*     s_highlightUniform;

//-----------------------------------------------------------------------

// Tests the (old) intersection-based picker.
struct TestIsectPicker : public osgGA::GUIEventHandler
{
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if ( ea.getEventType() == ea.RELEASE )
        {
            IntersectionPicker picker(dynamic_cast<osgViewer::View*>(aa.asView()));
            IntersectionPicker::Hits hits;
            if(picker.pick(ea.getX(), ea.getY(), hits)) {
                std::set<ObjectID> oids;
                if (picker.getObjectIDs(hits, oids)) {
                    ObjectIndex* index = Registry::objectIndex();
                    ObjectID oid = *oids.begin();
                    osg::ref_ptr<FeatureIndex> fi = index->get<FeatureIndex>(oid);
                    if ( fi.valid() ) {
                        OE_NOTICE << "IsectPicker: found OID " << oid << "\n";
                        Feature* f = fi->getFeature(oid);
                        if ( f ) {
                            OE_NOTICE << "...feature ID = " << f->getFID() << "\n";
                        }
                    }      
                    osg::ref_ptr<Feature> f = index->get<Feature>(oid);
                    if ( f.valid() ) {
                        OE_NOTICE << "IsectPicker: found OID " << oid << "\n";
                        OE_NOTICE << "...feature ID = " << f->getFID() << "\n";
                    }
                    osg::ref_ptr<AnnotationNode> a = index->get<AnnotationNode>(oid);
                    if ( a ) {
                        OE_NOTICE << "IsectPicker: found annotation " << a->getName() << "\n";
                    }
                }
                else {
                    OE_NOTICE << "IsectPicker: picked, but no OIDs\n";
                }
            }
            else {
                OE_NOTICE << "IsectPicker: no intersect\n";
            }
        }
        return false;
    }
};


//-----------------------------------------------------------------------

/**
 * Callback that you install on the RTTPicker.
 */
struct MyPickCallback : public RTTPicker::Callback
{
    void onHit(ObjectID id)
    {
        // First see whether it's a feature:
        FeatureIndex* index = Registry::objectIndex()->get<FeatureIndex>( id );
        Feature* feature = index ? index->getFeature( id ) : 0L;

        if ( feature )
        {
            s_fidLabel->setText( Stringify() << "Feature ID = " << feature->getFID() << " (oid = " << id << ")" );
            s_nameLabel->setText( Stringify() << "Name = " << feature->getString("name") );
        }

        else
        {
            // Check whether it's an annotation:
            AnnotationNode* anno = Registry::objectIndex()->get<AnnotationNode>( id );
            if ( anno )
            {
                s_fidLabel->setText( Stringify() << "ObjectID = " << id );
                s_nameLabel->setName( Stringify() << "Name = " << anno->getName() );
            }

            // None of the above.. clear.
            else
            {
                s_fidLabel->setText( Stringify() << "unknown oid = " << id );
                s_nameLabel->setText( " " );
            }
        }

        s_highlightUniform->set( id );
    }

    void onMiss()
    {
        s_fidLabel->setText( "No pick." );
        s_nameLabel->setText( " " );
        s_highlightUniform->set( 0u );
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

void installHighlighter(osg::StateSet* stateSet, int attrLocation)
{
    // This shader program will highlight the selected object.
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
    vp->setFunction( "checkForHighlight",  highlightVert, ShaderComp::LOCATION_VERTEX_CLIP );
    vp->setFunction( "highlightFragment",  highlightFrag, ShaderComp::LOCATION_FRAGMENT_COLORING );

    // Since we're accessing object IDs, we need to load the indexing shader as well:
    Registry::objectIndex()->loadShaders( vp );

    // A uniform that will tell the shader which object to highlight:
    s_highlightUniform = new osg::Uniform("objectid_to_highlight", 0u);
    stateSet->addUniform( s_highlightUniform );
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

    osg::Vec3Array* v = new osg::Vec3Array(4);
    (*v)[0].set(-.5,0,-.5); (*v)[1].set(.5,0,-.5); (*v)[2].set(.5,0,.5); (*v)[3].set(-.5,0,.5);

    osg::Vec2Array* t = new osg::Vec2Array(4);
    (*t)[0].set(0,0); (*t)[1].set(1,0); (*t)[2].set(1,1); (*t)[3].set(0,1);

    osg::Geometry* g = new osg::Geometry();
    g->setUseVertexBufferObjects(true);
    g->setUseDisplayList(false);
    g->setVertexArray( v );
    g->setTexCoordArray( 0, t );
    g->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( g );

    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setDataVariance(osg::Object::DYNAMIC);

    stateSet->setTextureAttributeAndModes(0, rttTex, 1);
    rttTex->setUnRefImageDataAfterApply( false );
    rttTex->setResizeNonPowerOfTwoHint(false);

    stateSet->setMode(GL_LIGHTING, 0);
    stateSet->setMode(GL_CULL_FACE, 0);
    stateSet->setAttributeAndModes(new osg::BlendFunc(GL_ONE, GL_ZERO), 1);
    
    const char* fs = "void swap(inout vec4 c) { c.rgba = c==vec4(0)? vec4(1) : vec4(vec3((c.r+c.g+c.b+c.a)/4.0),1); }\n";
    osgEarth::Registry::shaderGenerator().run(geode);
    VirtualProgram::getOrCreate(geode->getOrCreateStateSet())->setFunction("swap", fs, ShaderComp::LOCATION_FRAGMENT_COLORING);

    view->setSceneData( geode );
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
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer that will hold 2 viewports.
    osgViewer::CompositeViewer viewer(arguments);

    osgViewer::View* mainView = new osgViewer::View();
    mainView->setUpViewInWindow(30, 30, 1024, 1024, 0);
    mainView->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    viewer.addView(mainView);

    // Tell the database pager to not modify the unref settings
    mainView->getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    mainView->setCameraManipulator( new EarthManipulator() );    

    // Made some UI components:
    ui::VBox* uiContainer = new ui::VBox();
    uiContainer->setAlign( ui::Control::ALIGN_LEFT, ui::Control::ALIGN_TOP );
    uiContainer->setAbsorbEvents( true );
    uiContainer->setBackColor(0,0,0,0.8);

    uiContainer->addControl( new ui::LabelControl("RTT Picker Test", osg::Vec4(1,1,0,1)) );
    s_fidLabel = new ui::LabelControl("---");
    uiContainer->addControl( s_fidLabel );
    s_nameLabel = uiContainer->addControl( new ui::LabelControl( "---" ) );

    // Load up the earth file.
    osg::Node* node = MapNodeHelper().load( arguments, mainView, uiContainer );
    if ( node )
    {
        mainView->setSceneData( node );    
        mainView->addEventHandler( new TestIsectPicker() );

        // create a picker of the specified size.
        RTTPicker* picker = new RTTPicker();
        mainView->addEventHandler( picker );

        // add the graph that will be picked.
        picker->addChild( MapNode::get(node) );

        // install a callback that controls the picker and listens for hits.
        picker->setDefaultCallback( new MyPickCallback() );

        // Make a view that lets us see what the picker sees.
        osgViewer::View* rttView = new osgViewer::View();
        rttView->getCamera()->setGraphicsContext( mainView->getCamera()->getGraphicsContext() );
        rttView->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
        viewer.addView( rttView );
        setupRTTView( rttView, picker->getOrCreateTexture(mainView) );

        // Hightlight features as we pick'em.
        installHighlighter(
            MapNode::get(node)->getOrCreateStateSet(),
            Registry::objectIndex()->getObjectIDAttribLocation() );

        mainView->getCamera()->setName( "Main view" );

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
