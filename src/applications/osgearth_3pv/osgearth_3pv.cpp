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

#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/Utils>
#include <osgEarth/Math>
#include <osgEarth/ShaderLoader>

#include <osgEarth/Lighting>
#include <osgEarth/GLUtils>
#include <osgEarth/LineDrawable>

#define LC "[viewer] "

using namespace osgEarth;

#include <osg/Geometry>
#include <osg/Depth>
#include <osg/LineStipple>
#include <osg/DisplaySettings>
#include <osg/MatrixTransform>
#include <osg/LineWidth>
#include <osgDB/ReadFile>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>

// this shader will outline each terrain tile
const char* outline_shader = R"(
    #pragma vp_function outline_tile, fragment
    in vec4 oe_layer_tilec;
    void outline_tile(inout vec4 color) {
        vec2 uv = abs(oe_layer_tilec.xy * 2.0 - 1.0);
        if (uv.x > 0.99 || uv.y > 0.99) {
            color = vec4(1.0, 0.0, 0.0, 1.0);
        }
    }
)";

struct CaptureFrustum : public osg::NodeCallback
{
    osg::Matrix& _proj;

    CaptureFrustum(osg::Matrix& proj) : _proj(proj) { }

    void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        traverse(node, nv);

        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        double N = cv->getCalculatedNearPlane();
        double F = cv->getCalculatedFarPlane();
        _proj = cv->getCurrentCamera()->getProjectionMatrix();
        cv->clampProjectionMatrix(_proj, N, F);
    }
};

osg::Node*
createFrustumGeometry()
{
    LineDrawable* geom = new LineDrawable(GL_LINES);
    geom->setDataVariance(osg::Object::DYNAMIC);
    geom->allocate(24);
    geom->setColor(Color::Yellow);
    geom->finish();

    osg::MatrixTransform* mt = new osg::MatrixTransform();
    mt->addChild(geom);

    osg::Group* g0 = new osg::Group();
    osg::StateSet* g0ss = g0->getOrCreateStateSet();
    g0->addChild(mt);
    g0ss->setAttributeAndModes(new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1);
    GLUtils::setLineStipple(g0ss, 1, 0xFF, 1);
    g0ss->setRenderBinDetails(2, "RenderBin");

    osg::Group* g1 = new osg::Group();
    g1->addChild(mt);
    g1->getOrCreateStateSet()->setRenderBinDetails(3, "RenderBin");
    g1->getOrCreateStateSet()->setRenderBinDetails(3, "RenderBin");

    osg::Group* top = new osg::Group();
    top->addChild(g0);
    top->addChild(g1);
    top->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");

    top->setUserData(geom);

    GLUtils::setGlobalDefaults(top->getOrCreateStateSet());
    top->addCullCallback(new InstallCameraUniform());

    return top;
}

// Given a Camera, create a wireframe representation of its
// view frustum. Create a default representation if camera==NULL.
void
updateFrustumGeometry(osg::Node* node, const osg::Matrix& modelview, const osg::Matrix& proj)
{
    osg::Vec3 LBN, RBN, RTN, LTN, LBF, RBF, RTF, LTF;

    if (ProjectionMatrix::isPerspective(proj))
    {
        double L, R, B, T, N, F;
        ProjectionMatrix::getPerspective(proj, L, R, B, T, N, F);

        double x = F / N;
        LBN.set(L, B, -N);
        RBN.set(R, B, -N);
        RTN.set(R, T, -N);
        LTN.set(L, T, -N);
        LBF.set(x * L, x * B, -F);
        RBF.set(x * R, x * B, -F);
        RTF.set(x * R, x * T, -F);
        LTF.set(x * L, x * T, -F);
    }
    else
    {
        double L, R, B, T, N, F;
        ProjectionMatrix::getOrtho(proj, L, R, B, T, N, F);

        LBN.set(L, B, -N);
        RBN.set(R, B, -N);
        RTN.set(R, T, -N);
        LTN.set(L, T, -N);
        LBF.set(L, B, -F);
        RBF.set(R, B, -F);
        RTF.set(R, T, -F);
        LTF.set(L, T, -F);
    }

    LineDrawable* geom = static_cast<LineDrawable*>(node->getUserData());

    int i=0;

    // near
    geom->setVertex(i++, LBN); geom->setVertex(i++, RBN);
    geom->setVertex(i++, RBN); geom->setVertex(i++, RTN);
    geom->setVertex(i++, RTN); geom->setVertex(i++, LTN);
    geom->setVertex(i++, LTN); geom->setVertex(i++, LBN);

    // far
    geom->setVertex(i++, LBF); geom->setVertex(i++, RBF);
    geom->setVertex(i++, RBF); geom->setVertex(i++, RTF);
    geom->setVertex(i++, RTF); geom->setVertex(i++, LTF);
    geom->setVertex(i++, LTF); geom->setVertex(i++, LBF);

    // sides
    if (ProjectionMatrix::isPerspective(proj))
    {
        geom->setVertex(i++, osg::Vec3()); geom->setVertex(i++, LBF);
        geom->setVertex(i++, osg::Vec3()); geom->setVertex(i++, RBF);
        geom->setVertex(i++, osg::Vec3()); geom->setVertex(i++, LTF);
        geom->setVertex(i++, osg::Vec3()); geom->setVertex(i++, RTF);
    }
    else
    {
        geom->setVertex(i++, LBN); geom->setVertex(i++, LBF);
        geom->setVertex(i++, RBN); geom->setVertex(i++, RBF);
        geom->setVertex(i++, RTN); geom->setVertex(i++, RTF);
        geom->setVertex(i++, LTN); geom->setVertex(i++, LTF);
    }

    geom->dirty();

    osg::MatrixTransform* xform = static_cast<osg::MatrixTransform*>(geom->getParent(0));
    xform->setMatrix( osg::Matrixd::inverse(modelview) );
}

int
main( int argc, char** argv )
{
    osgEarth::initialize();

    osg::ArgumentParser arguments( &argc, argv );

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osgViewer::CompositeViewer viewer( arguments );
    viewer.setThreadingModel(viewer.SingleThreaded);

    // Child 0: We'll replace this every frame with an updated representation
    //   of the view frustum.
    osg::Node* frustum = createFrustumGeometry();
    root->addChild(frustum);

    osg::Group* scene = new osg::Group();
    root->addChild(scene);

    // Turn on FSAA, makes the lines look better.
    osg::DisplaySettings::instance()->setNumMultiSamples( 4 );

    bool track = true;
    if (arguments.read("--no-track"))
        track = false;

    auto modelManip = new EarthManipulator();
    auto spyManip = new EarthManipulator();

    // Create View 0 -- Just the loaded model.
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView(view);

        view->setUpViewInWindow(20, 20, 1400, 1400);
        view->setSceneData(scene);
        view->setCameraManipulator(modelManip);
    }

    // Create view 1 -- Contains the loaded moel, as well as a wireframe frustum derived from View 0's Camera.
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView(view);

        view->setUpViewInWindow(1450, 20, 1400, 1400);
        view->setSceneData(root.get());
        view->setCameraManipulator(spyManip);
        spyManip->home(0);
    }

    MapNodeHelper helper;
    osg::ref_ptr<osg::Node> node = helper.load(arguments, &viewer);
    if (!node.valid())
    {
        return -1;
    }
    scene->addChild( node );

    // A shader to outline the terrain tiles.
    ShaderLoader::load(scene, outline_shader);

    helper.configureView( viewer.getView(0) );
    helper.configureView( viewer.getView(1) );

    osg::Matrix proj;
    viewer.getView(0)->getCamera()->addCullCallback(new CaptureFrustum(proj));

    // This puts the terrain engine into SPY mode.
    viewer.getView(1)->getCamera()->setName("Spy");
    viewer.getView(1)->getCamera()->setCullCallback(
        new ObjectStorage::SetValue<bool>("osgEarth.Spy", true));

    auto* mapNode = MapNode::get(node);

    while (!viewer.done())
    {
        updateFrustumGeometry(
            frustum,
            viewer.getView(0)->getCamera()->getViewMatrix(),
            proj);

        if (track)
        {
            if (viewer.getFrameStamp()->getFrameNumber() == 0)
            {
                spyManip->home(0);
            }
            else
            {
                Viewpoint model_vp = modelManip->getViewpoint();
                Viewpoint sky_vp = spyManip->getViewpoint();
                sky_vp.focalPoint() = model_vp.focalPoint();
                spyManip->setViewpoint(sky_vp);
            }
        }

        viewer.frame();
    }
    return 0;
}
