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

#include <osgEarth/Horizon>
#include <osgEarth/Lighting>
#include <osgEarth/GLUtils>
#include <osgEarth/LineDrawable>

#include <osgEarth/PlaceNode>

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
#include <cstdlib> // for putenv

#if 0
struct PlacerCallback : public MouseCoordsTool::Callback
{
    PlaceNode* _place;
    osg::View* _eyeView;
    PlacerCallback(PlaceNode* place, osgViewer::View* eyeView) : _place(place), _eyeView(eyeView) { }

    // called when valid map coordinates are found under the mouse
    void set(const GeoPoint& coords, osg::View* view, MapNode* mapNode)
    {
        _place->setPosition(coords);
        _place->setNodeMask(~0);

        osg::Vec3d eyeWorld, c, u;
        _eyeView->getCamera()->getViewMatrixAsLookAt(eyeWorld, c, u);
        osg::Vec3d placeWorld;
        coords.toWorld(placeWorld);

        _place->setText( Stringify() << "Range: " << (int)(eyeWorld-placeWorld).length() << "m" );
    }

    // called when no map coords are found under the mouse
    void reset(osg::View* view, MapNode* mapNode)
    {
        _place->setNodeMask(0);
    }
};
#endif

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
    // Get near and far from the Projection matrix.
    const double near = proj(3,2) / (proj(2,2)-1.0);
    const double far = proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    const double nLeft = near * (proj(2,0)-1.0) / proj(0,0);
    const double nRight = near * (1.0+proj(2,0)) / proj(0,0);
    const double nTop = near * (1.0+proj(2,1)) / proj(1,1);
    const double nBottom = near * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    const double fLeft = far * (proj(2,0)-1.0) / proj(0,0);
    const double fRight = far * (1.0+proj(2,0)) / proj(0,0);
    const double fTop = far * (1.0+proj(2,1)) / proj(1,1);
    const double fBottom = far * (proj(2,1)-1.0) / proj(1,1);

    LineDrawable* geom = static_cast<LineDrawable*>(node->getUserData());

    osg::Vec3
        LBN(nLeft, nBottom, -near),
        RBN(nRight, nBottom, -near),
        RTN(nRight, nTop, -near),
        LTN(nLeft, nTop, -near),
        LBF(fLeft, fBottom, -far),
        RBF(fRight, fBottom, -far),
        RTF(fRight, fTop, -far),
        LTF(fLeft, fTop, -far),
        EYE(0,0,0);

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
    geom->setVertex(i++, EYE); geom->setVertex(i++, LBF);
    geom->setVertex(i++, EYE); geom->setVertex(i++, RBF);
    geom->setVertex(i++, EYE); geom->setVertex(i++, LTF);
    geom->setVertex(i++, EYE); geom->setVertex(i++, RTF);

    osg::MatrixTransform* xform = static_cast<osg::MatrixTransform*>(geom->getParent(0));
    xform->setMatrix( osg::Matrixd::inverse(modelview) );
}


char debugEnv[] = "OSGEARTH_REX_DEBUG=1";

int
main( int argc, char** argv )
{
    osgEarth::initialize();

    putenv(debugEnv);

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

    // Create View 0 -- Just the loaded model.
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 20, 20, 800, 800 );
        view->setSceneData( scene );
        view->setCameraManipulator( new EarthManipulator() );
    }

    // Create view 1 -- Contains the loaded moel, as well as a wireframe frustum derived from View 0's Camera.
    {
        osgViewer::View* view = new osgViewer::View;
        viewer.addView( view );

        view->setUpViewInWindow( 850, 20, 800, 800 );
        view->setSceneData( root.get() );
        view->setCameraManipulator( new EarthManipulator() );
    }

    MapNodeHelper helper;
    osg::ref_ptr<osg::Node> node = helper.load(arguments, &viewer);
    if (!node.valid())
    {
        return -1;
    }
    scene->addChild( node );

    helper.configureView( viewer.getView(0) );
    helper.configureView( viewer.getView(1) );

    MapNode* mapNode = MapNode::get(node.get());

    osg::ref_ptr<osg::Image> icon = osgDB::readRefImageFile("../data/placemark32.png");
    PlaceNode* place = new PlaceNode();
    place->setIconImage(icon.get());
    place->setMapNode(mapNode);
    place->getOrCreateStateSet()->setRenderBinDetails(10, "DepthSortedBin");
    place->setDynamic(true);
    place->setNodeMask(0);
    viewer.getView(0)->getCamera()->addChild( place );

#if 0
    MouseCoordsTool* mct = new MouseCoordsTool(mapNode);
    mct->addCallback( new PlacerCallback(place, viewer.getView(0)) );
    viewer.getView(1)->addEventHandler( mct );
#endif

    mapNode->addChild(new HorizonNode());

    osg::Matrix proj;
    viewer.getView(0)->getCamera()->addCullCallback(new CaptureFrustum(proj));

    viewer.getView(1)->getCamera()->setName("Spy");

    viewer.getView(1)->getCamera()->setCullCallback(
        new ObjectStorage::SetValue<bool>("osgEarth.Spy", true));

    while (!viewer.done())
    {
        updateFrustumGeometry(
            frustum,
            viewer.getView(0)->getCamera()->getViewMatrix(),
            proj);

        viewer.frame();
    }
    return 0;
}
