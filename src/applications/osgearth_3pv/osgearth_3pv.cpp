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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/Controls>

#include <osgEarth/Units>
#include <osgEarth/Viewpoint>
#include <osgEarth/Horizon>
#include <osgEarth/TraversalData>

#include <osgEarthAnnotation/PlaceNode>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;
namespace ui = osgEarth::Util::Controls;


#include <osg/Geometry>
#include <osg/Depth>
#include <osg/LineStipple>
#include <osg/DisplaySettings>
#include <osg/MatrixTransform>
#include <osg/LineWidth>
#include <osgDB/ReadFile>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>


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


// Given a Camera, create a wireframe representation of its
// view frustum. Create a default representation if camera==NULL.
osg::Node*
makeFrustumFromCamera( osg::Camera* camera )
{
    // Projection and ModelView matrices
    osg::Matrixd proj;
    osg::Matrixd mv;
    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
    }
    else
    {
        // Create some kind of reasonable default Projection matrix.
        proj.makePerspective( 30., 1., 1., 10. );
        // leave mv as identity
    }

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

    // Our vertex array needs only 9 vertices: The origin, and the
    // eight corners of the near and far planes.
    osg::Vec3Array* v = new osg::Vec3Array;
    v->resize( 9 );
    (*v)[0].set( 0., 0., 0. );
    (*v)[1].set( nLeft, nBottom, -near );
    (*v)[2].set( nRight, nBottom, -near );
    (*v)[3].set( nRight, nTop, -near );
    (*v)[4].set( nLeft, nTop, -near );
    (*v)[5].set( fLeft, fBottom, -far );
    (*v)[6].set( fRight, fBottom, -far );
    (*v)[7].set( fRight, fTop, -far );
    (*v)[8].set( fLeft, fTop, -far );

    osg::Geometry* geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::Vec4Array* c = new osg::Vec4Array;
    c->push_back( osg::Vec4( 1., 1., 0., 1. ) );
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geom );

    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    geode->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(2.0f), 1);


    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix( osg::Matrixd::inverse( mv ) );
    mt->addChild( geode );

    osg::Group* g0 = new osg::Group();
    g0->addChild(mt);
    g0->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1);
    g0->getOrCreateStateSet()->setAttributeAndModes(new osg::LineStipple(1, 0x000F), 1);
    g0->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");

    osg::Group* g1 = new osg::Group();
    g1->addChild(mt);
    g1->getOrCreateStateSet()->setRenderBinDetails(3, "RenderBin");

    osg::Group* gr = new osg::Group();
    gr->addChild(g0);
    gr->addChild(g1);
    gr->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");

    return gr;
}


int
main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osgViewer::CompositeViewer viewer( arguments );
    viewer.setThreadingModel(viewer.SingleThreaded);

    // Child 0: We'll replace this every frame with an updated representation
    //   of the view frustum.
    root->addChild( makeFrustumFromCamera( NULL ) );

    osg::Group* scene = new osg::Group();
    root->addChild( scene );


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
    osg::ref_ptr<osg::Node> node = helper.load(arguments, viewer.getView(0));
    if (!node.valid())
    {
        return -1;
    }
    scene->addChild( node );

    helper.configureView( viewer.getView(0) );
    helper.configureView( viewer.getView(1) );

    MapNode* mapNode = MapNode::get(node.get());

    osg::ref_ptr<osg::Image> icon = osgDB::readImageFile("../data/placemark32.png");
    PlaceNode* place = new PlaceNode(mapNode, GeoPoint::INVALID, icon.get(), "");
    place->getOrCreateStateSet()->setRenderBinDetails(10, "DepthSortedBin");
    place->setDynamic(true);
    place->setNodeMask(0);
    viewer.getView(0)->getCamera()->addChild( place );

    MouseCoordsTool* mct = new MouseCoordsTool(mapNode);
    mct->addCallback( new PlacerCallback(place, viewer.getView(0)) );
    viewer.getView(1)->addEventHandler( mct );

    mapNode->addChild(new HorizonNode());

    viewer.getView(1)->getCamera()->addCullCallback( new VisitorData::Install("osgEarth.Stealth") );

    while (!viewer.done())
    {
        // Update the wireframe frustum
        root->removeChild( 0, 1 );
        root->insertChild( 0, makeFrustumFromCamera( viewer.getView( 0 )->getCamera() ) );

        viewer.frame();
    }
    return 0;
}
