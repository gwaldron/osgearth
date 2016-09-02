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
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarth/Horizon>
#include <osgEarth/Registry>
#include <osgEarthUtil/ActivityMonitorTool>

#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/NodeCallback>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

#define RADIUS  500000.0f
#define RADIUS2 200000.0f

struct MyComputeBoundCallback : public osg::Node::ComputeBoundingSphereCallback
{
    double _radius;
    MyComputeBoundCallback(float radius) : _radius(radius) { }
    osg::BoundingSphere computeBound(const osg::Node&) const
    {
        return osg::BoundingSphere(osg::Vec3f(0,0,0), _radius);
    }
};

struct SetHorizonCallback : public osg::NodeCallback
{
    osg::ref_ptr<Horizon> _horizonProto;
    void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::ref_ptr<Horizon> horizon = osg::clone(_horizonProto.get(), osg::CopyOp::DEEP_COPY_ALL);
        horizon->setEye( nv->getViewPoint() );
        horizon->put( *nv );        
        traverse(node, nv);
    }
};

osg::Node*
installGeometry1(const SpatialReference* srs)
{
    osg::Geode* geode = new osg::Geode();
    geode->setComputeBoundingSphereCallback( new MyComputeBoundCallback(RADIUS) );
    geode->addDrawable( new osg::ShapeDrawable( new osg::Sphere(osg::Vec3f(0,0,0), RADIUS) ) );
    osg::Vec3f center = geode->getBound().center();
    GeoTransform* xform = new GeoTransform();
    xform->setPosition( GeoPoint(srs, 0.0, 0.0, 0.0, ALTMODE_ABSOLUTE) );
    xform->addChild( geode );
    return xform;
}

osg::Node*
installGeometry2(const SpatialReference* srs)
{
    osg::Geode* geode = new osg::Geode();
    geode->setComputeBoundingSphereCallback( new MyComputeBoundCallback(RADIUS2) );
    geode->addDrawable( new osg::ShapeDrawable( new osg::Sphere(osg::Vec3f(0,0,0), RADIUS2) ) );
    osg::Vec3f center = geode->getBound().center();
    GeoTransform* xform = new GeoTransform();
    xform->setPosition( GeoPoint(srs, 180.0, 0.0, 0.0, ALTMODE_ABSOLUTE) );
    xform->addChild( geode );
    return xform;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        osg::Group* root = new osg::Group();
        viewer.setSceneData( root );
        root->addChild( node );

        const SpatialReference* srs = MapNode::get(node)->getMapSRS();
        SetHorizonCallback* set = new SetHorizonCallback();
        set->_horizonProto = new Horizon(srs);
        root->addCullCallback( set );

        osg::Node* item1 = installGeometry1(srs);
        root->addChild( item1 );

        osg::Node* item2 = installGeometry2(srs);
        root->addChild( item2 );

        osg::ref_ptr<Horizon> horizon = new Horizon(srs);

        HorizonCullCallback* callback = new HorizonCullCallback();
        item2->addCullCallback( callback );

        while (!viewer.done())
        {
            viewer.frame();

            osg::Vec3d eye, center, up;
            viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
            horizon->setEye( eye );

            if ( horizon->isVisible( item1->getBound() ) )
            {
                Registry::instance()->endActivity( "large sphere" );
                Registry::instance()->startActivity( "large sphere", "VISIBLE" );
            }
            else
            {
                Registry::instance()->endActivity( "large sphere" );
                Registry::instance()->startActivity( "large sphere", "occluded" );
            }
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
