/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/Horizon>
#include <osgEarth/Registry>
#include <osgEarth/ActivityMonitorTool>

#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/NodeCallback>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth --activity" << std::endl
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
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    if (arguments.find("--activity") < 0)
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    auto node = MapNodeHelper().load( arguments, &viewer );
    if (node.valid())
    {
        osg::Group* root = new osg::Group();
        viewer.setSceneData( root );
        root->addChild( node );

        MapNode* mapNode = MapNode::get(node);
        if (!mapNode)
            return usage(argv[0]);

        const SpatialReference* srs = mapNode->getMapSRS();

        osg::Node* item1 = installGeometry1(srs);
        mapNode->addChild( item1 );

        osg::Node* item2 = installGeometry2(srs);
        mapNode->addChild( item2 );

        // Culls the second item based on its horizon visibility
        HorizonCullCallback* callback = new HorizonCullCallback();
        item2->addCullCallback( callback );

        // This horizon object we are just using to print out the results;
        // it's not actually part of the culling cullback!
        osg::ref_ptr<Horizon> horizon = new Horizon(srs);

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
