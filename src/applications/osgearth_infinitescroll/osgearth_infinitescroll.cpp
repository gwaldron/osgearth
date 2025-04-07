/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Threading>

#include <osg/MatrixTransform>

#include <iostream>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

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

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    float vfov = -1.0f;
    arguments.read("--vfov", vfov);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before calling load)
    osg::ref_ptr< EarthManipulator > manipulator = new EarthManipulator(arguments);
    viewer.setCameraManipulator( manipulator );

    if ( vfov > 0.0 )
    {
        double fov, ar, n, f;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fov, ar, n, f);
        viewer.getCamera()->setProjectionMatrixAsPerspective(vfov, ar, n, f);
    }

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::ref_ptr< osg::Node > node = MapNodeHelper().load(arguments, &viewer);

    osg::ref_ptr< MapNode > mapNode = MapNode::get(node.get());

    if ( mapNode.valid() )
    {
        if (mapNode->isGeocentric())
        {
            OE_NOTICE << "Please run this example with a projected earth file" << std::endl;
            return 1;
        }
        GeoExtent mapExtent = mapNode->getMap()->getProfile()->getExtent();

        //Disable the middle mouse by default, which is rotate.  This will keep us in 2D mode.
        manipulator->getSettings()->bindMouse(osgEarth::Util::EarthManipulator::ACTION_NULL, osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON, 0);

        // Compute a sensible max range so that the user can't zoom out too far to where we need more than 3 transforms.
        double maxDim = osg::maximum(mapExtent.width(), mapExtent.height());
        double range = ((0.5 * maxDim) / 0.267949849);
        manipulator->getSettings()->setMinMaxDistance(0.0, range);

        osg::Group* root = new osg::Group;

        // We're going to draw the map three times so that we can provide an infinite view scrolling left to right.

        // The centerMatrix is centered around the eye point.
        osg::MatrixTransform* centerMatrix = new osg::MatrixTransform;
        centerMatrix->addChild( mapNode );
        root->addChild( centerMatrix );


        // The left matrix is to the left of the center matrix
        osg::MatrixTransform* leftMatrix = new osg::MatrixTransform;
        leftMatrix->addChild( mapNode );
        root->addChild( leftMatrix );

        // The right matrix is to the right of the center matrix
        osg::MatrixTransform* rightMatrix = new osg::MatrixTransform;
        rightMatrix->addChild( mapNode );
        root->addChild( rightMatrix );

        viewer.setSceneData( root );

        while (!viewer.done())
        {
            // Get the current viewpoint from the EarthManipulator
            Viewpoint vp = manipulator->getViewpoint();
            double eyeX = vp.focalPoint()->x();

            GeoPoint focalPoint = *vp.focalPoint();

            // Adjust the focal point if the user is trying to too far north or south.
            if (focalPoint.y() > mapExtent.yMax())
            {
                focalPoint.y() = mapExtent.yMax();
                vp.focalPoint() = focalPoint;
                manipulator->setViewpoint( vp );
            }
            else if (focalPoint.y() < mapExtent.yMin())
            {
                focalPoint.y() = mapExtent.yMin();
                vp.focalPoint() = focalPoint;
                manipulator->setViewpoint( vp );
            }

            GeoExtent centerExtent =  mapExtent;

            // Figure out which direction we need to shift the map extent
            float direction = 0.0;
            if (eyeX < mapExtent.xMin())
            {
                // Move to the left
                direction = -1.0;
            }
            else if (eyeX > mapExtent.xMax())
            {
                // Move to the right
                direction = 1.0;
            }


            // Shift the center extent so that it's centered around the eye point.
            float offset = 0.0;

            if (direction != 0.0)
            {
                while (true)
                {
                    centerExtent = GeoExtent(centerExtent.getSRS(),
                                   mapExtent.xMin() + offset, mapExtent.yMin(),
                                   mapExtent.xMax() + offset, mapExtent.yMax());
                    if (eyeX >= centerExtent.xMin() && eyeX <= centerExtent.xMax())
                    {
                        break;
                    }

                    offset += direction * centerExtent.width();
                }
            }

            // Update the matrix transforms.
            centerMatrix->setMatrix(osg::Matrixd::translate(offset, 0.0, 0.0));
            leftMatrix->setMatrix(osg::Matrixd::translate(offset - mapExtent.width(), 0.0, 0.0));
            rightMatrix->setMatrix(osg::Matrixd::translate(offset + mapExtent.width(), 0.0, 0.0));

            viewer.frame();
        }

    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}
