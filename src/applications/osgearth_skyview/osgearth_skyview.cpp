/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgViewer/Viewer>
#include <osg/CullFace>
#include <osgEarth/Notify>
#include <osgEarth/ExampleResources>
#include "SkyManipulator"


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

    // Increase the fov to provide a more immersive experience.
    float vfov = 100.0f;
    arguments.read("--vfov", vfov);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    if ( vfov > 0.0 )
    {
        double fov, ar, n, f;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fov, ar, n, f);
        viewer.getCamera()->setProjectionMatrixAsPerspective(vfov, ar, n, f);
    }

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    auto node = MapNodeHelper().load( arguments, &viewer );

    //Set our custom manipulator
    viewer.setCameraManipulator(new SkyManipulator());

    if ( node )
    {
        // Disable backface culling
       node->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);


        viewer.setSceneData( node );
        while(!viewer.done())
        {
            viewer.frame();
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
