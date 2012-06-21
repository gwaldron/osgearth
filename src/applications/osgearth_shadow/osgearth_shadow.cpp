/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <osg/ArgumentParser>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/Geometry>

#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgShadow/ShadowedScene>
#include <osgShadow/ViewDependentShadowMap>

#include <osgUtil/Optimizer>

#include <osgDB/ReadFile>

#include <osg/io_utils>
#include <iostream>
#include <sstream>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/ShadowUtils>
#include <osgEarth/NodeUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc, argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() + " demonstrates osgEarth working with osgShadow");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName());
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");     
   
    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    // if user request help write it out to cout.
    if (arguments.read("-h") || arguments.read("--help"))
    {
        arguments.getApplicationUsage()->write(std::cout);
        return 1;
    }

    float ambientBrightness = 0.4f;
    arguments.read("--ambientBrightness", ambientBrightness);

    // set up the camera manipulators.
    viewer.setCameraManipulator(new EarthManipulator);

    osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene;
    
    //Setup a vdsm shadow map
    osgShadow::ShadowSettings* settings = new osgShadow::ShadowSettings;
    shadowedScene->setShadowSettings(settings);

    while( arguments.read("--debugHUD") ) settings->setDebugDraw( true );
    if (arguments.read("--persp")) settings->setShadowMapProjectionHint(osgShadow::ShadowSettings::PERSPECTIVE_SHADOW_MAP);
    if (arguments.read("--ortho")) settings->setShadowMapProjectionHint(osgShadow::ShadowSettings::ORTHOGRAPHIC_SHADOW_MAP);

    unsigned int unit=1;
    if (arguments.read("--unit",unit)) settings->setBaseShadowTextureUnit(unit);
    
    double n=0.0;
    if (arguments.read("-n",n)) settings->setMinimumShadowMapNearFarRatio(n);


    unsigned int numShadowMaps;
    if (arguments.read("--num-sm",numShadowMaps)) settings->setNumShadowMapsPerLight(numShadowMaps);

    if (arguments.read("--parallel-split") || arguments.read("--ps") ) settings->setMultipleShadowMapHint(osgShadow::ShadowSettings::PARALLEL_SPLIT);
    if (arguments.read("--cascaded")) settings->setMultipleShadowMapHint(osgShadow::ShadowSettings::CASCADED);


    int mapres = 1024;
    while (arguments.read("--mapres", mapres))
        settings->setTextureSize(osg::Vec2s(mapres,mapres));

    osg::ref_ptr<osgShadow::ViewDependentShadowMap> vdsm = new osgShadow::ViewDependentShadowMap;
    shadowedScene->setShadowTechnique(vdsm.get());
    
    osg::ref_ptr<osg::Group> root = shadowedScene;
    osg::ref_ptr<osg::Group> model = MapNodeHelper().load(arguments, &viewer);

    SkyNode* skyNode = findTopMostNodeOfType< SkyNode > ( model.get() );

    if (!model.valid())
    {
        OE_NOTICE
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
        exit(1);
    }

    if (!skyNode)
    {
        OE_NOTICE << "Please run with options --sky to enable the SkyNode" << std::endl;
        exit(1);
    }

    ShadowUtils::setUpShadows(shadowedScene, model);

    // The ControlCanvas is a camera and doesn't play nicely with the
    // shadow traversal. Also, it shouldn't be shadowed, and the
    // ReceivesShadowTraversalMask doesn't really prevent that. So,
    // take the canvas out of the shadowed scene.
    for (unsigned int i = 0; i < model->getNumChildren(); ++i)
    {
        osg::ref_ptr<Controls::ControlCanvas> canvas
            = dynamic_cast<Controls::ControlCanvas*>(model->getChild(i));
        if (canvas.valid())
        {
            root = new osg::Group;
            root->addChild(shadowedScene);
            model->removeChild(i);
            root->addChild(canvas.get());
            break;
        }
    }

    shadowedScene->addChild(model.get());

    if (skyNode )
    {
        skyNode->setAmbientBrightness( ambientBrightness );
    }

    viewer.setSceneData(root.get());

    // create the windows and run the threads.
    viewer.realize();


    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));    

    return viewer.run();    
}

