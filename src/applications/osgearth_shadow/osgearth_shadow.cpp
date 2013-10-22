/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarthDrivers/engine_osgterrain/OSGTerrainOptions>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers;

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

    osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene();
    
    //Setup a vdsm shadow map
    osgShadow::ShadowSettings* settings = new osgShadow::ShadowSettings;
    settings->setShaderHint( osgShadow::ShadowSettings::NO_SHADERS );
    settings->setUseOverrideForShadowMapTexture( true );
    shadowedScene->setShadowSettings(settings);

    if (arguments.read("--persp")) settings->setShadowMapProjectionHint(osgShadow::ShadowSettings::PERSPECTIVE_SHADOW_MAP);
    if (arguments.read("--ortho")) settings->setShadowMapProjectionHint(osgShadow::ShadowSettings::ORTHOGRAPHIC_SHADOW_MAP);

    double n=0.0;
    if (arguments.read("-n",n)) settings->setMinimumShadowMapNearFarRatio(n);

    unsigned int numShadowMaps;
    if (arguments.read("--num-sm",numShadowMaps)) settings->setNumShadowMapsPerLight(numShadowMaps);

    if (arguments.read("--parallel-split") || arguments.read("--ps") ) settings->setMultipleShadowMapHint(osgShadow::ShadowSettings::PARALLEL_SPLIT);
    if (arguments.read("--cascaded")) settings->setMultipleShadowMapHint(osgShadow::ShadowSettings::CASCADED);

    settings->setDebugDraw( arguments.read("--debug") );

    int mapres = 1024;
    while (arguments.read("--mapres", mapres))
        settings->setTextureSize(osg::Vec2s(mapres,mapres));

    // VDSM is really the only technique that osgEarth supports.
    osg::ref_ptr<osgShadow::ViewDependentShadowMap> vdsm = new osgShadow::ViewDependentShadowMap;
    shadowedScene->setShadowTechnique(vdsm.get());
    
    osg::ref_ptr<osg::Group> model = MapNodeHelper().load(arguments, &viewer);
    if (!model.valid())
    {
        OE_NOTICE
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
        exit(1);
    }

    MapNode* mapNode = osgEarth::findTopMostNodeOfType< MapNode > ( model.get() );

    SkyNode* skyNode = osgEarth::findTopMostNodeOfType< SkyNode > ( model.get() );
    if (!skyNode)
    {
        OE_NOTICE << "Please run with options --sky to enable the SkyNode" << std::endl;
        //exit(1);
    }

    // Prevent terrain skirts (or other "secondary geometry") from casting shadows
    const TerrainOptions& terrainOptions = mapNode->getTerrainEngine()->getTerrainOptions();
    shadowedScene->setCastsShadowTraversalMask( ~terrainOptions.secondaryTraversalMask().value() );

    // Enables shadowing on an osgEarth map node
    ShadowUtils::setUpShadows(shadowedScene, mapNode);

    // For shadowing to work, lighting MUST be enabled on the map node.
    mapNode->getOrCreateStateSet()->setMode(
        GL_LIGHTING,
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

    // Insert the ShadowedScene decorator just above the MapNode. We don't want other
    // elements (Control canvas, annotations, etc) under the decorator.
    osg::Group* root;

    if ( mapNode->getNumParents() > 0 )
    {
        osg::Group* parent = mapNode->getParent(0);
        parent->addChild( shadowedScene );
        shadowedScene->addChild( mapNode );
        parent->removeChild( mapNode );
        root = model.get();
    }
    else
    {
        root = shadowedScene;
        shadowedScene->addChild( model.get() );
    }

    // The skynode's ambient brightness will control the "darkness" of shadows.
    if ( skyNode )
    {
        skyNode->setAmbientBrightness( ambientBrightness );
    }

    viewer.setSceneData( root );
    viewer.realize();
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
    return viewer.run();    
}

