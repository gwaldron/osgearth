/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osg/GraphicsThread>

#include <osgViewer/Viewer>
#include <osgDB/FileNameUtils>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSilverLining/SilverLiningNode>
#include <osgEarthSilverLining/SilverLiningOptions>
#include <osgEarthTriton/TritonContext>
#include <osgEarthTriton/TritonNode>
#include <osgEarthTriton/TritonOptions>

#define LC "[osgearth_sundog] "

using namespace osgEarth::SilverLining;

struct SetupSL : public osgEarth::SilverLining::InitializationCallback
{
    void operator()(Atmosphere& atmosphere)
    {
        CloudLayer cirrus = CloudLayerFactory::Create(CIRRUS_FIBRATUS);
        cirrus.SetIsInfinite(true);
        cirrus.SetFadeTowardEdges(true);
        cirrus.SetBaseAltitude(8000);
        cirrus.SetThickness(500);
        cirrus.SetDensity(0.4);
        cirrus.SetBaseLength(100000);
        cirrus.SetBaseWidth(100000);
        cirrus.SetLayerPosition(0, 0);
        cirrus.SeedClouds(atmosphere);
        atmosphere.GetConditions().AddCloudLayer(cirrus);

        CloudLayer cumulus = CloudLayerFactory::Create(CUMULUS_CONGESTUS);
        cumulus.SetIsInfinite(true);
        cumulus.SetBaseAltitude(1500);
        cumulus.SetThickness(100);
        cumulus.SetBaseLength(30000);
        cumulus.SetBaseWidth(30000);
        cumulus.SetDensity(2.0);
        cumulus.SetLayerPosition(0,0);
        cumulus.SetFadeTowardEdges(true);
        cumulus.SetAlpha(0.8);
        cumulus.SetCloudAnimationEffects(0.1, false, 0, 0);
        cumulus.SeedClouds(atmosphere);
        atmosphere.GetConditions().AddCloudLayer(cumulus);
        
        atmosphere.EnableLensFlare(true);
    }
};

int
usage(const char* name)
{
    OE_DEBUG 
        << "\nUsage: " << name << " file.earth" << std::endl
        << osgEarth::Util::MapNodeHelper().usage() << std::endl;

    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    if ( arguments.read("--stencil") )
        osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Group* node = osgEarth::Util::MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        viewer.getCamera()->setNearFarRatio(0.00002);
        viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

        viewer.setSceneData( node );

        osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( node );

        // Create SilverLiningNode from SilverLiningOptions
        osgEarth::SilverLining::SilverLiningOptions slOptions;
        slOptions.user() = "my_user_name";
        slOptions.licenseCode() = "my_license_code";
        slOptions.cloudsMaxAltitude() = 100000;

        const char* ev_sl = ::getenv("SILVERLINING_PATH");
        if ( ev_sl )
        {
            slOptions.resourcePath() = osgDB::concatPaths(
                std::string(ev_sl),
                "Resources" );
        }
        else
        {
            OE_WARN << LC
                << "No resource path! SilverLining might not initialize properly. "
                << "Consider setting the SILVERLINING_PATH environment variable."
                << std::endl;
        }

        // TODO: uncommenting the callback on the following line results in a crash when SeedClouds is called.
        osg::ref_ptr<osgEarth::SilverLining::SilverLiningNode> sky = new osgEarth::SilverLining::SilverLiningNode(
            mapNode->getMap(), slOptions, new SetupSL() );

        node->addChild(sky);


        // Create TritonNode from TritonOptions
        osgEarth::Triton::TritonOptions tritonOptions;
        tritonOptions.user() = "my_user_name";
        tritonOptions.licenseCode() = "my_license_code";
        tritonOptions.maxAltitude() = 10000;

        const char* ev_t = ::getenv("TRITON_PATH");
        if ( ev_t )
        {
            tritonOptions.resourcePath() = osgDB::concatPaths(
                std::string(ev_t),
                "Resources" );

            OE_INFO << LC 
                << "Setting resource path to << " << tritonOptions.resourcePath().get()
                << std::endl;
        }
        else
        {
            OE_WARN << LC
                << "No resource path! Triton might not initialize properly. "
                << "Consider setting the TRITON_PATH environment variable."
                << std::endl;
        }

#if 0
        osg::ref_ptr<osgEarth::Triton::TritonNode> ocean = new osgEarth::Triton::TritonNode( mapNode, tritonOptions );
        sky->addChild( ocean );


        bool initialized_triton = false;
        while(!viewer.done())
        {
            viewer.frame();

            // need to wait until triton environment is created before working with it
            if (!initialized_triton && ocean->getContext()->ready())
            {
                // set sea state to hurricane levels
                ocean->getContext()->getEnvironment()->SimulateSeaState( 12.0, 90.0 );
                initialized_triton = true;
            }
        }
        return 0;
#else
        return viewer.run();
#endif
    }
    else
    {
        return usage(argv[0]);
    }
}
