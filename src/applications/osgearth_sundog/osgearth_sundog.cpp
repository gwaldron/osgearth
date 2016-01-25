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

#include <SilverLining.h>
#include <Triton.h>

#include <osg/GraphicsThread>

#include <osgViewer/Viewer>
#include <osgDB/FileNameUtils>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSilverLining/SilverLiningContext>
#include <osgEarthSilverLining/SilverLiningNode>
#include <osgEarthSilverLining/SilverLiningOptions>
#include <osgEarthTriton/TritonContext>
#include <osgEarthTriton/TritonNode>
#include <osgEarthTriton/TritonOptions>

#define LC "[osgearth_sundog] "


class SetupCloudsGraphicsOp : public osg::GraphicsOperation
{
public:

    SetupCloudsGraphicsOp(::SilverLining::Atmosphere* atmosphere):
        osg::Referenced(true),
        osg::GraphicsOperation("SetupCloudsGraphicsOp",false),
        _atmosphere(atmosphere) {}

    virtual void operator () (osg::GraphicsContext* context)
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

        ::srand(1234);

        if (_atmosphere)
        {
            // create a cirrus layer
            SilverLining::CloudLayer* cirrusCloudLayer;
            cirrusCloudLayer = SilverLining::CloudLayerFactory::Create(CIRRUS_FIBRATUS);
            cirrusCloudLayer->SetIsInfinite( true );
            cirrusCloudLayer->SetFadeTowardEdges(true);
            cirrusCloudLayer->SetBaseAltitude(8000);
            cirrusCloudLayer->SetThickness(500);
            cirrusCloudLayer->SetDensity(0.4);
            cirrusCloudLayer->SetBaseLength(100000);
            cirrusCloudLayer->SetBaseWidth(100000);
            cirrusCloudLayer->SetLayerPosition(0, 0);
            cirrusCloudLayer->SeedClouds(*_atmosphere);
            _atmosphere->GetConditions()->AddCloudLayer(cirrusCloudLayer);

            // create a cumulus layer
            SilverLining::CloudLayer *cumulusCongestusLayer;
            cumulusCongestusLayer = SilverLining::CloudLayerFactory::Create(CUMULUS_CONGESTUS);
            cumulusCongestusLayer->SetIsInfinite(true);
            cumulusCongestusLayer->SetBaseAltitude(1500);
            cumulusCongestusLayer->SetThickness(100);
            cumulusCongestusLayer->SetBaseLength(30000);
            cumulusCongestusLayer->SetBaseWidth(30000);
            cumulusCongestusLayer->SetDensity(0.4);
            cumulusCongestusLayer->SetLayerPosition(0, 0);
            cumulusCongestusLayer->SetFadeTowardEdges(true);
            cumulusCongestusLayer->SetAlpha(0.8);
            // Enable convection effects, but not growth:
            cumulusCongestusLayer->SetCloudAnimationEffects(0.1, false, 0);
            cumulusCongestusLayer->SeedClouds(*_atmosphere);
            _atmosphere->GetConditions()->AddCloudLayer(cumulusCongestusLayer);
        }
    }

    OpenThreads::Mutex  _mutex;
    ::SilverLining::Atmosphere* _atmosphere;
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

        osg::ref_ptr<osgEarth::SilverLining::SilverLiningNode> sky = new osgEarth::SilverLining::SilverLiningNode( mapNode->getMap(), slOptions );

        // Set up wind blowing west at 50 meters/sec
        ::SilverLining::Atmosphere* atm = sky->getContext()->getAtmosphere();

        SilverLining::WindVolume wv;
        wv.SetDirection(90);
        wv.SetMinAltitude(0);
        wv.SetMaxAltitude(10000);
        wv.SetWindSpeed(50);
        atm->GetConditions()->SetWind(wv);

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

        osg::ref_ptr<osgEarth::Triton::TritonNode> ocean = new osgEarth::Triton::TritonNode( mapNode, tritonOptions );
        sky->addChild( ocean );

        osg::ref_ptr<SetupCloudsGraphicsOp> cloudOp = new SetupCloudsGraphicsOp(atm);

        bool initialized_triton = false;
        bool initialized_sl = false;
        while(!viewer.done())
        {
            viewer.frame();

            // neet to wait until SilverLining is initialized before adding the clouds
            if (!initialized_sl && sky->getContext()->ready())
            {
                initialized_sl = true;
                viewer.getCamera()->getGraphicsContext()->add(cloudOp.get());
            }

            // need to wait until triton environment is created before working with it
            if (!initialized_triton && ocean->getContext()->ready())
            {
                // set sea state to hurricane levels
                ocean->getContext()->getEnvironment()->SimulateSeaState( 12.0, 90.0 );
                initialized_triton = true;
            }
        }
        return 0;
    }
    else
    {
        return usage(argv[0]);
    }
}
