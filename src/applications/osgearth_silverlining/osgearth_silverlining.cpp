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
#include <osgViewer/Viewer>
#include <osgDB/FileNameUtils>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthSilverLining/SilverLiningNode>

#define LC "[osgearth_silverlining] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::SilverLining;

namespace ui = osgEarth::Util::Controls;


struct Settings
{
    SkyNode*         sky;
    optional<double> visibility;
    optional<double> rain;
    optional<double> snow;
    optional<bool>   lighting;
    
    void apply(Atmosphere& atmo)
    {
        if (visibility.isSet())
        {
            atmo.GetConditions().SetVisibility(visibility.get());
            visibility.clear();
        }

        if (rain.isSet())
        {
            atmo.GetConditions().SetPrecipitation(CloudLayer::NONE, 0.0);
            atmo.GetConditions().SetPrecipitation(CloudLayer::RAIN, rain.get());
            rain.clear();
        }

        if (snow.isSet())
        {
            atmo.GetConditions().SetPrecipitation(CloudLayer::NONE, 0.0);
            atmo.GetConditions().SetPrecipitation(CloudLayer::DRY_SNOW, snow.get());
            snow.clear();
        }

        if (lighting.isSet())
        {
            sky->setLighting(lighting.get());
            lighting.clear();
        }
    }
};
Settings s_settings;


template<typename T> struct Set : public ui::ControlEventHandler
{
    optional<T>& _var;
    Set(optional<T>& var) : _var(var) { }
    void onValueChanged(ui::Control*, T value) { _var = value; }
};

struct SetDateTime : public ui::ControlEventHandler
{
    void onValueChanged(ui::Control*, double value)
    {
        DateTime now;
        DateTime skyTime(now.year(), now.month(), now.day(), value);
        s_settings.sky->setDateTime(skyTime);
    }
};


Container* createUI()
{
    VBox* box = new VBox();
    box->setBackColor(0,0,0,0.5);
    Grid* grid = box->addControl(new Grid());
    int r=0;
    //grid->setControl(0, r, new LabelControl("Visibility"));
    //HSliderControl* vis = grid->setControl(1, r, new HSliderControl(100.0f, 1000000.0f, 1000000.0f, new Set<double>(s_settings.visibility)));
    //vis->setHorizFill(true, 175);
    //grid->setControl(2, r, new LabelControl(vis));
    //++r;
    grid->setControl(0, r, new LabelControl("Rain"));
    grid->setControl(1, r, new HSliderControl(0, 100, 0, new Set<double>(s_settings.rain)));
    ++r;
    grid->setControl(0, r, new LabelControl("Snow"));
    grid->setControl(1, r, new HSliderControl(0, 100, 0, new Set<double>(s_settings.snow)));
    ++r;
    grid->setControl(0, r, new LabelControl("Time"));
    grid->setControl(1, r, new HSliderControl(0, 24, 0, new SetDateTime()));
    ++r;
    grid->setControl(0, r, new LabelControl("Lighting"));
    grid->setControl(1, r, new CheckBoxControl(false, new Set<bool>(s_settings.lighting)));
    ++r;
    grid->getControl(1, r-1)->setHorizFill(true,200);
    return box;
}


struct SLCallback : public osgEarth::SilverLining::Callback
{
    void onInitialize(Atmosphere& atmosphere)
    {
        CloudLayer cumulus = CloudLayerFactory::Create(CUMULUS_CONGESTUS);
        cumulus.SetIsInfinite(true);
        cumulus.SetBaseAltitude(3000);
        cumulus.SetThickness(50);
        cumulus.SetBaseLength(100000);
        cumulus.SetBaseWidth(100000);
        cumulus.SetDensity(1.0);
        cumulus.SetLayerPosition(0,0);
        cumulus.SetFadeTowardEdges(true);
        cumulus.SetAlpha(0.8);
        cumulus.SetCloudAnimationEffects(0.1, false, 0, 0);
        cumulus.SeedClouds(atmosphere);
        atmosphere.GetConditions().AddCloudLayer(cumulus);
        
        atmosphere.EnableLensFlare(true);
    }

    void onDrawSky(Atmosphere& atmosphere)
    {
        //nop
    }

    void onDrawClouds(Atmosphere& atmosphere)
    {
        s_settings.apply(atmosphere);
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

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new osgEarth::Util::EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Group* node = MapNodeHelper().load(arguments, &viewer, createUI());
    if ( node )
    {
        // Make sure we don't already have a sky.
        SkyNode* skyNode = osgEarth::findTopMostNodeOfType<SkyNode>(node);
        if (skyNode)
        {
            OE_WARN << LC << "Earth file already has a Sky. This example requires an "
                "earth file that does not use a sky already.\n";
            return -1;
        }

        viewer.getCamera()->setNearFarRatio(0.00002);
        viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

        MapNode* mapNode = MapNode::findMapNode( node );

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
        s_settings.sky = new SilverLiningNode(
            mapNode->getMapSRS(),
            slOptions,
            new SLCallback() );

        // insert the new sky above the map node.
        osgEarth::insertParent(s_settings.sky, mapNode);
        
        // use the topmost node.
        viewer.setSceneData(osgEarth::findTopOfGraph(node));

        // connects the sky's light to the viewer.
        s_settings.sky->attach(&viewer);

        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
