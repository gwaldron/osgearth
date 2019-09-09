/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include <osgEarth/NodeUtils>
#include <osgEarth/LineDrawable>
#include <osgEarth/Registry>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarthTriton/TritonAPIWrapper>
#include <osgEarthTriton/TritonCallback>
#include <osgEarthTriton/TritonOptions>
#include <osgEarthTriton/TritonLayer>
#include <osgEarthAnnotation/AnnotationLayer>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/GeoPositionNode>

#define LC "[osgearth_triton] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Triton;
using namespace osgEarth::Annotation;
namespace ui = osgEarth::Util::Controls;

struct Settings
{
    optional<double> chop;
    optional<double> seaState;
    optional<float> alpha;
    osg::observer_ptr<TritonLayer> tritonLayer;
    
    void apply(Environment& env, Ocean& ocean)
    {
        if (chop.isSet())
        {
            ocean.SetChoppiness(chop.get());
            chop.clear();
        }

        if (seaState.isSet())
        {
            env.SimulateSeaState(seaState.get(), 0.0);
            seaState.clear();
        }

        osg::ref_ptr<TritonLayer> layer;
        if (alpha.isSet() && tritonLayer.lock(layer))
        {
            layer->setOpacity(alpha.value());
            alpha.clear();
        }
    }
};

class TritonCallback : public osgEarth::Triton::Callback
{
public:
    TritonCallback(Settings& settings) : _settings(settings) { }

    void onInitialize(Environment& env, Ocean& ocean)
    {
        //todo
    }

    void onDrawOcean(Environment& env, Ocean& ocean)
    {
        _settings.apply(env, ocean);
    }

    Settings& _settings;
};


struct App
{
    App()
    {
        tritonLayer = NULL;
        map = NULL;
        const double lon = -118.5406, lat = 32.7838;

        anchor.set(
            SpatialReference::get("wgs84"), lon, lat, 0.0,
            ALTMODE_ABSOLUTE);

        isect = new Triton::TritonIntersections();
    }

    Map*         map;
    TritonLayer* tritonLayer;
    Settings     settings;
    osg::ref_ptr<Triton::TritonIntersections> isect;
    AnnotationLayer* labels;
    AnnotationLayer* normals;
    LineDrawable* normalDrawable;
    GeoPoint anchor;

    void addTriton()
    {
        // Create TritonNode from TritonOptions
        osgEarth::Triton::TritonOptions tritonOptions;
        tritonOptions.user()        = "my_user_name";
        tritonOptions.licenseCode() = "my_license_code";
        tritonOptions.maxAltitude() = 100000;
        tritonOptions.useHeightMap() = true;

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


        tritonLayer = new TritonLayer(tritonOptions, new TritonCallback(settings));
        map->addLayer(tritonLayer);
        settings.tritonLayer = tritonLayer;

        tritonLayer->addIntersections(isect.get());
    }

    void removeTriton()
    {
        if (tritonLayer)
            map->removeLayer(tritonLayer);
        tritonLayer = 0L;
    }

    void addBuoyancyTest(osg::Node* model)
    {
        //labels = new AnnotationLayer();
        //map->addLayer(labels);

        normals = new AnnotationLayer();
        map->addLayer(normals);

        // geometry for a unit normal vector
        normalDrawable = new LineDrawable(GL_LINES);
        normalDrawable->setColor(osg::Vec4(1,1,0,1));
        normalDrawable->pushVertex(osg::Vec3(0,0,0));
        normalDrawable->pushVertex(osg::Vec3(0,0,10));
        normalDrawable->pushVertex(osg::Vec3(-2,0,0.5));
        normalDrawable->pushVertex(osg::Vec3(2,0,0.5));
        normalDrawable->pushVertex(osg::Vec3(0,-2,0.5));
        normalDrawable->pushVertex(osg::Vec3(0,2,0.5));
        normalDrawable->finish();

        // a single shared anchor point for the intersection set:
        isect->setAnchor(anchor);

        // generate a bunch of local points around the anchor:
        for(int x=-50; x<=50; x+=25)
        {
            for(int y=-50; y<=50; y+=25)
            {
                isect->addLocalPoint(osg::Vec3d(x, y, 0));

                // a label communicating the wave height:
                //PlaceNode* label = new PlaceNode();
                //label->setDynamic(true);
                //label->setPosition(anchor);
                //label->setIconImage(image);
                //label->setText("-");
                //labels->getGroup()->addChild(label);

                // a normal vector and optional model:
                GeoPositionNode* normal = new GeoPositionNode();
                normal->setDynamic(true);
                normal->getPositionAttitudeTransform()->addChild(normalDrawable);
                if (model)
                    normal->getPositionAttitudeTransform()->addChild(model);
                normal->setPosition(anchor);
                normals->getGroup()->addChild(normal);
            }
        }

        //ScreenSpaceLayout::setDeclutteringEnabled(false);
    }

    void updateBuoyancyTest()
    {
        for(unsigned i=0; i<isect->getHeights().size(); ++i)
        {
            osg::Vec3d local = isect->getInput()[i];
            local.z() = isect->getHeights()[i];

            //PlaceNode* label = dynamic_cast<PlaceNode*>(labels->getGroup()->getChild(i));
            //label->getPositionAttitudeTransform()->setPosition(local);
            //label->setText(Stringify()<<std::setprecision(2)<<local.z());

            GeoPositionNode* normalNode = dynamic_cast<GeoPositionNode*>(normals->getGroup()->getChild(i));
            normalNode->getPositionAttitudeTransform()->setPosition(local);
            osg::Quat q;
            q.makeRotate(osg::Vec3d(0,0,1), isect->getNormals()[i]);
            normalNode->getPositionAttitudeTransform()->setAttitude(q);
        }
    }
};

App s_app;



template<typename T> struct Set : public ui::ControlEventHandler
{
    optional<T>& _var;
    Set(optional<T>& var) : _var(var) { }
    void onValueChanged(ui::Control*, double value) { _var = value; }
};

struct Toggle : public ui::ControlEventHandler
{
    void onValueChanged(ui::Control*, bool value) {
        if (s_app.tritonLayer)
            s_app.removeTriton();
        else
            s_app.addTriton();
    }
};

Container* createUI()
{
    VBox* box = new VBox();
    box->setBackColor(0,0,0,0.5);
    Grid* grid = box->addControl(new Grid());
    int r=0;
    grid->setControl(0, r, new LabelControl("Chop"));
    grid->setControl(1, r, new HSliderControl(0, 3, 0, new Set<double>(s_app.settings.chop)));
    ++r;
    grid->setControl(0, r, new LabelControl("Sea State"));
    grid->setControl(1, r, new HSliderControl(0, 12, 5, new Set<double>(s_app.settings.seaState)));
    ++r;  
    grid->setControl(0, r, new LabelControl("Alpha"));
    grid->setControl(1, r, new HSliderControl(0, 1.0, 1.0, new Set<float>(s_app.settings.alpha)));
    ++r;
    grid->setControl(0, r, new LabelControl("Toggle"));
    grid->setControl(1, r, new CheckBoxControl(true, new Toggle()));

    grid->getControl(1, r-1)->setHorizFill(true,200);

    return box;
}


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

    osg::Node* model = 0L;
    std::string filename;
    if (arguments.read("--model", filename))
    {
        model = osgDB::readRefNodeFile(filename).release();
        Registry::shaderGenerator().run(model);
    }

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator(manip);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Group* node = osgEarth::Util::MapNodeHelper().load(arguments, &viewer, createUI());
    if ( node )
    {        
        viewer.getCamera()->setNearFarRatio(0.00002);
        viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

        viewer.setSceneData( node );

        s_app.map = MapNode::get( node )->getMap();

        s_app.addTriton();
        s_app.addBuoyancyTest(model);
        
        // Zoom the camera to our area of interest:
        Viewpoint vp;
        vp.heading() = 25.0f;
        vp.pitch() = -25;
        vp.range() = 400.0;
        vp.focalPoint() = s_app.anchor;
        manip->setViewpoint(vp);

        while(!viewer.done())
        {
            viewer.frame();
            s_app.updateBuoyancyTest();
        }
    }
    else
    {
        return usage(argv[0]);
    }
}
