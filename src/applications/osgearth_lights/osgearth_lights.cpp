/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

/**
 * Lights test. This application is for testing the LightSource support in osgEarth.
 */
#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/Lighting>
#include <osgEarth/PhongLightingEffect>
#include <osgEarth/NodeUtils>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/Ephemeris>
#include <osgEarth/Shadowing>
#include <osgEarth/Sky>

#define LC "[lights] "

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

// converts a double-precision Vec3d to an equivalent single-precision Vec4f position
// as needed for light positions.
osg::Vec4
worldToVec4(const osg::Vec3d& ecef)
{
    osg::Vec4 result(0.0f, 0.0f, 0.0f, 1.0f);
    osg::Vec3d d = ecef;
    while (d.length() > 1e6)
    {
        d *= 0.1;
        result.w() *= 0.1;
    }
    return osg::Vec4(d.x(), d.y(), d.z(), result.w());
}

osg::Vec4
randomColor()
{
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;
    return osg::Vec4(r,g,b,1.0f);
}


osg::Group*
addLights(osg::View* view, osg::Node* root, int lightNum)
{
    MapNode* mapNode = MapNode::get(root);
    const SpatialReference* mapsrs = mapNode->getMapSRS();
    const SpatialReference* geosrs = mapsrs->getGeographicSRS();

    osg::Vec3d world;
    osg::Group* lights = new osg::Group();

    // Add a directional light that simulates the sun - but skip this if a sky
    // was already added in the earth file.
    if (lightNum == 0)
    {
        Ephemeris e;
        DateTime dt(2016, 8, 10, 14.0);
        CelestialBody sun = e.getSunPosition(dt);
        world = sun.geocentric;

        osg::Light* sunLight = new osg::Light(lightNum++);
        world.normalize();
        sunLight->setPosition(osg::Vec4d(world, 0.0));

        sunLight->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.0));
        sunLight->setDiffuse(osg::Vec4(1.0, 1.0, 0.9, 1.0));

        osg::LightSource* sunLS = new osg::LightSource();
        sunLS->setLight(sunLight);

        lights->addChild( sunLS );

        ShadowCaster* caster = osgEarth::findTopMostNodeOfType<ShadowCaster>(root);
        if (caster)
        {
            OE_INFO << "Found a shadow caster!\n";
            caster->setLight(sunLight);
        }
    }

#if 1
    // A red spot light. A spot light has a real position in space
    // and points in a specific direciton. The Cutoff and Exponent
    // properties control the cone angle and sharpness, respectively
    {
        GeoPoint p(geosrs, -121, 34, 5000000., ALTMODE_ABSOLUTE);
        p.toWorld(world);

        osg::Light* spot = new osg::Light(lightNum++);
        spot->setPosition(worldToVec4(world));
        spot->setAmbient(osg::Vec4(0,0.2,0,1));
        spot->setDiffuse(osg::Vec4(1,0,0,1));
        spot->setSpotCutoff(20.0f);
        spot->setSpotExponent(100.0f);

        // point straight down at the map:
        world.normalize();
        spot->setDirection(-world);

        osg::LightSource* spotLS = new osg::LightSource();
        spotLS->setLight(spot);

        lights->addChild( spotLS );
    }

    // A green point light. A Point light lives at a real location in
    // space and lights equally in all directions.
    {
        GeoPoint p(geosrs, -45, -35, 1000000., ALTMODE_ABSOLUTE);
        p.toWorld(world);

        osg::Light* point = new osg::Light(lightNum++);
        point->setPosition(worldToVec4(world));
        point->setAmbient(osg::Vec4(0,0,0,1));
        point->setDiffuse(osg::Vec4(1.0, 1.0, 0.0,1));

        osg::LightSource* pointLS = new osg::LightSource();
        pointLS->setLight(point);

        lights->addChild( pointLS );
    }
#endif

    // Generate the necessary uniforms for the shaders.
    GenerateGL3LightingUniforms gen;
    lights->accept(gen);

    return lights;
}



int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    // Whether to test updating material
    bool update = arguments.read("--update");

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    viewer.setLightingMode(viewer.NO_LIGHT);

    // load an earth file, and support all or our example command-line options
    osg::ref_ptr<osg::Node> node = MapNodeHelper().load(arguments, &viewer);
    if (node.valid())
    {
        MapNode* mapNode = MapNode::get(node.get());
        if ( !mapNode )
            return -1;

        // Example of a custom material for the terrain.
        osg::ref_ptr< osg::Material > material = 0;
        if (update)
        {
            OE_NOTICE << "Custom material" << std::endl;
            material = new osg::Material;
            material->setDiffuse(osg::Material::FRONT, osg::Vec4(1,1,1,1));
            material->setAmbient(osg::Material::FRONT, osg::Vec4(1,1,1,1));
            // Attach our StateAttributeCallback so that uniforms are updated.
            material->setUpdateCallback(new MaterialCallback());
            mapNode->getOrCreateStateSet()->setAttributeAndModes(material);
        }

        // Does a Sky already exist (loaded from the earth file)?
        SkyNode* sky = osgEarth::findTopMostNodeOfType<SkyNode>(node.get());
        if (!sky)
        {
            // Add phong lighting.
            PhongLightingEffect* phong = new PhongLightingEffect();
            phong->attach(node->getOrCreateStateSet());
        }

        osg::Group* lights = addLights(&viewer, node.get(), sky?1:0);

        mapNode->addChild(lights);

        viewer.setSceneData(node.get());
        while (!viewer.done())
        {
            if (viewer.getFrameStamp()->getFrameNumber() % 100 == 0)
            {
                if (material)
                {
                    material->setDiffuse(osg::Material::FRONT, randomColor());
                }
            }
            viewer.frame();
        }
        return 0;
    }
    else
    {
        return usage(argv[0]);
    }
}
