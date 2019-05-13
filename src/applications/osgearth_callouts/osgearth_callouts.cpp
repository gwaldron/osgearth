#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/GeoTransform>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/Horizon>
#include <osgEarth/GLUtils>
#include <osgText/Font>
#include <osgEarth/Callouts>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

struct App
{
    CalloutManager* _cm;
    osg::ref_ptr<osgText::Font> _font;
    osg::ref_ptr<HorizonCullCallback> _horizonCull;
};

osg::Node* loadData(App& app)
{
    osg::Group* group = new osg::Group();

    osg::ref_ptr<OGRFeatureSource> fs = new OGRFeatureSource();
    fs->setURL("../data/cities.gpkg");
    Status s = fs->open();
    if (s.isError())
    {
        OE_WARN << s.message() << std::endl;
        exit(0);
    }

    osg::ref_ptr<FeatureCursor> c = fs->createFeatureCursor(Query(), 0L);
    while(c.valid() && c->hasMore())
    {
        Feature* f = c->nextFeature();
        std::string text = f->getString("name");
        GeoPoint centroid;
        f->getExtent().getCentroid(centroid);
        GeoTransform* xform = new GeoTransform();
        xform->setPosition(centroid);
        Callout* label = new Callout(app._cm);
        label->setText(text);

        int rank = f->getInt("rank_max");
        char buf[256];
        sprintf(buf, "%02d%s", rank, text.c_str());
        label->setUID(buf);

        label->setFont(app._font);
        label->setDrawMode(label->TEXT | label->FILLEDBOUNDINGBOX | label->BOUNDINGBOX );
        label->setBoundingBoxColor(osg::Vec4f(1,1,1,0.8));
        label->setBoundingBoxMargin(2.0f);
        label->setColor(osg::Vec4(0,0,0,1));
        label->setCharacterSize(16.0f + ((float)rank)/3.0f);


        xform->addChild(label);
        xform->addCullCallback(app._horizonCull.get());
        group->addChild(xform);
        //break;
    }
    fs->close();
    return group;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator(new EarthManipulator(arguments));
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    GLUtils::setGlobalDefaults(viewer.getCamera()->getOrCreateStateSet());
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        App app;
        app._font = osgText::readRefFontFile("arial.ttf");
        app._cm = new CalloutManager();
        app._horizonCull = new HorizonCullCallback();

        MapNode::get(node)->addChild(app._cm);
        MapNode::get(node)->addChild(loadData(app));

        viewer.setSceneData( node );
        viewer.run();
    }
    return 0;
}