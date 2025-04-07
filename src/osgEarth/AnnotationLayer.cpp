/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/AnnotationLayer>
#include <osgEarth/AnnotationRegistry>
#include <osgEarth/Registry>

using namespace osgEarth;

REGISTER_OSGEARTH_LAYER(annotations, AnnotationLayer);

//...................................................................

Config
AnnotationLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    return conf;
}

void
AnnotationLayer::Options::fromConfig(const Config& conf)
{
    // NOP
}

//...................................................................

void
AnnotationLayer::init()
{
    VisibleLayer::init();

    _root = new osg::Group();

    deserialize();
}

osg::Node*
AnnotationLayer::getNode() const
{
    return _root.get();
}

osg::Group*
AnnotationLayer::getGroup() const
{
    return _root.get();
}

void
AnnotationLayer::addChild(osg::Node* node)
{
    _root->addChild(node);
}

void
AnnotationLayer::deserialize()
{
    // reset:
    _root->removeChildren(0, _root->getNumChildren());

    // deserialize from the options:
    osg::Group* group = 0L;
    AnnotationRegistry::instance()->create(nullptr, options().getConfig(), getReadOptions(), group);
    if (group)
    {
        _root->addChild(group);
    }
}
