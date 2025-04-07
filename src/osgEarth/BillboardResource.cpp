/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/BillboardResource>

#define LC "[BillboardResource] "

using namespace osgEarth;


BillboardResource::BillboardResource(const Config& conf) :
InstanceResource( conf )
{
    mergeConfig( conf );
}

void
BillboardResource::mergeConfig(const Config& conf)
{
    conf.get( "width", _width );
    conf.get( "height", _height );
}

Config
BillboardResource::getConfig() const
{
    Config conf = InstanceResource::getConfig();
    conf.key() = "billboard";
    conf.set( "width", _width );
    conf.set( "height", _height );
    //nop
    return conf;
}

osg::Node*
BillboardResource::createNodeFromURI(const URI& uri, const osgDB::Options* dbOptions) const
{
    // unsupported atm.
    osg::Node* node = 0L;
    return node;
}
