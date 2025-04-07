/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/InstanceResource>

#define LC "[InstanceResource] "

using namespace osgEarth;

//---------------------------------------------------------------------------

InstanceResource::InstanceResource( const Config& conf ) :
Resource( conf )
{
    mergeConfig( conf );
}

void
InstanceResource::mergeConfig( const Config& conf )
{
    conf.get( "url", _uri );
}

Config
InstanceResource::getConfig() const
{
    Config conf = Resource::getConfig();
    conf.key() = "instance";

    conf.set( "url", _uri );

    return conf;
}

osg::Node*
InstanceResource::createNode( const osgDB::Options* dbOptions ) const
{
    if (_node.valid())
        return _node.get();
    else
        return createNodeFromURI( _uri.value(), dbOptions );
}
