/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/Resource>

using namespace osgEarth;

Resource::Resource( const Config& conf )
{
    mergeConfig( conf );
}

void
Resource::mergeConfig( const Config& conf )
{
    _name = conf.value("name");
    addTags( conf.value("tags") );
}

Config
Resource::getConfig() const
{
    Config conf( "resource" );
    conf.set("name", _name );

    std::string tags = tagString();
    if ( !tags.empty() )
        conf.add( "tags", tags );

    return conf;
}
