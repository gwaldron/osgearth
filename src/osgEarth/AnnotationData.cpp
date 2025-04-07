/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/AnnotationData>

using namespace osgEarth;

AnnotationData::AnnotationData() :
_viewpoint( 0L ),
_priority ( 0.0f )
{
    //nop
}

AnnotationData::AnnotationData(const Config& conf) :
_viewpoint( 0L ),
_priority ( 0.0f )
{
    mergeConfig(conf);
}

AnnotationData::~AnnotationData()
{
    if ( _viewpoint )
        delete _viewpoint;
}

void
AnnotationData::mergeConfig(const Config& conf)
{
    _name        = conf.value("name");
    _description = conf.value("description");
    _priority    = conf.value<float>("priority", _priority);

    if ( conf.hasValue("viewpoint") )
    {
        _viewpoint = new Viewpoint( conf.value("viewpoint") );
    }
}

Config
AnnotationData::getConfig() const
{
    Config conf("annotation_data");

    if ( !_name.empty() )
        conf.add("name", _name);
    if ( !_description.empty() )
        conf.add("description", _description);
    if ( _priority != 0.0f )
        conf.add("priority", _priority );
    if ( _viewpoint )
        conf.add( "viewpoint", _viewpoint->getConfig() );

    return conf;
}

