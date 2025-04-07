/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_Object"

using namespace osgEarth_kml;

void
KML_Object::build( xml_node<>* node, KMLContext& cx, osg::Node* working )
{
    //todo - read ID
}

AnnotationData*
KML_Object::getOrCreateAnnotationData( osg::Node* node )
{
    AnnotationData* data = dynamic_cast<AnnotationData*>( node->getUserData() );
    if ( !data ) {
        data = new AnnotationData();
        node->setUserData( data );
    }
    return data;
}

