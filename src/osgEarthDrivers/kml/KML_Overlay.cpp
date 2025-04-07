/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_Overlay"

using namespace osgEarth_kml;

void
KML_Overlay::scan( xml_node<>* node, KMLContext& cx )
{
    KML_Feature::scan( node, cx );
}

void
KML_Overlay::build( xml_node<>* node, KMLContext& cx, osg::Node* working )
{
    KML_Feature::build( node, cx, working );
}
