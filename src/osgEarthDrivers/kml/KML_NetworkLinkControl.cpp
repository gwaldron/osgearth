/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_NetworkLinkControl"

using namespace osgEarth_kml;

void
KML_NetworkLinkControl::scan( xml_node<>* node, KMLContext& cx )
{
    KML_Object::scan( node, cx );
}

void
KML_NetworkLinkControl::build( xml_node<>* node, KMLContext& cx )
{
    KML_Object::build( node, cx, 0L );
}
