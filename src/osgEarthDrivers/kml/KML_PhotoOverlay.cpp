/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_PhotoOverlay"

using namespace osgEarth_kml;

void
KML_PhotoOverlay::scan( xml_node<>* node, KMLContext& cx )
{
    KML_Overlay::scan( node, cx );
}

void
KML_PhotoOverlay::build( xml_node<>* node, KMLContext& cx )
{
    KML_Overlay::build( node, cx, 0L );
}
