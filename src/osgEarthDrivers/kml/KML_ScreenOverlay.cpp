/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_ScreenOverlay"

using namespace osgEarth_kml;

void
KML_ScreenOverlay::scan( xml_node<>* node, KMLContext& cx )
{
    KML_Overlay::scan( node, cx );
}

void
KML_ScreenOverlay::build( xml_node<>* node, KMLContext& cx )
{
    //todo
    //KML_Overlay::build( conf, cx, 0L );
}
