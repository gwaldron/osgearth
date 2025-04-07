/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_Root"
#include "KML_Document"
#include "KML_Folder"
#include "KML_PhotoOverlay"
#include "KML_ScreenOverlay"
#include "KML_GroundOverlay"
#include "KML_NetworkLink"
#include "KML_Placemark"
#include "KML_NetworkLinkControl"

using namespace osgEarth_kml;

void 
KML_Root::scan( xml_node<>* node, KMLContext& cx )
{
    for_features( scan, node, cx );
    for_one( NetworkLinkControl, scan, node, cx );
}

void
KML_Root::scan2( xml_node<>* node, KMLContext& cx )
{
    for_features( scan2, node, cx );
    for_one( NetworkLinkControl, scan2, node, cx );
}

void
KML_Root::build( xml_node<>* node, KMLContext& cx )
{
    for_features( build, node, cx );
    for_one( NetworkLink, build, node, cx );
}
