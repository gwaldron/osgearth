/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_MultiGeometry"

using namespace osgEarth_kml;

void
KML_MultiGeometry::parseCoords( xml_node<>* node, KMLContext& cx )
{
    _geom = new MultiGeometry();
    //KML_Geometry::parseCoords( conf, cx );
}
