/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_Point"

using namespace osgEarth_kml;

void
KML_Point::parseCoords( xml_node<>* node, KMLContext& cx )
{
    _geom = new Point();
    KML_Geometry::parseCoords( node, cx );
}
