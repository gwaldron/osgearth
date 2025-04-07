/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_Style"
#include "KML_IconStyle"
#include "KML_LabelStyle"
#include "KML_LineStyle"
#include "KML_PolyStyle"

using namespace osgEarth_kml;

void
KML_Style::scan( xml_node<>* node, KMLContext& cx )
{
    Style style( getValue(node, "id") );

    KML_IconStyle icon;
    icon.scan( node->first_node("iconstyle", 0, false), style, cx );

    KML_LabelStyle label;
    label.scan( node->first_node("labelstyle", 0, false), style, cx );

    KML_LineStyle line;
    line.scan( node->first_node("linestyle", 0, false), style, cx );

    KML_PolyStyle poly;
    poly.scan( node->first_node("polystyle", 0, false), style, cx );

    cx._sheet->addStyle( style );

    cx._activeStyle = style;
}
