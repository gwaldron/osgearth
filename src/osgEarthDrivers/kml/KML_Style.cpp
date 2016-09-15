/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
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
