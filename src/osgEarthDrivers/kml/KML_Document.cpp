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
#include "KML_Document"
#include "KML_Schema"
#include "KML_Folder"
#include "KML_PhotoOverlay"
#include "KML_ScreenOverlay"
#include "KML_GroundOverlay"
#include "KML_NetworkLink"
#include "KML_Placemark"

using namespace osgEarth_kml;

void
KML_Document::scan( xml_node<>* node, KMLContext& cx )
{
    KML_Container::scan(node, cx);
    for_many    ( Schema, scan, node, cx );
    for_features( scan, node, cx );
}

void
KML_Document::scan2( xml_node<>* node, KMLContext& cx )
{
    KML_Container::scan2(node, cx);
    for_many    ( Schema, scan2, node, cx );
    for_features( scan2, node, cx );
}

void
KML_Document::build( xml_node<>* node, KMLContext& cx )
{
    // creates an empty group and pushes it on the stack.
    osg::Group* group = new osg::Group();
    cx._groupStack.top()->addChild( group );
    cx._groupStack.push( group );

    KML_Container::build(node, cx, group);
    for_features(build, node, cx);

    cx._groupStack.pop();
}
