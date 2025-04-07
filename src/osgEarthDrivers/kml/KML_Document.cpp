/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
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
