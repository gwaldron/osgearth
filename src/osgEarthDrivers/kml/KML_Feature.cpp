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
#include "KML_Feature"
#include "KML_Style"
#include "KML_StyleMap"
#include <osg/UserDataContainer>
#include <osg/ValueObject>
#include <osgEarth/Viewpoint>

using namespace osgEarth_kml;
using namespace osgEarth;

void
KML_Feature::scan( xml_node<>* node, KMLContext& cx )
{
    KML_Object::scan(node, cx);
    for_many( Style, scan, node, cx );
    for_many( StyleMap, scan, node, cx );
}

void
KML_Feature::scan2( xml_node<>* node, KMLContext& cx )
{
    KML_Object::scan2(node, cx);
    for_many( Style, scan2, node, cx );
    for_many( StyleMap, scan2, node, cx );
}

void
KML_Feature::build( xml_node<>* node, KMLContext& cx, osg::Node* working )
{
    KML_Object::build(node, cx, working);

    // subclass feature is built; now add feature level data if available
    if ( working )
    {
        // parse the visibility to show/hide the item by default:
		std::string visibility = getValue(node, "visibility");
        if ( !visibility.empty() )
            working->setNodeMask( as<int>(visibility, 1) == 1 ? ~0 : 0 );

        // parse a "LookAt" element (stores a viewpoint)
        AnnotationData* anno = getOrCreateAnnotationData(working);
        
        anno->setName( getValue(node, "name") );
        anno->setDescription( getValue(node, "description") );

        xml_node<>* lookat = node->first_node("lookat", 0, false);
        if ( lookat )
        {
            Viewpoint vp;

            vp.focalPoint() = GeoPoint(
                cx._srs.get(),
				as<double>(getValue(lookat, "longitude"), 0.0),
				as<double>(getValue(lookat, "latitude"), 0.0),
				as<double>(getValue(lookat, "altitude"), 0.0),
                ALTMODE_ABSOLUTE );

            vp.heading() =  as<double>(getValue(lookat, "heading"), 0.0);
            vp.pitch()   = -as<double>(getValue(lookat, "tilt"), 45.0),
            vp.range()   =  as<double>(getValue(lookat, "range"), 10000.0);

            anno->setViewpoint( vp );
        }

        xml_node<>* timespan = node->first_node("timespan", 0, false);
        if ( timespan )
        {
            DateTimeRange range;

            std::string begin = getValue(timespan, "begin");
            if ( !begin.empty() )
            {
                range.begin() = DateTime(begin);
            }

            std::string end = getValue(timespan, "end");
            if ( !end.empty() )
            {
                range.end() = DateTime(end);
            }

            anno->setDateTimeRange( range );
        }

        xml_node<>* extdata = node->first_node("extendeddata", 0, false);
        if ( extdata )
        {
            xml_node<>* data = extdata->first_node("data", 0, false);
            if ( data )
            {
                working->setUserValue(getValue(data, "name"), getValue(data, "value"));			    
            }
        }
    }
}
