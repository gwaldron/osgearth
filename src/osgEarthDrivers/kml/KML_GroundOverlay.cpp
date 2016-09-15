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
#include "KML_GroundOverlay"
#include "KML_Geometry"
#include <osgEarthAnnotation/ImageOverlay>

using namespace osgEarth_kml;
using namespace osgEarth::Annotation;

void
KML_GroundOverlay::scan( xml_node<>* node, KMLContext& cx )
{
    KML_Overlay::scan( node, cx );
}

void
KML_GroundOverlay::build( xml_node<>* node, KMLContext& cx )
{
    // the URL of the overlay image
	xml_node<>* icon = node->first_node("icon", 0, false);
	std::string href;
	if (icon)
	{
		href = getValue(icon, "href");
	}

    if ( href.empty() ) {
        OE_WARN << LC << "GroundOverlay missing required Icon element" << std::endl;
        return;
    }

    ImageOverlay* im = 0L;

    // the extent of the overlay image
	xml_node<>* llb = node->first_node("latlonbox", 0, false);
	xml_node<>* llq = node->first_node("gx:latlonquad", 0, false);
    if ( llb)
    {
        double north = as<double>(getValue(llb, "north"), 0.0);
        double south = as<double>(getValue(llb, "south"), 0.0);
        double east  = as<double>(getValue(llb, "east"), 0.0);;
        double west  = as<double>(getValue(llb, "west"), 0.0);;
        Angular rotation( -as<double>(getValue(llb, "rotation"), 0.0), Units::DEGREES );

		osg::ref_ptr<osg::Image> image = URI(href, cx._referrer).readImage().getImage();
		if ( !image.valid() )
        {
            OE_WARN << LC << "GroundOverlay failed to read image from " << href << std::endl;
            return;
        }

        im = new ImageOverlay( cx._mapNode, image.get() );
        im->setBoundsAndRotation( Bounds(west, south, east, north), rotation );
        cx._groupStack.top()->addChild( im );
    }

    else if ( llq )
    {
        KML_Geometry g;
        Style style;
        g.buildChild( llq, cx, style );
        if ( g._geom.valid() && g._geom->size() >= 4 )
        {
            osg::ref_ptr<osg::Image> image = URI(href, cx._referrer).readImage().getImage();
		    if ( !image.valid() )
            {
                OE_WARN << LC << "GroundOverlay failed to read image from " << href << std::endl;
                return;
            }

            const Geometry& p = *(g._geom.get());
            im = new ImageOverlay( cx._mapNode, image.get() );
            im->setCorners( 
                osg::Vec2d( p[0].x(), p[0].y() ),
                osg::Vec2d( p[1].x(), p[1].y() ),
                osg::Vec2d( p[3].x(), p[3].y() ),
                osg::Vec2d( p[2].x(), p[2].y() ) );
            cx._groupStack.top()->addChild( im );
        }
    }

    else {
        OE_WARN << LC << "GroundOverlay missing required LatLonBox/gx:LatLonQuad element" << std::endl;
        return;
    }


    // superclass build always called last
    KML_Overlay::build( node, cx, im );
}
