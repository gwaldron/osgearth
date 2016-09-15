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
#include "KML_IconStyle"
#include <osgEarthSymbology/IconSymbol>

using namespace osgEarth_kml;

void
KML_IconStyle::scan( xml_node<>* node, Style& style, KMLContext& cx )
{
    if ( node )
    {
        IconSymbol* icon = style.getOrCreate<IconSymbol>();

        // Icon/Href or just Icon are both valid
		std::string iconHref;
		xml_node<>* iconNode = node->first_node("icon", 0, false);
		if (iconNode)
		{
			iconHref = getValue(iconNode, "href");
			if ( iconHref.empty() )
				iconHref = getValue(node, "icon");
		}

        if ( !iconHref.empty() )
        {
            // We set a literal here to avoid filenames with spaces being evaluated.
            icon->url()->setLiteral(iconHref);
            icon->url()->setURIContext(URIContext(cx._referrer));
        }
			
        // see: https://developers.google.com/kml/documentation/kmlreference#headingdiagram
		std::string heading = getValue(node, "heading");
        if ( !heading.empty() )
            icon->heading() = NumericExpression( heading );

        float finalScale = *cx._options->iconBaseScale();

		std::string scale = getValue(node, "scale");
        if ( !scale.empty() )
        {
            icon->scale() = NumericExpression(NumericExpression( scale ).eval() * finalScale);
        }
        else
        {
            icon->scale() = NumericExpression(finalScale);
        }
    }
}
