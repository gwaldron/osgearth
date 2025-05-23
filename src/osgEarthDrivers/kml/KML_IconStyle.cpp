/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_IconStyle"
#include <osgEarth/IconSymbol>

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
            icon->url().mutable_value().setLiteral(iconHref);
            icon->url().mutable_value().setURIContext(URIContext(cx._referrer));
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
