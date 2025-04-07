/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_Polygon"
#include "KML_LinearRing"
#include <iterator>

using namespace osgEarth_kml;

void
KML_Polygon::parseStyle(xml_node<>* node, KMLContext& cx, Style& style)
{
    KML_Geometry::parseStyle(node, cx, style);

    // need at minimum a poly symbol.
    if ( !style.has<PolygonSymbol>() )
    {
        style.getOrCreate<PolygonSymbol>()->fill().mutable_value().color() = osg::Vec4f(1,1,1,1);
    }
}

void
KML_Polygon::parseCoords( xml_node<>* node, KMLContext& cx )
{
    Polygon* poly = new Polygon();

    xml_node<>* outer = node->first_node("outerboundaryis", 0, false);
    if ( outer )
    {
        xml_node<>* outerRing = outer->first_node("linearring", 0, false);
        if ( outerRing )
        {
            KML_LinearRing outer;
            outer.parseCoords( outerRing, cx );
            if ( outer._geom.valid() )
            {
                static_cast<Ring*>(outer._geom.get())->rewind( Ring::ORIENTATION_CCW );
                poly->reserve( outer._geom->size() );
                std::copy( outer._geom->begin(), outer._geom->end(), std::back_inserter(*poly) );
            }
        }

		for (xml_node<>* n = node->first_node("innerboundaryis", 0, false); n; n = n->next_sibling("innerboundaryis", 0, false))
		{
			xml_node<>* innerRing = n->first_node("linearring", 0, false);
			if ( innerRing )
			{
				KML_LinearRing inner;
				inner.parseCoords( innerRing, cx );
				if ( inner._geom.valid() )
				{
					Geometry* innerGeom = inner._geom.get();
					static_cast<Ring*>(innerGeom)->rewind( Ring::ORIENTATION_CW );
					poly->getHoles().push_back( dynamic_cast<Ring*>(innerGeom) );
				}
			}
		}
    }

    _geom = poly;
}
