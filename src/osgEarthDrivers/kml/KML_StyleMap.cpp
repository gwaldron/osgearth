/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_StyleMap"

using namespace osgEarth_kml;

void
KML_StyleMap::scan2( xml_node<>* node, KMLContext& cx )
{
    xml_node<>* pair = node->first_node("pair", 0, false);
    if ( pair )
    {
        const std::string& url = getValue(pair, "styleurl");
        if ( !url.empty() )
        {
            const Style* style = cx._sheet->getStyle( url );
            if ( style )
            {
                Style aliasStyle = *style;
                aliasStyle.setName( getValue(node, "id") );
                cx._sheet->addStyle( aliasStyle );
            }
        }
    }
}
