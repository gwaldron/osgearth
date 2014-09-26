/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
KML_IconStyle::scan( const Config& conf, Style& style, KMLContext& cx )
{
    if ( !conf.empty() )
    {
        IconSymbol* icon = style.getOrCreate<IconSymbol>();

        // Icon/Href or just Icon are both valid
        std::string iconHref = conf.child("icon").value("href");
        if ( iconHref.empty() )
            iconHref = conf.value("icon");

        if ( !iconHref.empty() )
            icon->url() = StringExpression( iconHref, URIContext(conf.referrer()) );

        // see: https://developers.google.com/kml/documentation/kmlreference#headingdiagram
        if ( conf.hasValue("heading") )
            icon->heading() = NumericExpression( conf.value("heading") );

        float finalScale = *cx._options->iconBaseScale();

        if ( conf.hasValue("scale") )
            icon->scale() = NumericExpression( conf.value("scale") );
    }
}
