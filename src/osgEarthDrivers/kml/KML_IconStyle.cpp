/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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

void
KML_IconStyle::scan( const Config& conf, Style& style, KMLContext& cx )
{
    if ( !conf.empty() )
    {
        MarkerSymbol* marker = style.getOrCreate<MarkerSymbol>();

        // Icon/Href or just Icon are both valid
        std::string iconHref = conf.child("icon").value("href");
        if ( iconHref.empty() )
            iconHref = conf.value("icon");

        if ( !iconHref.empty() )
        {
            marker->url() = StringExpression( iconHref );
            marker->url()->setURIContext( URIContext(conf.referrer()) );
        }

        float finalScale = *cx._options->iconBaseScale();

        optional<float> scale;
        conf.getIfSet( "scale", scale );
        if ( scale.isSet() )
            finalScale *= scale.value();

        if ( finalScale != 1.0f )
            marker->scale() = NumericExpression( finalScale );
    }
}
