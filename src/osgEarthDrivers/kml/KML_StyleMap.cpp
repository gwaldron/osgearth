/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include "KML_StyleMap"

using namespace osgEarth_kml;

void
KML_StyleMap::scan2( const Config& conf, KMLContext& cx )
{
    const Config& pair = conf.child("pair");
    if ( !pair.empty() )
    {
        const std::string& url = pair.value("styleurl" );
        if ( !url.empty() )
        {
            const Style* style = cx._sheet->getStyle( url );
            if ( style )
            {
                Style aliasStyle = *style;
                aliasStyle.setName( conf.value("id") );
                cx._sheet->addStyle( aliasStyle );
            }
        }
    }
}
