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
#include "KML_Root"
#include "KML_Document"
#include "KML_Folder"
#include "KML_PhotoOverlay"
#include "KML_ScreenOverlay"
#include "KML_GroundOverlay"
#include "KML_NetworkLink"
#include "KML_Placemark"
#include "KML_NetworkLinkControl"

using namespace osgEarth_kml;

void 
KML_Root::scan( const Config& conf, KMLContext& cx )
{
    for_features( scan, conf, cx );
    for_one( NetworkLinkControl, scan, conf, cx );
}

void
KML_Root::scan2( const Config& conf, KMLContext& cx )
{
    for_features( scan2, conf, cx );
    for_one( NetworkLinkControl, scan2, conf, cx );
}

void
KML_Root::build( const Config& conf, KMLContext& cx )
{
    for_features( build, conf, cx );
    for_one( NetworkLink, build, conf, cx );
}
