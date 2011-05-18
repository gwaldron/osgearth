/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include <osgEarthFeatures/OgrUtils>
#include <osgEarthFeatures/GeometryUtils>

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

std::string osgEarth::Features::geometryToWkt( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        if (OGR_G_ExportToWkt( g, &buf ) == OGRERR_NONE)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string osgEarth::Features::geometryToJson( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        buf = OGR_G_ExportToJson( g );
        if (buf)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string osgEarth::Features::geometryToKml( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        buf = OGR_G_ExportToKML( g, 0);
        if (buf)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string osgEarth::Features::geometryToGml( Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        buf = OGR_G_ExportToGML( g );
        if (buf)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}