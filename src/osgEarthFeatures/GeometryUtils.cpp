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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthFeatures/OgrUtils>

using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

std::string
osgEarth::Features::GeometryUtils::geometryToWKT( const Geometry* geometry )
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

std::string 
osgEarth::Features::GeometryUtils::geometryToGeoJSON( const Geometry* geometry )
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

Geometry*
osgEarth::Features::GeometryUtils::geometryFromGeoJSON(const std::string& geojson)
{
    Geometry* result = 0L;
    OGRGeometryH g = OGR_G_CreateGeometryFromJson(geojson.c_str());
    if ( g )
    {
        result = OgrUtils::createGeometry( g );
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string 
osgEarth::Features::GeometryUtils::geometryToKML( const Geometry* geometry )
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

std::string 
osgEarth::Features::GeometryUtils::geometryToGML( const Geometry* geometry )
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

Geometry*
osgEarth::Features::GeometryUtils::geometryFromWKT( const std::string& wkt )
{       
    OGRwkbGeometryType type = 
        startsWith( wkt, "POINT" ) ? wkbPoint :
        startsWith( wkt, "LINESTRING" ) ? wkbLineString :
        startsWith( wkt, "POLYGON" ) ? wkbPolygon :
        startsWith( wkt, "MULTIPOINT" ) ? wkbMultiPoint :
        startsWith( wkt, "MULTILINESTRING" ) ? wkbMultiLineString :
        startsWith( wkt, "MULTIPOLYGON" ) ? wkbMultiPolygon :
        startsWith( wkt, "GEOMETRYCOLLECTION" ) ? wkbGeometryCollection :
        wkbNone;

    Symbology::Geometry* output = 0L;
    if ( type != wkbNone )
    {
        OGRGeometryH geom = OGR_G_CreateGeometry( type );
        if ( geom )
        {
            char* ptr = (char*)wkt.c_str();
            if ( OGRERR_NONE == OGR_G_ImportFromWkt( geom, &ptr ) )
            {
                output = OgrUtils::createGeometry( geom );
                OGR_G_DestroyGeometry( geom );
            }
            else
            {
                OE_NOTICE
                    << "OGR Feature Source: malformed WKT geometry" << std::endl;
            }
        }
    }
    return output;
}

double
osgEarth::Features::GeometryUtils::getGeometryArea( const Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    
    double result = 0.0;
    if (g)
    {
        result = OGR_G_Area( g );
        OGR_G_DestroyGeometry( g );
    }
    return result;
}
