/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include <osgEarth/GeometryUtils>
#include <osgEarth/OgrUtils>

using namespace osgEarth;

std::string
osgEarth::GeometryUtils::geometryToWKT( const Geometry* geometry )
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
osgEarth::GeometryUtils::geometryToIsoWKT( const Geometry* geometry )
{
    OGRGeometryH g = OgrUtils::createOgrGeometry( geometry );
    std::string result;
    if (g)
    {
        char* buf;   
        if (OGR_G_ExportToIsoWkt( g, &buf ) == OGRERR_NONE)
        {
            result = std::string(buf);
            OGRFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string 
osgEarth::GeometryUtils::geometryToGeoJSON( const Geometry* geometry )
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
osgEarth::GeometryUtils::geometryFromGeoJSON(const std::string& geojson, bool rewindPolygons)
{
    Geometry* result = 0L;
    OGRGeometryH g = OGR_G_CreateGeometryFromJson(geojson.c_str());
    if ( g )
    {
        result = OgrUtils::createGeometry( g, rewindPolygons);
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string 
osgEarth::GeometryUtils::geometryToKML( const Geometry* geometry )
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
osgEarth::GeometryUtils::geometryToGML( const Geometry* geometry )
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
osgEarth::GeometryUtils::geometryFromWKT( const std::string& wkt, bool rewindPolygons)
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

    Geometry* output = 0L;
    if ( type != wkbNone )
    {
        OGRGeometryH geom = OGR_G_CreateGeometry( type );
        if ( geom )
        {
            char* ptr = (char*)wkt.c_str();
            if ( OGRERR_NONE == OGR_G_ImportFromWkt( geom, &ptr ) )
            {
                output = OgrUtils::createGeometry( geom, rewindPolygons);
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
osgEarth::GeometryUtils::getGeometryArea( const Geometry* geometry )
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
