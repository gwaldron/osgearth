/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/GeometryUtils>
#include <osgEarth/OgrUtils>
#include <cpl_conv.h>

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
            CPLFree( buf );
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
            CPLFree( buf );
        }
        OGR_G_DestroyGeometry( g );
    }
    return result;
}

std::string
osgEarth::GeometryUtils::geometryToGeoJSON(const Geometry* input, const SpatialReference* srs)
{
    const Geometry* geometry = input;

    if (srs && !srs->isGeographic())
    {
        // GeoJSON is ALWAYS in geographic coordinates.
        auto* geog = srs->getGeographicSRS();
        auto* cloned = input->clone();
        GeometryIterator(cloned, true).forEach([&](Geometry* part)
            {
                srs->transform(part->asVector(), geog);
            });
        geometry = cloned;
    }

    OGRGeometryH g = OgrUtils::createOgrGeometry(geometry);
    std::string result;
    if (g)
    {
        char* buf;
        buf = OGR_G_ExportToJson(g);
        if (buf)
        {
            result = std::string(buf);
            CPLFree(buf);
        }
        OGR_G_DestroyGeometry(g);
    }

    if (geometry != input)
    {
        delete geometry;
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
            CPLFree( buf );
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
            CPLFree( buf );
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
