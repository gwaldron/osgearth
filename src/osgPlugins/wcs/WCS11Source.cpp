/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include "WCS11Source.h"
#include <osgEarth/HTTPClient>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osg/Notify>
#include <osgDB/Registry>
#include <iostream>
#include <stdlib.h>

using namespace osgEarth;

#define PROPERTY_MAP_CONFIG     "map_config"
#define PROPERTY_URL            "url"
#define PROPERTY_IDENTIFIER     "identifier"
#define PROPERTY_FORMAT         "format"
#define PROPERTY_ELEVATION_UNIT "elevation_unit"
#define PROPERTY_TILE_SIZE      "tile_size"
#define PROPERTY_SRS            "srs"

WCS11Source::WCS11Source( const osgDB::ReaderWriter::Options* options ) :
tile_size(16),
map_config(0),
profile( Profile::GLOBAL_GEODETIC )
{
    if ( options->getPluginData( PROPERTY_MAP_CONFIG ))
         map_config = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );

    if ( options->getPluginData( PROPERTY_URL ) )
        url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

    if ( options->getPluginData( PROPERTY_IDENTIFIER ) )
        identifier = std::string( (const char*)options->getPluginData( PROPERTY_IDENTIFIER ) );

    if ( options->getPluginData( PROPERTY_FORMAT ) )
        cov_format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

    if ( options->getPluginData( PROPERTY_ELEVATION_UNIT ) )
         elevation_unit = std::string( (const char*)options->getPluginData( PROPERTY_ELEVATION_UNIT ) );

    if ( options->getPluginData( PROPERTY_TILE_SIZE ) )
        tile_size = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_SIZE ), 32 );

    if ( options->getPluginData( PROPERTY_SRS ) )
        srs = std::string( (const char*)options->getPluginData( PROPERTY_SRS ) );

    //TODO: Read GetCapabilities and determine everything from that..


    //std::string host = "192.168.0.101";
    //if ( ::getenv( "OSGEARTH_HOST" ) )
    //    host = std::string( ::getenv( "OSGEARTH_HOST" ) );

    //std::stringstream buf;
    //buf << "http://" << host << "/cgi-bin/mapserv";
    //prefix     = buf.str();

    //map_file   = "/var/www/maps/srtm.map";
    //coverage   = "srtm";
    //cov_format = "GEOTIFFINT16";
    //osg_format = "tif";

    if ( cov_format.empty() )
        cov_format = "image/GeoTIFF";

    osg_format = "tif";
}


const Profile&
WCS11Source::getProfile() const
{
    return profile;
}


std::string
WCS11Source::getExtension() const {
    return "tif";
}


osg::Image*
WCS11Source::createImage( const TileKey* key)
{
    osg::ref_ptr<HTTPRequest> request = createRequest( key );

    osg::notify(osg::INFO) << "[osgEarth::WCS1.1] URL = " << request->getURL() << std::endl;

    double lon0,lat0,lon1,lat1;
    key->getGeoExtents( lon0, lat0, lon1, lat1 );

    // download the data
    HTTPClient client;
    osg::ref_ptr<HTTPResponse> response = client.get( request.get() );
    if ( !response.valid() )
    {
        osg::notify(osg::WARN) << "[osgEarth::WCS1.1] WARNING: HTTP request failed" << std::endl;
        return NULL;
    }

    unsigned int part_num = response->getNumParts() > 1? 1 : 0;
    std::istream& input_stream = response->getPartStream( part_num );

    //TODO: un-hard-code TIFFs
    osgDB::ReaderWriter* reader = osgDB::Registry::instance()->getReaderWriterForExtension( "tiff" );

    if ( !reader )
    {
        osg::notify(osg::NOTICE) << "[osgEarth::WCS1.1] WARNING: no reader for \"tiff\"" << std::endl;
        return NULL;
    }

    osgDB::ReaderWriter::ReadResult result = reader->readImage( input_stream );
    if ( !result.success() )
    {
        osg::notify(osg::NOTICE) << "[osgEarth::WCS1.1] WARNING: readImage() failed for Reader " << reader->getName() << std::endl;
        return NULL;
    }

    osg::Image* image = result.getImage();
    if ( image ) image->ref();
    return image;
}


osg::HeightField*
WCS11Source::createHeightField( const TileKey* key )
{
    osg::HeightField* field = NULL;

    osg::ref_ptr<osg::Image> image = createImage( key );
    if ( image.valid() )
    {        
        ImageToHeightFieldConverter conv;
        conv.setRemoveNoDataValues( true );
        field = conv.convert( image.get() );
    }

    return field;
}

/*
http://server/ArcGIS/services/WorldElevation/MapServer/WCSServer
    ?SERVICE=WCS
    &VERSION=1.1.0
    &REQUEST=GetCoverage
    &IDENTIFIER=1
    &FORMAT=image/GeoTIFF
    &BOUNDINGBOX=-180,-90,0,90,urn:ogc:def:crs:EPSG::4326  // (sic - coord ordering bug in ESRI)
    &RangeSubset=Field_1:bilinear[Band[1]]
    &GridBaseCRS=urn:ogc:def:crs:EPSG::4326
    &GridCS=urn:ogc:def:crs:EPSG::4326
    &GridType=urn:ogc:def:method:WCS:1.1:2dGridIn2dCrs
    &GridOrigin=-180,90
    &GridOffsets=6,-6
*/


HTTPRequest*
WCS11Source::createRequest( const TileKey* key ) const
{
    std::stringstream buf;

    double lon_min, lat_min, lon_max, lat_max;
    key->getGeoExtents( lon_min, lat_min, lon_max, lat_max );

    int lon_samples = tile_size;
    int lat_samples = tile_size;
    double lon_interval = (lon_max-lon_min)/(double)(lon_samples-1);
    double lat_interval = (lat_max-lat_min)/(double)(lat_samples-1);

    HTTPRequest* req = new HTTPRequest( url );

    req->addParameter( "SERVICE",    "WCS" );
    req->addParameter( "VERSION",    "1.1.0" );
    req->addParameter( "REQUEST",    "GetCoverage" );
    req->addParameter( "IDENTIFIER", identifier );
    req->addParameter( "FORMAT",     cov_format );

    req->addParameter( "GridBaseCRS", "urn:ogc:def:crs:EPSG::4326" );
    req->addParameter( "GridCS",      "urn:ogc:def:crs:EPSG::4326" );
    req->addParameter( "GridType",    "urn:ogc:def:method:WCS:1.1:2dGridIn2dCrs" );

    // IMPORTANT NOTE:
    //   For WCS1.1+, the BOUNDINGBOX for geographic CRS's (like WGS84) are expressed
    //   at minlat,minlon,maxlat,maxlon instead of the usual minx,miny,maxx,maxy.
    //   So we will somehow need to figure out whether the CRS is geographic.
    //
    // MORE IMPORTANT NOTE:
    //   ESRI's ArcGIS WCS Server doesn't obey the above rule. Their server expects
    //   minx,miny,maxx,maxy no matter what ...

    // Hack to guess whether it's an ArcGIS Server:
    bool use_legacy_geog_bbox_encoding = ::strstr( url.c_str(), "/MapServer/WCSServer" ) != NULL;

    buf.str("");
    if ( use_legacy_geog_bbox_encoding )
        buf << lon_min << "," << lat_min << "," << lon_max << "," << lat_max;
    else
        buf << lat_min << "," << lon_min << "," << lat_max << "," << lon_max;
    buf << ",urn:ogc:def:crs:EPSG::4326";
    req->addParameter( "BOUNDINGBOX", buf.str() );

    buf.str("");
    buf << lon_min << "," << (lat_min + (lat_max-lat_min));           // note: top-down
    req->addParameter( "GridOrigin", buf.str() );

    buf.str("");
    buf << lon_interval << "," << -lat_interval;   // note: top-down
    req->addParameter( "GridOffsets", buf.str() );

    buf.str("");
    buf << "Field_1:bilinear[Band[1]]";            // TODO: paramaterize: "bicubic", "bilinear", "nearest"
    req->addParameter( "RangeSubset", buf.str() );

    return req;
}