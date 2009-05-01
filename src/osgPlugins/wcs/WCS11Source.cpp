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
#include <osgEarth/Registry>
#include <osg/Notify>
#include <osgDB/Registry>
#include <iostream>
#include <stdlib.h>

using namespace osgEarth;

#define PROPERTY_URL            "url"
#define PROPERTY_IDENTIFIER     "identifier"
#define PROPERTY_FORMAT         "format"
#define PROPERTY_ELEVATION_UNIT "elevation_unit"
#define PROPERTY_TILE_SIZE      "tile_size"
#define PROPERTY_SRS            "srs"
#define PROPERTY_DEFAULT_TILE_SIZE "default_tile_size"
#define PROPERTY_RANGE_SUBSET   "range_subset"

WCS11Source::WCS11Source( const osgDB::ReaderWriter::Options* options ) :
TileSource( options ),
_tile_size(16)
{
    if ( options->getPluginData( PROPERTY_URL ) )
        _url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

    if ( options->getPluginData( PROPERTY_IDENTIFIER ) )
        _identifier = std::string( (const char*)options->getPluginData( PROPERTY_IDENTIFIER ) );

    if ( options->getPluginData( PROPERTY_FORMAT ) )
        _cov_format = std::string( (const char*)options->getPluginData( PROPERTY_FORMAT ) );

    if ( options->getPluginData( PROPERTY_ELEVATION_UNIT ) )
        _elevation_unit = std::string( (const char*)options->getPluginData( PROPERTY_ELEVATION_UNIT ) );

    //Try to read the tile size
    if ( options->getPluginData( PROPERTY_TILE_SIZE ) )
    {
        _tile_size = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_SIZE ), 32 );
    }
    //If the tile size wasn't specified, use the default tile size if it was specified
    else if ( options->getPluginData( PROPERTY_DEFAULT_TILE_SIZE ) )
    {
        _tile_size = as<int>( (const char*)options->getPluginData( PROPERTY_DEFAULT_TILE_SIZE ), 32 );
    }

    if ( options->getPluginData( PROPERTY_SRS ) )
        _srs = std::string( (const char*)options->getPluginData( PROPERTY_SRS ) );

    if (options->getPluginData( PROPERTY_RANGE_SUBSET ) )
        _range_subset = std::string( (const char*)options->getPluginData( PROPERTY_RANGE_SUBSET ) );

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

    if ( _cov_format.empty() )
        _cov_format = "image/GeoTIFF";

    _osg_format = "tif";
}


const Profile*
WCS11Source::createProfile( const Profile* mapProfile, const std::string& configPath )
{
    //TODO: once we read GetCapabilities.. this will change..
    return osgEarth::Registry::instance()->getGlobalGeodeticProfile();
}


std::string
WCS11Source::getExtension() const
{
    return "tif";
}


osg::Image*
WCS11Source::createImage( const TileKey* key )
{
    osg::ref_ptr<HTTPRequest> request = createRequest( key );

    osg::notify(osg::INFO) << "[osgEarth::WCS1.1] Key=" << key->str() << " URL = " << request->getURL() << std::endl;

    double lon0,lat0,lon1,lat1;
    key->getGeoExtent().getBounds( lon0, lat0, lon1, lat1 );

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

    osgDB::ReaderWriter::ReadResult result = reader->readImage( input_stream, getOptions() );
    if ( !result.success() )
    {
        osg::notify(osg::NOTICE) << "[osgEarth::WCS1.1] WARNING: readImage() failed for Reader " << reader->getName() << std::endl;
        return NULL;
    }

    osg::Image* image = result.getImage();
    //osg::notify(osg::NOTICE) << "Returned grid is " << image->s() << "x" << image->t() << std::endl;
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
    key->getGeoExtent().getBounds( lon_min, lat_min, lon_max, lat_max );

    int lon_samples = _tile_size;
    int lat_samples = _tile_size;
    double lon_interval = (lon_max-lon_min)/(double)(lon_samples-1);
    double lat_interval = (lat_max-lat_min)/(double)(lat_samples-1);

    HTTPRequest* req = new HTTPRequest( _url );

    req->addParameter( "SERVICE",    "WCS" );
    req->addParameter( "VERSION",    "1.1.0" );
    req->addParameter( "REQUEST",    "GetCoverage" );
    req->addParameter( "IDENTIFIER", _identifier );
    req->addParameter( "FORMAT",     _cov_format );

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
    buf.str("");

    //bool use_legacy_geog_bbox_encoding = _url.find( "/MapServer/WCSServer" ) != std::string::npos;
    //if ( use_legacy_geog_bbox_encoding )
    //    buf << lon_min << "," << lat_min << "," << lon_max << "," << lat_max;
    //else
    //    buf << lat_min << "," << lon_min << "," << lat_max << "," << lon_max;
    //buf << ",urn:ogc:def:crs:EPSG::4326";

    double halfLon = lon_interval/2.0;
    double halfLat = lat_interval/2.0;

    //We need to shift the bounding box out by half a pixel in all directions so that the center of the edge pixels lie on
    //the edge of this TileKey's extents.  Doing this makes neighboring tiles have the same elevation values so there is no need
    //to run the tile edge normalization code.
    buf << lon_min - halfLon << "," << lat_min - halfLat << "," << lon_max + halfLon << "," << lat_max + halfLat << ",EPSG:4326";
    req->addParameter( "BOUNDINGBOX", buf.str() );

    double originX = lon_min;
    double originY = lat_max;

    buf.str("");
    buf << originX << "," << originY; 
    req->addParameter( "GridOrigin", buf.str() );
    
    buf.str("");
    buf << lon_interval << "," << lat_interval;   // note: top-down
    //buf << lon_interval << "," << lat_interval;
    req->addParameter( "GridOffsets", buf.str() );

    if (!_range_subset.empty())
    {
        req->addParameter( "RangeSubset", _range_subset );
    }

    return req;
}
