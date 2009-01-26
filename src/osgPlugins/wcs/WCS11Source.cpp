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
profile( TileGridProfile::GLOBAL_GEODETIC )
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
}

const TileGridProfile&
WCS11Source::getProfile() const
{
    return profile;
}

osg::Image*
WCS11Source::createImage( const TileKey* key)
{
    //NYI
    osg::notify( osg::WARN ) << "[osgEarth] [WCS11] images are not yet supported by the WCS driver." << std::endl;
    return 0;
}


osg::HeightField*
WCS11Source::createHeightField( const TileKey* key )
{
    osg::ref_ptr<HTTPRequest> request = createRequest( key );

    double lon0,lat0,lon1,lat1;
    key->getGeoExtents( lon0, lat0, lon1, lat1 );

    //osg::notify(osg::NOTICE) << "TILE: " << key.str() 
    //    << " (" <<lon0<<","<<lat0<<" => "<<lon1<<","<<lat1<<")"<< std::endl;
    //osg::notify(osg::NOTICE) << "URL: " << request->getURL() << std::endl;

    // download the data
    HTTPClient client;
    osg::ref_ptr<HTTPResponse> response = client.get( request.get() );
    if ( !response.valid() )
    {
        osg::notify(osg::WARN) << "[osgEarth] [WCS11]: WARNING: HTTP request failed" << std::endl;
        return NULL;
    }

    unsigned int part_num = response->getNumParts() > 1? 1 : 0;
    std::istream& input_stream = response->getPartStream( part_num );

    // tif reader doesn't support height fields, so read as image and convert
    osg::HeightField* field = NULL;
    
    osgDB::ReaderWriter* reader = osgDB::Registry::instance()->getReaderWriterForExtension( "tiff" );

    if ( !reader )
    {
        osg::notify(osg::NOTICE) << "[osgEarth] [WCS11] WARNING: no reader for \"tiff\"" << std::endl;
        return NULL;
    }

    osgDB::ReaderWriter::ReadResult result = reader->readImage( input_stream );
    if ( !result.success() )
    {
        osg::notify(osg::NOTICE) << "[osgEarth] [WCS11] WARNING: readImage() failed for Reader " << reader->getName() << std::endl;
        return NULL;
    }

    osg::ref_ptr<osg::Image> image = result.getImage();
    if ( image.valid() )
    {
        field = ImageToHeightFieldConverter::convert( image.get() );

        //field = new osg::HeightField();
        //field->allocate( image->s(), image->t() );
        //for( unsigned int row=0; row < image->t(); row++ ) {
        //    for( unsigned int col=0; col < image->s(); col++ ) {
        //        unsigned char* ptr = image->data( col, row );
        //        if ( image->getPixelSizeInBits() == 16 ) {
        //            short val = (short)*(short*)ptr;
        //            field->setHeight( col, row, (float)val );
        //        }
        //        else if ( image->getPixelSizeInBits() == 32 ) {
        //            float val = (float)*(float*)ptr;
        //            field->setHeight( col, row, val );
        //        }
        //    }
        //}

    }

    return field;
}

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

    //req->addParameter( "map", map_file );
    req->addParameter( "SERVICE",    "WCS" );
    req->addParameter( "VERSION",    "1.1.0" );
    req->addParameter( "REQUEST",    "GetCoverage" );
    req->addParameter( "IDENTIFIER", identifier );
    req->addParameter( "FORMAT",     cov_format );
    req->addParameter( "WIDTH",      lon_samples );
    req->addParameter( "HEIGHT",     lat_samples );

    buf.str("");
    buf << lat_min << "," << lon_min << "," << lat_max << "," << lon_max << ",urn:ogc:def:crs:EPSG::4326";
    req->addParameter( "BOUNDINGBOX", buf.str() );

    //buf.str("");
    //buf << lon_min << "," << lat_min;
    //req->addParameter( "GRIDORIGIN", buf.str() );

    //buf.str("");
    //buf << lon_interval << "," << lat_interval;
    //req->addParameter( "GRIDOFFSETS", buf.str() );

    //TODO: un-hard-code this
    req->addParameter( "GRIDCS", "urn:ogc:def:crs:EPSG::4326" );
    req->addParameter( "GRIDTYPE", "urn:ogc:def:method:WCS:1.1:2dGridIn2dCrs" );

    return req;
}