#include "WCS11Source.h"
#include <osgEarth/HTTPClient>
#include <osg/Notify>
#include <osgDB/Registry>
#include <iostream>
#include <stdlib.h>

using namespace osgEarth;

WCS11Source::WCS11Source()
{
    //prefix     = "http://192.168.0.101/cgi-bin/mapserv?map=/var/www/maps/srtm.map";

    std::string host = "192.168.0.101";
    if ( ::getenv( "OSGEARTH_HOST" ) )
        host = std::string( ::getenv( "OSGEARTH_HOST" ) );

    std::stringstream buf;
    buf << "http://" << host << "/cgi-bin/mapserv";
    prefix     = buf.str();

    map_file   = "/var/www/maps/srtm.map";
    coverage   = "srtm";
    cov_format = "GEOTIFFINT16";
    osg_format = "tif";
}


osg::HeightField*
WCS11Source::createHeightField( const TileKey* key )
{
    osg::ref_ptr<HTTPRequest> request = createRequest( key );
    //std::string uri = createURI( key );

    double lon0,lat0,lon1,lat1;
    key->getGeoExtents(lon0,lat0,lon1,lat1);
    //osg::notify(osg::NOTICE) << "TILE: " << key.str() 
    //    << " (" <<lon0<<","<<lat0<<" => "<<lon1<<","<<lat1<<")"<< std::endl;
    //osg::notify(osg::NOTICE) << "URL: " << request->getURL() << std::endl;

    // download the data
    HTTPClient client;
    osg::ref_ptr<HTTPResponse> response = client.get( request.get() );
    if ( !response.valid() )
    {
        osg::notify(osg::NOTICE) << "[WCS11] WARNING: HTTP request failed" << std::endl;
        return NULL;
    }

    unsigned int part_num = response->getNumParts() > 1? 1 : 0;
    std::istream& input_stream = response->getPartStream( part_num );

    // tif reader doesn't support height fields, so read as image and convert
    osg::HeightField* field = NULL;
    
    osgDB::ReaderWriter* reader = osgDB::Registry::instance()->getReaderWriterForExtension( "tiff" );

    if ( !reader )
    {
        osg::notify(osg::NOTICE) << "[WCS11] WARNING: no reader for \"tiff\"" << std::endl;
        return NULL;
    }

    osgDB::ReaderWriter::ReadResult result = reader->readImage( input_stream );
    if ( !result.success() )
    {
        osg::notify(osg::NOTICE) << "[WCS11] WARNING: readImage() failed for Reader " << reader->getName() << std::endl;
        return NULL;
    }

    osg::ref_ptr<osg::Image> image = result.getImage();

//    osg::ref_ptr<osg::Image> image = osgDB::readImageFile( uri );
    if ( image.valid() )
    {
        field = new osg::HeightField();
        field->allocate( image->s(), image->t() );

        float vert_exag = 1.0f;

        for( unsigned int row=0; row < image->t(); row++ ) {
            //osg::notify(osg::NOTICE) << "ROW " << row << ":\t";
            for( unsigned int col=0; col < image->s(); col++ ) {
                unsigned char* ptr = image->data( col, row );
                if ( image->getPixelSizeInBits() == 16 ) {
                    short val = (short)*(short*)ptr;
                    field->setHeight( col, row, vert_exag * (float)val );
              //      osg::notify(osg::NOTICE) << val << "\t";
                }
                else if ( image->getPixelSizeInBits() == 32 ) {
                    float val = (float)*(float*)ptr;
                    field->setHeight( col, row, vert_exag * val );
                }
            }
            //osg::notify(osg::NOTICE) << std::endl;
        }
        //osg::notify(osg::NOTICE) << std::endl;
    }

    return field;
}

HTTPRequest*
WCS11Source::createRequest( const TileKey* key ) const
{
    std::stringstream buf;

    double lon_min, lat_min, lon_max, lat_max;
    key->getGeoExtents( lon_min, lat_min, lon_max, lat_max );

    int lon_samples = 8;
    int lat_samples = 8;
    double lon_interval = (lon_max-lon_min)/(double)(lon_samples-1);
    double lat_interval = (lat_max-lat_min)/(double)(lat_samples-1);

    HTTPRequest* req = new HTTPRequest( prefix );

    req->addParameter( "map", map_file );
    req->addParameter( "SERVICE", "WCS" );
    req->addParameter( "VERSION", "1.1.0" );
    req->addParameter( "REQUEST", "GetCoverage" );
    req->addParameter( "IDENTIFIER", coverage );
    req->addParameter( "FORMAT", cov_format );
    req->addParameter( "WIDTH", lon_samples );
    req->addParameter( "HEIGHT", lat_samples );

    buf.str("");
    buf << lat_min << "," << lon_min << "," << lat_max << "," << lon_max << ",urn:ogc:def:crs:EPSG::4326";
    req->addParameter( "BOUNDINGBOX", buf.str() );

    buf.str("");
    buf << lon_min << "," << lat_min;
    //req->addParameter( "GRIDORIGIN", buf.str() );

    buf.str("");
    buf << lon_interval << "," << lat_interval;
    //req->addParameter( "GRIDOFFSETS", buf.str() );

    req->addParameter( "GRIDCS", "urn:ogc:def:crs:EPSG::4326" );
    req->addParameter( "GRIDTYPE", "urn:ogc:def:method:WCS:1.1:2dGridIn2dCrs" );

    return req;
}