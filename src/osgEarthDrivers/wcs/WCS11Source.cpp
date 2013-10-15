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

#include "WCS11Source.h"
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osg/Notify>
#include <osgDB/Registry>
#include <iostream>
#include <stdlib.h>

#include <osgEarth/XmlUtils>

using namespace osgEarth;


WCS11Source::WCS11Source( const TileSourceOptions& options ) :
TileSource( options ),
_options  ( options )
{
    _covFormat = _options.format().value();
    
    if ( _covFormat.empty() )
        _covFormat = "image/GeoTIFF";

    _osgFormat = "tif";
}



osgEarth::TileSource::Status WCS11Source::initialize(const osgDB::Options* dbOptions)
{        
    //TODO: fetch GetCapabilities and set profile from there.
    setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
    _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );
    CachePolicy::NO_CACHE.apply( _dbOptions.get() );

    std::string capUrl;

    if ( _options.url().isSet() )
    {
        char sep = _options.url()->full().find_first_of('?') == std::string::npos? '?' : '&';

        capUrl = 
            _options.url()->full() +
            sep + 
            "SERVICE=WCS&VERSION=1.1.0&REQUEST=GetCapabilities";
    }        

    _capabilities = WCSCapabilitiesReader::read( capUrl, _dbOptions.get() );

    return STATUS_OK;
}


std::string
WCS11Source::getExtension() const
{
    return "tif";
}


osg::Image*
WCS11Source::createImage(const TileKey&        key,
                         ProgressCallback*     progress)
{
    HTTPRequest request = createRequest( key );

    OE_INFO << "[osgEarth::WCS1.1] Key=" << key.str() << " URL = " << request.getURL() << std::endl;

    double lon0,lat0,lon1,lat1;
    key.getExtent().getBounds( lon0, lat0, lon1, lat1 );

    // download the data. It's a multipart-mime stream, so we have to use HTTP directly.
    HTTPResponse response = HTTPClient::get( request, _dbOptions.get(), progress );
    if ( !response.isOK() )
    {
        OE_WARN << "[osgEarth::WCS1.1] WARNING: HTTP request failed" << std::endl;
        return NULL;
    }

    //TODO:  Make WCS driver use progress callback
    unsigned int part_num = response.getNumParts() > 1? 1 : 0;
    std::istream& input_stream = response.getPartStream( part_num );

    //TODO: un-hard-code TIFFs
    osgDB::ReaderWriter* reader = osgDB::Registry::instance()->getReaderWriterForExtension( "tiff" );

    if ( !reader )
    {
        OE_NOTICE << "[osgEarth::WCS1.1] WARNING: no reader for \"tiff\"" << std::endl;
        return NULL;
    }

    osgDB::ReaderWriter::ReadResult result = reader->readImage( input_stream ); //, getOptions() );
    if ( !result.success() )
    {
        OE_NOTICE << "[osgEarth::WCS1.1] WARNING: readImage() failed for Reader " << reader->getName() << std::endl;
        return NULL;
    }

    osg::Image* image = result.getImage();
    //OE_NOTICE << "Returned grid is " << image->s() << "x" << image->t() << std::endl;
    if ( image ) image->ref();
    return image;
}


osg::HeightField*
WCS11Source::createHeightField(const TileKey&        key,
                               ProgressCallback*     progress)
{
    osg::HeightField* field = NULL;

    osg::ref_ptr<osg::Image> image = createImage( key, progress );
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


HTTPRequest
WCS11Source::createRequest( const TileKey& key ) const
{
    std::stringstream buf;

    double lon_min, lat_min, lon_max, lat_max;
    key.getExtent().getBounds( lon_min, lat_min, lon_max, lat_max );

    int lon_samples = _options.tileSize().value();
    int lat_samples = _options.tileSize().value();
    double lon_interval = (lon_max-lon_min)/(double)(lon_samples-1);
    double lat_interval = (lat_max-lat_min)/(double)(lat_samples-1);

    HTTPRequest req( _options.url()->full() );

    req.addParameter( "SERVICE",    "WCS" );
    req.addParameter( "VERSION",    "1.1.0" );
    req.addParameter( "REQUEST",    "GetCoverage" );
    req.addParameter( "IDENTIFIER", _options.identifier().value() );
    req.addParameter( "FORMAT",     _covFormat );

    req.addParameter( "GridBaseCRS", "urn:ogc:def:crs:EPSG::4326" );
    req.addParameter( "GridCS",      "urn:ogc:def:crs:EPSG::4326" );
    req.addParameter( "GridType",    "urn:ogc:def:method:WCS:1.1:2dGridIn2dCrs" );

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
    std::string crs = "EPSG:4326";
    if( _capabilities.valid() && _capabilities->getSupportedCRS().compare("") )
        crs = _capabilities->getSupportedCRS();
    buf << lon_min - halfLon << "," << lat_min - halfLat << "," << lon_max + halfLon << "," << lat_max + halfLat << ","<< crs;
	std::string bufStr;
	bufStr = buf.str();
    req.addParameter( "BOUNDINGBOX", bufStr );

    double originX = lon_min;
    double originY = lat_max;

    buf.str("");
    buf << originX << "," << originY; 
	bufStr = buf.str();
    req.addParameter( "GridOrigin", bufStr );
    
    buf.str("");
    buf << lon_interval << "," << lat_interval;   // note: top-down
    //buf << lon_interval << "," << lat_interval;
	bufStr = buf.str();
    req.addParameter( "GridOffsets", bufStr );

    if ( !_options.rangeSubset()->empty() )
        req.addParameter( "RangeSubset", _options.rangeSubset().value() );

    return req;
}




WCSCapabilities::WCSCapabilities()
:   _version(""),
    _name(""),
    _title(""),
    _abstract(""),
    _supportedCRS("")
{
}


#define ATTR_VERSION "version"
#define ELEM_CAPABILITY "capability"
#define ELEM_SERVICE "owcs:ServiceIdentification"
#define ELEM_REQUEST "request"
#define ELEM_ABSTRACT "abstract"
#define ELEM_TILED "tiled"
#define ELEM_MAXLEVEL "maxlevel"
#define ELEM_FIRSTLEVEL "firstlevel"
#define ELEM_FORMAT "format"
#define ELEM_NAME "name"
#define ELEM_TITLE "title"
#define ELEM_SRS "srs"
#define ELEM_FEATURETYPELIST "featuretypelist"
#define ELEM_FEATURETYPE "featuretype"
#define ELEM_LATLONGBOUNDINGBOX "latlongboundingbox"
#define ELEM_CONTENTS "contents"
#define ATTR_MINX              "minx"
#define ATTR_MINY              "miny"
#define ATTR_MAXX              "maxx"
#define ATTR_MAXY              "maxy"
#define ATTR_SUPPORTEDCRS      "SupportedCRS"

WCSCapabilities* 
WCSCapabilitiesReader::read( const URI& location, const osgDB::Options* dbOptions )
{
    // read the data into a string buffer and parse it from there
    std::string buffer = location.readString(dbOptions).getString();
    if ( !buffer.empty() )
    {
        std::stringstream buf(buffer);
        return read(buf);
    }
    else return 0L;
}

WCSCapabilities*
WCSCapabilitiesReader::read(std::istream &in)
{
    osg::ref_ptr<WCSCapabilities> capabilities = new WCSCapabilities;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
    if (!doc.valid() || doc->getChildren().empty())
    {
        OE_NOTICE << "Failed to load Capabilities " << std::endl;
        return 0;
    }

    //Get the Capabilities version
    osg::ref_ptr<XmlElement> e_root = static_cast<XmlElement*>(doc->getChildren()[0].get());
    capabilities->setVersion( e_root->getAttr(ATTR_VERSION ) );

    osg::ref_ptr<XmlElement> e_service = e_root->getSubElement( ELEM_SERVICE );
    if (!e_service.valid())
    {
        OE_NOTICE << "Could not find Service element" << std::endl;
    }
    else
    {
        //Read the parameters from the Service block
        capabilities->setName( e_service->getSubElementText(ELEM_NAME ) );
        capabilities->setAbstract( e_service->getSubElementText( ELEM_ABSTRACT ) );
        capabilities->setTitle( e_service->getSubElementText( ELEM_TITLE ) );    
    }

    osg::ref_ptr<XmlElement> e_contents = e_root->getSubElement( ELEM_CONTENTS );
    if (!e_contents.valid())
    {
        OE_NOTICE << "Could not find Contents element" << std::endl;
    }
    else
    {
        capabilities->setSupportedCRS(e_contents->getSubElementText( ATTR_SUPPORTEDCRS ) );
    }

    return capabilities.release();
}