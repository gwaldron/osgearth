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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/ArcGISImageServer>
#include <osgEarth/Registry>
#include <osgEarth/JsonUtils>
#include <osgEarth/ImageToHeightFieldConverter>

using namespace osgEarth;

#undef LC
#define LC "[ArcGISImageServer] "

//........................................................................

ArcGISImageServer::ImageServiceLayer::ImageServiceLayer(int in_id,
                                               const std::string& in_name) :
id( in_id ),
name( in_name )
{
    //NOP
}

int
ArcGISImageServer::ImageServiceLayer::getId() const
{
    return id;
}

const std::string&
ArcGISImageServer::ImageServiceLayer::getName() const
{
    return name;
}

//........................................................................

ArcGISImageServer::ImageServiceField::ImageServiceField(const std::string& in_name, const std::string& in_type, const std::string& in_alias) :
	name(in_name),
	type(in_type),
	alias(in_alias)
{
    //NOP
}

const std::string&
ArcGISImageServer::ImageServiceField::getName() const
{
    return name;
}

const std::string&
ArcGISImageServer::ImageServiceField::getType() const
{
	return type;
}

const std::string&
ArcGISImageServer::ImageServiceField::getAlias() const
{
	return alias;
}

//........................................................................

ArcGISImageServer::TileInfo::TileInfo() :
is_valid( false ),
tile_size(0),
min_level(0),
max_level(0),
num_tiles_wide(0),
num_tiles_high(0)
{
    //NOP
}

ArcGISImageServer::TileInfo::TileInfo( int _tile_size, const std::string& _format, int _min_level, int _max_level, int _num_tiles_wide, int _num_tiles_high ) :
format( _format ),
tile_size( _tile_size ),
min_level( _min_level ),
max_level( _max_level ),
is_valid( true ),
num_tiles_wide(_num_tiles_wide),
num_tiles_high(_num_tiles_high)
{
    //NOP
}

ArcGISImageServer::TileInfo::TileInfo( const TileInfo& rhs ) :
format( rhs.format ),
tile_size( rhs.tile_size ),
min_level( rhs.min_level ),
max_level( rhs.max_level ),
is_valid( rhs.is_valid ),
num_tiles_wide( rhs.num_tiles_wide ),
num_tiles_high( rhs.num_tiles_high )
{
    //NOP
}

bool
ArcGISImageServer::TileInfo::isValid() const {
    return is_valid;
}

int
ArcGISImageServer::TileInfo::getTileSize() const {
    return tile_size;
}

const std::string&
ArcGISImageServer::TileInfo::getFormat() const {
    return format;
}

int
ArcGISImageServer::TileInfo::getMinLevel() const {
    return min_level;
}

int
ArcGISImageServer::TileInfo::getMaxLevel() const {
    return max_level;
}

int
ArcGISImageServer::TileInfo::getNumTilesWide() const {
    return num_tiles_wide;
}

int
ArcGISImageServer::TileInfo::getNumTilesHigh() const {
    return num_tiles_high;
}

//........................................................................

ArcGISImageServer::ImageService::ImageService() :
is_valid( false ),
tiled( false )
{
    //NOP
}

bool
ArcGISImageServer::ImageService::isValid() const {
    return is_valid;
}

bool
ArcGISImageServer::ImageService::isTiled() const {
    return tiled;
}

const Profile*
ArcGISImageServer::ImageService::getProfile() const {
    return profile.get();
}

const ArcGISImageServer::TileInfo&
ArcGISImageServer::ImageService::getTileInfo() const {
    return tile_info;
}

const std::string&
ArcGISImageServer::ImageService::getCopyright() const
{
    return _copyright;
}

bool
ArcGISImageServer::ImageService::init( const URI& _uri, const osgDB::ReaderWriter::Options* options )
{
    uri = _uri;
    std::string sep = uri.full().find( "?" ) == std::string::npos ? "?" : "&";
    std::string json_url = uri.full() + sep + std::string("f=pjson");  // request the data in JSON format

    ReadResult r = URI(json_url).readString( options );
    if ( r.failed() )
        return setError( "Unable to read metadata from ArcGIS service" );

    Json::Value doc;
    Json::Reader reader;
//    if ( !reader.parse( response.getPartStream(0), doc ) )
    if ( !reader.parse( r.getString(), doc ) )
	{
        return setError( "Unable to parse metadata; invalid JSON" );
	}

    // Read the profile. We are using "fullExtent"; perhaps an option to use "initialExtent" instead?
	double xmin = 0.0;
	double ymin = 0.0;
	double xmax = 0.0;
	double ymax = 0.0;
	int srs = 0;

    Json::Value fullExtentValue = doc["fullExtent"];
	Json::Value extentValue = doc["extent"];
	std::string srsValue;

	// added a case for "extent" which can be removed if we want to fall back on initialExtent if fullExtent fails
    if ( !fullExtentValue.empty() )
	{
		// if "fullExtent" exists .. use that
		xmin = doc["fullExtent"].get("xmin", 0).asDouble();
		ymin = doc["fullExtent"].get("ymin", 0).asDouble();
		xmax = doc["fullExtent"].get("xmax", 0).asDouble();
		ymax = doc["fullExtent"].get("ymax", 0).asDouble();
		srs = doc["fullExtent"].get("spatialReference", osgEarth::Json::Value::null).get("wkid", 0).asInt();

		srsValue = doc["fullExtent"].get("spatialReference", osgEarth::Json::Value::null).get("wkt", "null").asString();

		OE_DEBUG << LC << "fullExtent discovered: xmin: " << xmin << ", ymin: " << ymin << ", xmax: " << xmax << ", ymax: " << ymax << ", srs: " << srs << std::endl;
	}
	else if( !extentValue.empty() )
	{
		// else if "extent" exists .. use that
		xmin = doc["extent"].get("xmin", 0).asDouble();
		ymin = doc["extent"].get("ymin", 0).asDouble();
		xmax = doc["extent"].get("xmax", 0).asDouble();
		ymax = doc["extent"].get("ymax", 0).asDouble();
		srs = doc["extent"].get("spatialReference", osgEarth::Json::Value::null).get("wkid", 0).asInt();

		srsValue = doc["extent"].get("spatialReference", osgEarth::Json::Value::null).get("wkt", "null").asString();

		OE_DEBUG << LC << "extent discovered: xmin: " << xmin << ", ymin: " << ymin << ", xmax: " << xmax << ", ymax: " << ymax << ", srs: " << srs << std::endl;
	}
	else
	{
		// else "initialExtent" must exist ..
		xmin = doc["initialExtent"].get("xmin", 0).asDouble();
		ymin = doc["initialExtent"].get("ymin", 0).asDouble();
		xmax = doc["initialExtent"].get("xmax", 0).asDouble();
		ymax = doc["initialExtent"].get("ymax", 0).asDouble();
		srs = doc["initialExtent"].get("spatialReference", osgEarth::Json::Value::null).get("wkid", 0).asInt();

		srsValue = doc["initialExtent"].get("spatialReference", osgEarth::Json::Value::null).get("wkt", "null").asString();

		OE_DEBUG << LC << "initialExtent discovered: xmin: " << xmin << ", ymin: " << ymin << ", xmax: " << xmax << ", ymax: " << ymax << ", srs: " << srs << std::endl;
	}

    //Assumes the SRS is going to be an EPSG code
    std::stringstream ss;
    ss << "epsg:" << srs;

	// we can create a valid spatial reference from the WKT/proj4 string .. here just check if x/y/min/max values are set & correct
    if ( ! (xmax > xmin && ymax > ymin /*&& srs != 0*/ ) )
    {
        return setError( "Image service does not define a full extent" );
    }

    pixelSizeX = doc["pixelSizeX"].asDouble();
	pixelSizeY = doc["pixelSizeY"].asDouble();
	bandCount = doc["bandCount"].asInt();
    pixelType = doc["pixelType"].asString();

    // Check that the layers list is not empty
    Json::Value j_layers = doc["layers"];
    if (!j_layers.empty())
    {
        // not required to initialise a layer .. in any case this code does nothing
        for (unsigned int i = 0; i < j_layers.size(); i++)
        {
            Json::Value layer = j_layers[i];
            int id = i; // layer.get("id", -1).asInt();
            std::string name = layer["name"].asString();

            if (id >= 0 && !name.empty())
            {
                layers.push_back(ImageServiceLayer(id, name));
            }
        }
	}

	// Check that the fields list is not empty
	Json::Value j_fields = doc["fields"];
	if (!j_fields.empty())
	{
		// not required to initialise a layer .. in any case this code does nothing
		for (unsigned int i = 0; i < j_fields.size(); i++)
		{
			Json::Value field = j_fields[i];
			std::string name = field["name"].asString();
			std::string type = field["type"].asString();
			std::string alias = field["alias"].asString();

			if (!name.empty())
			{
                fields.push_back(ImageServiceField(name, type, alias));
			}
		}
	}

    tiled = false;
    std::string format = "png";
    int tile_rows = 256;
    int tile_cols = 256;
    int min_level = 25;
    int max_level = 0;
    int num_tiles_wide = 1;
    int num_tiles_high = 1;

    // Read the tiling schema
    Json::Value j_tileinfo = doc["tileInfo"];
    if ( !j_tileinfo.empty() )
    {
        tiled = true;

     //   return setError( "Image service does not define a tiling schema" );

        // TODO: what do we do if the width <> height?
        tile_rows = j_tileinfo.get( "rows", 0 ).asInt();
        tile_cols = j_tileinfo.get( "cols", 0 ).asInt();
        if ( tile_rows <= 0 && tile_cols <= 0 )
		{
            return setError( "Image service tile size not specified" );
		}

        format = j_tileinfo.get( "format", "" ).asString();
        if ( format.empty() )
		{
            return setError( "Image service tile schema does not specify an image format" );
		}

        Json::Value j_levels = j_tileinfo["lods"];
        if ( j_levels.empty() )
		{
            return setError( "Image service tile schema contains no LODs" );
		}

        min_level = INT_MAX;
        max_level = 0;
        for( unsigned int i=0; i<j_levels.size(); i++ )
        {
            int level = j_levels[i].get( "level", -1 ).asInt();
            if ( level >= 0 && level < min_level )
			{
                min_level = level;
			}
            if ( level >= 0 && level > max_level )
			{
                max_level = level;
			}
        }

        if (j_levels.size() > 0)
        {
            int l = j_levels[0u].get("level", -1).asInt();
            double res = j_levels[0u].get("resolution", 0.0).asDouble();
            num_tiles_wide = (int)osg::round((xmax - xmin) / (res * tile_cols));
            num_tiles_high = (int)osg::round((ymax - ymin) / (res * tile_rows));

            //In case the first level specified isn't level 0, compute the number of tiles at level 0
            for (int i = 0; i < l; i++)
            {
                num_tiles_wide /= 2;
                num_tiles_high /= 2;
            }

            //profile.setNumTilesWideAtLod0(num_tiles_wide);
            //profile.setNumTilesHighAtLod0(num_tiles_high);
        }
    }

	std::string ssStr;
	ssStr = ss.str();

	osg::ref_ptr< SpatialReference > spatialReference;

	 // if srs is in a non integer form .. find out what form it's in & create ..
	if(srs == 0)
	{
		OE_DEBUG << LC << "srsString: " << srsValue << std::endl;

		// create spatial reference from WKT string
		spatialReference = SpatialReference::create( srsValue );
	}
	else
	{
		// create spatial reference from epsg string (WKID)
		spatialReference = SpatialReference::create( ssStr );
	}

	// if spatial reference is valid .. create a profile
	if( spatialReference.valid() )
	{
		// create profile from spatial reference
		if ( spatialReference->isGeographic() )
		{
			// If we have a geographic SRS, just use the geodetic profile
			profile = Registry::instance()->getGlobalGeodeticProfile();
		}
		else if ( spatialReference->isMercator() )
		{
			// If we have a mercator SRS, just use the mercator profile
			profile = Registry::instance()->getGlobalMercatorProfile();
		}
		else
		{
			//It's not geodetic or mercator, so try to use the full extent
			profile = Profile::create(
				spatialReference.get(),
				xmin, ymin, xmax, ymax,
				num_tiles_wide,
				num_tiles_high);
		}
	}
	else
	{
		return setError( "Image service Spatial Reference INVALID" );
	}

	if( !profile.valid() )
	{
		return setError( "Image service could not create a valid profile" );
	}

    Json::Value j_copyright = doc["copyrightText"];
    if (!j_copyright.empty())
        _copyright = trim(j_copyright.asString());

    // now we're good.
    tile_info = ArcGISImageServer::TileInfo( tile_rows, format, min_level, max_level, num_tiles_wide, num_tiles_high);
    is_valid = true;
    return is_valid;
}

bool
ArcGISImageServer::ImageService::setError( const std::string& msg )
{
    error_msg = msg;
    return false;
}

const std::string&
ArcGISImageServer::ImageService::getError() const
{
    return error_msg;
}

//........................................................................

void
ArcGISImageServerOptions::readFrom(const Config& conf)
{
    conf.get("url", url());
    conf.get("token", token());
    conf.get("format", format());
    conf.get("layers", layers());
}

void
ArcGISImageServerOptions::writeTo(Config& conf) const
{
    conf.set("url", url());
    conf.set("token", token());
    conf.set("format", format());
    conf.set("layers", layers());
}


Config
ArcGISImageServerImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
ArcGISImageServerImageLayer::Options::fromConfig(const Config& conf)
{
    return readFrom(conf);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(arcgisimageserverimage, ArcGISImageServerImageLayer);

OE_LAYER_PROPERTY_IMPL(ArcGISImageServerImageLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(ArcGISImageServerImageLayer, std::string, Token, token);
OE_LAYER_PROPERTY_IMPL(ArcGISImageServerImageLayer, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(ArcGISImageServerImageLayer, std::string, Layers, layers);

void
ArcGISImageServerImageLayer::init()
{
    ImageLayer::init();
}

ArcGISImageServerImageLayer::~ArcGISImageServerImageLayer()
{
    //nop
}

Status
ArcGISImageServerImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // Default cache policy to NO_CACHE for public services
    if (options().cachePolicy().isSet() == false &&
        options().url()->full().find("arcgisonline.com") != std::string::npos)
    {
        options().cachePolicy() = CachePolicy::NO_CACHE;
    }

    if (options().url().isSet() == false)
    {
        return Status(Status::ConfigurationError, "Missing required endpoint URL");
    }

    // add the security token to the URL if necessary:
    URI url = options().url().value();

    OE_DEBUG << LC << "Initial URL: " << url.full() << std::endl;

    // append token to url in query string is set
    if (options().token().isSet())
    {
        std::string token = options().token().value();
        if (!token.empty())
        {
            std::string sep = url.full().find("?") == std::string::npos ? "?" : "&";
            url = url.append(sep + std::string("token=") + token);
        }
    }
    else
    {
        OE_DEBUG << LC << "Token not set" << std::endl;
    }

    // append layers to url in query string is set .. format is show:1,2,3
    if (options().layers().isSet())
    {
        std::string layers = options().layers().value();
        OE_DEBUG << LC << "_Layers: " << layers << std::endl;
        if (!layers.empty())
        {
            std::string sep = url.full().find("?") == std::string::npos ? "?" : "&";
            url = url.append(sep + std::string("layers=show:") + layers);
        }
    }
    else
    {
        OE_DEBUG << LC << "Layer options not set" << std::endl;
    }

    OE_DEBUG << LC << "_image_service URL: " << url.full() << std::endl;

    // read map service metadata from the server
    if (!_image_service.init(url, getReadOptions()))
    {
        OE_INFO << LC << "_image_service.init failed: " << _image_service.getError() << std::endl;

        return Status(
            Status::ResourceUnavailable, Stringify()
            << "ArcGIS image service initialization failed: "
            << _image_service.getError());
    }

    // image format to request:
    _format = options().format().getOrUse(_image_service.getTileInfo().getFormat());
    _format = osgEarth::toLower(_format);
    if (_format.length() > 3 && _format.substr(0, 3) == "png")
    {
        _format = "png";
    }
    if (_format == "mixed")
    {
        _format = "";
    }
    if (!_format.empty())
    {
        _dot_format = "." + _format;
    }

    _copyright = _image_service.getCopyright();
    if (options().attribution().isSet() == false)
    {
        setAttribution(_copyright);
    }

    // set the tile size from the map service
    setTileSize(_image_service.getTileInfo().getTileSize());

    // establish a profile if we don't already have one:
    if (!getProfile())
    {
        const Profile* profile = NULL;

        if (_profileConf.isSet())
        {
            profile = Profile::create(_profileConf.get());
        }
        else if (_image_service.getProfile())
        {
            profile = _image_service.getProfile();
        }
        else
        {
            // finally, fall back on lat/long
            profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        }
        setProfile(profile);
    }

    return Status::NoError;
}

GeoImage
ArcGISImageServerImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    std::stringstream buf;

    int level = key.getLevelOfDetail();

    unsigned int tile_x, tile_y;
    key.getTileXY(tile_x, tile_y);

    if (_image_service.isTiled())
    {
        /*buf << options().url()->full() << "/tile"
            << "/" << level
            << "/" << tile_y
            << "/" << tile_x << _dot_format;*/
        buf << options().url()->full() << "/tile"
            << "/" << level
            << "/" << tile_y
            << "/" << tile_x;
    }
    else
    {
        const GeoExtent& ex = key.getExtent();

        buf << std::setprecision(16)
            << options().url()->full() << "/exportImage"
            << "?bbox=" << ex.xMin() << "," << ex.yMin() << "," << ex.xMax() << "," << ex.yMax()
            << "&format=" << _format
            << "&size=256,256"
            << "&transparent=true"
            << "&f=image";
    }

    //Add the token if necessary
    if (options().token().isSet())
    {
        std::string token = options().token().value();
        if (!token.empty())
        {
            std::string str;
            str = buf.str();
            std::string sep = str.find("?") == std::string::npos ? "?" : "&";
            buf << sep << "token=" << token;
        }
    }

    //Add the layers if necessary
    if (!_layers.empty())
    {
        std::string str;
        str = buf.str();
        std::string sep = str.find("?") == std::string::npos ? "?" : "&";
        buf << sep << "layers=show:" << _layers;
	}

    std::string bufStr;
    bufStr = buf.str();

	URI uri(bufStr, options().url()->context());
    osg::Image* image = uri.getImage(getReadOptions(), progress);

    return GeoImage(image, key.getExtent());
}


//........................................................................

Config
ArcGISImageServerElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
ArcGISImageServerElevationLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}

REGISTER_OSGEARTH_LAYER(arcgisimageserverelevation, ArcGISImageServerElevationLayer);

OE_LAYER_PROPERTY_IMPL(ArcGISImageServerElevationLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(ArcGISImageServerElevationLayer, std::string, Token, token);
OE_LAYER_PROPERTY_IMPL(ArcGISImageServerElevationLayer, std::string, Format, format);
OE_LAYER_PROPERTY_IMPL(ArcGISImageServerElevationLayer, std::string, Layers, layers);


void
ArcGISImageServerElevationLayer::init()
{
    ElevationLayer::init();
}

Status
ArcGISImageServerElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    _imageLayer = new ArcGISImageServerImageLayer(options());

    // Initialize and open the image layer
    _imageLayer->setReadOptions(getReadOptions());
    Status status = _imageLayer->open();

    if (status.isError())
        return status;

    setProfile(_imageLayer->getProfile());
    dataExtents() = _imageLayer->getDataExtents();

    return Status::NoError;
}

Status
ArcGISImageServerElevationLayer::closeImplementation()
{
    if (_imageLayer.valid())
    {
        _imageLayer->close();
        _imageLayer = NULL;
    }
    return ElevationLayer::closeImplementation();
}

GeoHeightField
ArcGISImageServerElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    // Make an image, then convert it to a heightfield
    GeoImage image = _imageLayer->createImageImplementation(key, progress);
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        osg::HeightField* hf = conv.convert(image.getImage());
        return GeoHeightField(hf, key.getExtent());
    }
    else
    {
        return GeoHeightField(image.getStatus());
    }
}




