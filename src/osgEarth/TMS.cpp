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
#include "TMS"
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/LandCover>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::TMS;

#undef LC
#define LC "[TMS] "

//........................................................................

namespace osgEarth { namespace TMS
{
    std::string toString(double value, int precision = 7)
    {
        std::stringstream out;
        out << std::fixed << std::setprecision(precision) << value;
	    std::string outStr;
	    outStr = out.str();
        return outStr;
    }
} }

//........................................................................

TileFormat::TileFormat():
_width(0),
_height(0)
{
}

TileSet::TileSet():
_unitsPerPixel(0.0),
_order(0)
{
}

TileMap::TileMap():
_originX(0),
_originY(0),
_minX(0.0),
_minY(0.0),
_maxX(0.0),
_maxY(0.0),
_minLevel(0),
_maxLevel(0),
_numTilesHigh(-1),
_numTilesWide(-1),
_timestamp(0),
_version("1.0"),
_tileMapService("http://tms.osgeo.org/1.0.0"),
_profile_type(Profile::TYPE_MERCATOR)
{
}

void TileMap::setOrigin(double x, double y)
{
    _originX = x;
    _originY = y;
}

void TileMap::getExtents( double &minX, double &minY, double &maxX, double &maxY) const
{
    minX = _minX;
    minY = _minY;
    maxX = _maxX;
    maxY = _maxY;
}

void TileMap::setExtents( double minX, double minY, double maxX, double maxY)
{
    _minX = minX;
    _minY = minY;
    _maxX = maxX;
    _maxY = maxY;
}

#define ELEM_TILEMAP "tilemap"
#define ELEM_TITLE "title"
#define ELEM_ABSTRACT "abstract"
#define ELEM_SRS "srs"
#define ELEM_VERTICAL_SRS "vsrs"
#define ELEM_VERTICAL_DATUM "vdatum"
#define ELEM_BOUNDINGBOX "boundingbox"
#define ELEM_ORIGIN "origin"
#define ELEM_TILE_FORMAT "tileformat"
#define ELEM_TILESETS "tilesets"
#define ELEM_TILESET "tileset"
#define ELEM_DATA_EXTENTS "dataextents"
#define ELEM_DATA_EXTENT "dataextent"

#define ATTR_VERSION "version"
#define ATTR_TILEMAPSERVICE "tilemapservice"

#define ATTR_MINX "minx"
#define ATTR_MINY "miny"
#define ATTR_MAXX "maxx"
#define ATTR_MAXY "maxy"
#define ATTR_X "x"
#define ATTR_Y "y"
#define ATTR_MIN_LEVEL "minlevel"
#define ATTR_MAX_LEVEL "maxlevel"

#define ATTR_WIDTH "width"
#define ATTR_HEIGHT "height"
#define ATTR_MIME_TYPE "mime-type"
#define ATTR_EXTENSION "extension"

#define ATTR_PROFILE "profile"

#define ATTR_HREF "href"
#define ATTR_ORDER "order"
#define ATTR_UNITSPERPIXEL "units-per-pixel"

#define ATTR_DESCRIPTION "description"

namespace
{
    bool intersects(const double &minXa, const double &minYa, const double &maxXa, const double &maxYa,
                    const double &minXb, const double &minYb, const double &maxXb, const double &maxYb)
    {
        return  osg::maximum(minXa, minXb) <= osg::minimum(maxXa,maxXb) &&
                osg::maximum(minYa, minYb) <= osg::minimum(maxYa, maxYb);
    }

    std::string getHorizSRSString(const osgEarth::SpatialReference* srs)
    {
        if (srs->isSphericalMercator())
        {
            return "EPSG:900913";
        }
        else if (srs->isGeographic())
        {
            return "EPSG:4326";
        }
        else
        {
            return srs->getHorizInitString(); //srs();
        }
    }
}


void TileMap::computeMinMaxLevel()
{
    _minLevel = INT_MAX;
    _maxLevel = 0;
    for (TileSetList::iterator itr = _tileSets.begin(); itr != _tileSets.end(); ++itr)
    {
        if (itr->getOrder() < _minLevel) _minLevel = itr->getOrder();
        if (itr->getOrder() > _maxLevel) _maxLevel = itr->getOrder();
    }
}

void TileMap::computeNumTiles()
{
    _numTilesWide = -1;
    _numTilesHigh = -1;

    if (_tileSets.size() > 0)
    {
        unsigned int level = _tileSets[0].getOrder();
        double res = _tileSets[0].getUnitsPerPixel();

        _numTilesWide = (int)((_maxX - _minX) / (res * _format.getWidth()));
        _numTilesHigh = (int)((_maxY - _minY) / (res * _format.getWidth()));

        //In case the first level specified isn't level 0, compute the number of tiles at level 0
        for (unsigned int i = 0; i < level; i++)
        {
            _numTilesWide /= 2;
            _numTilesHigh /= 2;
        }

        OE_DEBUG << LC << "TMS has " << _numTilesWide << ", " << _numTilesHigh << " tiles at level 0 " <<  std::endl;
    }
}

const Profile*
TileMap::createProfile() const
{
    osg::ref_ptr<const Profile> profile = 0L;
    osg::ref_ptr< SpatialReference > spatialReference =  osgEarth::SpatialReference::create(_srs, _vsrs);

    if (getProfileType() == Profile::TYPE_GEODETIC)
    {
        profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    }
    else if (getProfileType() == Profile::TYPE_MERCATOR)
    {
        profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
    }
    else if (spatialReference->isSphericalMercator())
    {
        //HACK:  Some TMS sources, most notably TileCache, use a global mercator extent that is very slightly different than
        //       the automatically computed mercator bounds which can cause rendering issues due to the some texture coordinates
        //       crossing the dateline.  If the incoming bounds are nearly the same as our definion of global mercator, just use our definition.
        double eps = 0.01;
        osg::ref_ptr< const Profile > merc = osgEarth::Registry::instance()->getSphericalMercatorProfile();
        if (_numTilesWide == 1 && _numTilesHigh == 1 &&
            osg::equivalent(merc->getExtent().xMin(), _minX, eps) &&
            osg::equivalent(merc->getExtent().yMin(), _minY, eps) &&
            osg::equivalent(merc->getExtent().xMax(), _maxX, eps) &&
            osg::equivalent(merc->getExtent().yMax(), _maxY, eps))
        {
            profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
        }
    }

    else if (
        spatialReference->isGeographic() &&
        osg::equivalent(_minX, -180.) &&
        osg::equivalent(_maxX,  180.) &&
        osg::equivalent(_minY,  -90.) &&
        osg::equivalent(_maxY,   90.) )
    {
        profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    }
    else if ( _profile_type == Profile::TYPE_MERCATOR )
    {
        profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
    }

    if ( !profile )
    {
        // everything else is a "LOCAL" profile.
        profile = Profile::create(
            _srs,
            _minX, _minY, _maxX, _maxY,
            _vsrs,
            osg::maximum(_numTilesWide, (unsigned int)1),
            osg::maximum(_numTilesHigh, (unsigned int)1) );
    }
    else if ( !_vsrs.empty() )
    {
        // vdatum override?
        ProfileOptions options(profile->toProfileOptions());
        options.vsrsString() = _vsrs;
        profile = Profile::create(options);
    }


    return profile.release();
}


std::string
TileMap::getURL(const osgEarth::TileKey& tilekey, bool invertY)
{
    if (!intersectsKey(tilekey))
    {
        //OE_NOTICE << LC << "No key intersection for tile key " << tilekey.str() << std::endl;
        return "";
    }

    unsigned int zoom = tilekey.getLevelOfDetail();

    unsigned int x, y;
    tilekey.getTileXY(x, y);

    //Some TMS like services swap the Y coordinate so 0,0 is the upper left rather than the lower left.  The normal TMS
    //specification has 0,0 at the bottom left, so inverting Y will make 0,0 in the upper left.
    //http://code.google.com/apis/maps/documentation/overlays.html#Google_Maps_Coordinates
    if (!invertY)
    {
        unsigned int numRows, numCols;
        tilekey.getProfile()->getNumTiles(tilekey.getLevelOfDetail(), numCols, numRows);
        y  = numRows - y - 1;
    }

    //OE_NOTICE << LC << "KEY: " << tilekey.str() << " level " << zoom << " ( " << x << ", " << y << ")" << std::endl;

    //Select the correct TileSet
    if ( _tileSets.size() > 0 )
    {
        for (TileSetList::iterator itr = _tileSets.begin(); itr != _tileSets.end(); ++itr)
        {
            if (itr->getOrder() == zoom)
            {
                std::stringstream ss;
                std::string basePath = osgDB::getFilePath(_filename);
                if (!basePath.empty())
                {
                    ss << basePath << "/";
                }
                ss << zoom << "/" << x << "/" << y << "." << _format.getExtension();
                std::string ssStr;
				ssStr = ss.str();
				return ssStr;
            }
        }
    }
    else // Just go with it. No way of knowing the max level.
    {
        std::stringstream ss;
        std::string basePath = osgDB::getFilePath(_filename);
        if (!basePath.empty())
        {
            ss << basePath << "/";
        }
        ss << zoom << "/" << x << "/" << y << "." << _format.getExtension();
        std::string ssStr;
        ssStr = ss.str();
        return ssStr;
    }

    return "";
}

bool
TileMap::intersectsKey(const TileKey& tilekey)
{
    osg::Vec3d keyMin, keyMax;

    //double keyMinX, keyMinY, keyMaxX, keyMaxY;

    //Check to see if the key overlaps the bounding box using lat/lon.  This is necessary to check even in
    //Mercator situations in case the BoundingBox is described using lat/lon coordinates such as those produced by GDAL2Tiles
    //This should be considered a bug on the TMS production side, but we can work around it for now...
    tilekey.getExtent().getBounds(keyMin.x(), keyMin.y(), keyMax.x(), keyMax.y());
    //tilekey.getExtent().getBounds(keyMinX, keyMinY, keyMaxX, keyMaxY);

    bool inter = intersects(_minX, _minY, _maxX, _maxY, keyMin.x(), keyMin.y(), keyMax.x(), keyMax.y() ); //keyMinX, keyMinY, keyMaxX, keyMaxY);

    if (!inter && tilekey.getProfile()->getSRS()->isSphericalMercator())
    {
        tilekey.getProfile()->getSRS()->transform(keyMin, tilekey.getProfile()->getSRS()->getGeographicSRS(), keyMin );
        tilekey.getProfile()->getSRS()->transform(keyMax, tilekey.getProfile()->getSRS()->getGeographicSRS(), keyMax );
        inter = intersects(_minX, _minY, _maxX, _maxY, keyMin.x(), keyMin.y(), keyMax.x(), keyMax.y() );
        //tilekey.getProfile()->getSRS()->transform2D(keyMinX, keyMinY, tilekey.getProfile()->getSRS()->getGeographicSRS(), keyMinX, keyMinY);
        //tilekey.getProfile()->getSRS()->transform2D(keyMaxX, keyMaxY, tilekey.getProfile()->getSRS()->getGeographicSRS(), keyMaxX, keyMaxY);
        //inter = intersects(_minX, _minY, _maxX, _maxY, keyMinX, keyMinY, keyMaxX, keyMaxY);
    }

    return inter;
}

void
TileMap::generateTileSets(unsigned int numLevels)
{
    osg::ref_ptr<const Profile> profile = createProfile();

    _tileSets.clear();

    double width = (_maxX - _minX);
//    double height = (_maxY - _minY);

    for (unsigned int i = 0; i < numLevels; ++i)
    {
        unsigned int numCols, numRows;
        profile->getNumTiles(i, numCols, numRows);
        double res = (width / (double)numCols) / (double)_format.getWidth();

        TileSet ts;
        ts.setUnitsPerPixel(res);
        ts.setOrder(i);
        _tileSets.push_back(ts);
    }
}


TileMap*
TileMap::create(const std::string& url,
                const Profile*     profile,
                const DataExtentList& dataExtents,
                const std::string& format,
                int                tile_width,
                int                tile_height)
{
    const GeoExtent& ex = profile->getExtent();

    TileMap* tileMap = new TileMap();
    tileMap->setProfileType(profile->getProfileType());
    tileMap->setExtents(ex.xMin(), ex.yMin(), ex.xMax(), ex.yMax());
    tileMap->setOrigin(ex.xMin(), ex.yMin());
    tileMap->_filename = url;
    tileMap->_srs = getHorizSRSString(profile->getSRS());
    tileMap->_vsrs = profile->getSRS()->getVertInitString();
    tileMap->_format.setWidth( tile_width );
    tileMap->_format.setHeight( tile_height );
    profile->getNumTiles( 0, tileMap->_numTilesWide, tileMap->_numTilesHigh );

    // format can be a mime-type or an extension:
    std::string::size_type p = format.find('/');
    if ( p == std::string::npos )
    {
        tileMap->_format.setExtension(format);
        tileMap->_format.setMimeType( Registry::instance()->getMimeTypeForExtension(format) );
    }
    else
    {
        tileMap->_format.setMimeType(format);
        tileMap->_format.setExtension( Registry::instance()->getExtensionForMimeType(format) );
    }

    //Add the data extents
    tileMap->getDataExtents().insert(tileMap->getDataExtents().end(), dataExtents.begin(), dataExtents.end());

    // If we have some data extents specified then make a nicer bounds than the
    if (!tileMap->getDataExtents().empty())
    {
        // Get the union of all the extents
        GeoExtent e(tileMap->getDataExtents()[0]);
        for (unsigned int i = 1; i < tileMap->getDataExtents().size(); i++)
        {
            e.expandToInclude(tileMap->getDataExtents()[i]);
        }

        // Convert the bounds to the output profile
        GeoExtent bounds = e.transform(profile->getSRS());
        tileMap->setExtents(bounds.xMin(), bounds.yMin(), bounds.xMax(), bounds.yMax());
    }

    tileMap->generateTileSets();
    tileMap->computeMinMaxLevel();

    return tileMap;
}


//----------------------------------------------------------------------------


TileMap*
TileMapReaderWriter::read( const URI& uri, const osgDB::Options* options )
{
    TileMap* tileMap = NULL;

    ReadResult r = uri.readString(options);
    if ( r.failed() )
    {
        OE_DEBUG << LC << "Failed to read TMS tile map file from " << uri.full()
            << " ... " << r.errorDetail() << std::endl;
        return 0L;
    }

    // Read tile map into a Config:
    Config conf;
    std::stringstream buf( r.getString() );
    conf.fromXML( buf );

    // parse that into a tile map:
    tileMap = TileMapReaderWriter::read( conf );

    if (tileMap)
    {
        tileMap->setFilename( uri.full() );

        // record the timestamp (if there is one) in the tilemap. It's not a persistent field
        // but will help with things like per-session caching.
        tileMap->setTimeStamp( r.lastModifiedTime() );
    }

    return tileMap;
}

TileMap*
TileMapReaderWriter::read( const Config& conf )
{
    const Config* tileMapConf = conf.find( ELEM_TILEMAP );
    if ( !tileMapConf )
    {
        OE_WARN << LC << "Could not find root TileMap element " << std::endl;
        return 0L;
    }

    TileMap* tileMap = new TileMap();

    tileMap->setVersion       ( tileMapConf->value(ATTR_VERSION) );
    tileMap->setTileMapService( tileMapConf->value(ATTR_TILEMAPSERVICE) );
    tileMap->setTitle         ( tileMapConf->value(ELEM_TITLE) );
    tileMap->setAbstract      ( tileMapConf->value(ELEM_ABSTRACT) );
    tileMap->setSRS           ( tileMapConf->value(ELEM_SRS) );

    if (tileMapConf->hasValue(ELEM_VERTICAL_SRS))
        tileMap->setVerticalSRS( tileMapConf->value(ELEM_VERTICAL_SRS) );
    if (tileMapConf->hasValue(ELEM_VERTICAL_DATUM))
        tileMap->setVerticalSRS( tileMapConf->value(ELEM_VERTICAL_DATUM) );

    const Config* bboxConf = tileMapConf->find( ELEM_BOUNDINGBOX );
    if ( bboxConf )
    {
        double minX = bboxConf->value<double>( ATTR_MINX, 0.0 );
        double minY = bboxConf->value<double>( ATTR_MINY, 0.0 );
        double maxX = bboxConf->value<double>( ATTR_MAXX, 0.0 );
        double maxY = bboxConf->value<double>( ATTR_MAXY, 0.0 );
        tileMap->setExtents( minX, minY, maxX, maxY);
    }

    //Read the origin
    const Config* originConf = tileMapConf->find(ELEM_ORIGIN);
    if ( originConf )
    {
        tileMap->setOriginX( originConf->value<double>( ATTR_X, 0.0) );
        tileMap->setOriginY( originConf->value<double>( ATTR_Y, 0.0) );
    }

    //Read the tile format
    const Config* formatConf = tileMapConf->find( ELEM_TILE_FORMAT );
    if ( formatConf )
    {
        OE_DEBUG << LC << "Read TileFormat " << formatConf->value(ATTR_EXTENSION) << std::endl;
        tileMap->getFormat().setExtension( formatConf->value(ATTR_EXTENSION) );
        tileMap->getFormat().setMimeType ( formatConf->value(ATTR_MIME_TYPE) );
        tileMap->getFormat().setWidth    ( formatConf->value<unsigned>(ATTR_WIDTH,  256) );
        tileMap->getFormat().setHeight   ( formatConf->value<unsigned>(ATTR_HEIGHT, 256) );
    }
    else
    {
        OE_WARN << LC << "No TileFormat in TileMap!" << std::endl;
    }

    //Read the tilesets
    const Config* tileSetsConf = tileMapConf->find(ELEM_TILESETS);
    if ( tileSetsConf )
    {
        //Read the profile
        std::string profile = tileSetsConf->value(ATTR_PROFILE);
        if (profile == "global-geodetic") tileMap->setProfileType( Profile::TYPE_GEODETIC );
        else if (profile == "global-mercator") tileMap->setProfileType( Profile::TYPE_MERCATOR );
        else if (profile == "local") tileMap->setProfileType( Profile::TYPE_LOCAL );
        else tileMap->setProfileType( Profile::TYPE_UNKNOWN );

        //Read each TileSet
        const ConfigSet& setConfs = tileSetsConf->children(ELEM_TILESET);
        for( ConfigSet::const_iterator i = setConfs.begin(); i != setConfs.end(); ++i )
        {
            const Config& conf = *i;
            TileSet tileset;
            tileset.setHref( conf.value(ATTR_HREF) );
            tileset.setOrder( conf.value<unsigned>(ATTR_ORDER, ~0) );
            tileset.setUnitsPerPixel( conf.value<double>(ATTR_UNITSPERPIXEL, 0.0) );
            tileMap->getTileSets().push_back(tileset);
        }
    }

    //Try to compute the profile based on the SRS if there was no PROFILE tag given
    if (tileMap->getProfileType() == Profile::TYPE_UNKNOWN && !tileMap->getSRS().empty())
    {
        tileMap->setProfileType( Profile::getProfileTypeFromSRS(tileMap->getSRS()) );
    }

    tileMap->computeMinMaxLevel();
    tileMap->computeNumTiles();

    //Read the data areas
    const Config* extentsConf = tileMapConf->find(ELEM_DATA_EXTENTS);
    if ( extentsConf )
    {
        osg::ref_ptr< const osgEarth::Profile > profile = tileMap->createProfile();
        OE_DEBUG << LC << "Found DataExtents " << std::endl;
        const ConfigSet& children = extentsConf->children(ELEM_DATA_EXTENT);
        for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
        {
            const Config& conf = *i;
            double minX = conf.value<double>(ATTR_MINX, 0.0);
            double minY = conf.value<double>(ATTR_MINY, 0.0);
            double maxX = conf.value<double>(ATTR_MAXX, 0.0);
            double maxY = conf.value<double>(ATTR_MAXY, 0.0);

            unsigned int maxLevel = conf.value<unsigned>(ATTR_MAX_LEVEL, 0);

            std::string description = conf.value<std::string>(ATTR_DESCRIPTION, std::string());

            //OE_DEBUG << LC << "Read area " << minX << ", " << minY << ", " << maxX << ", " << maxY << ", minlevel=" << minLevel << " maxlevel=" << maxLevel << std::endl;

            if ( maxLevel > 0 )
            {
                if(description.empty())
                    tileMap->getDataExtents().push_back( DataExtent(GeoExtent(profile->getSRS(), minX, minY, maxX, maxY), 0, maxLevel));
                else
                    tileMap->getDataExtents().push_back( DataExtent(GeoExtent(profile->getSRS(), minX, minY, maxX, maxY), 0, maxLevel, description));
            }
            else
            {
                if(description.empty())
                    tileMap->getDataExtents().push_back( DataExtent(GeoExtent(profile->getSRS(), minX, minY, maxX, maxY), 0) );
                else
                    tileMap->getDataExtents().push_back( DataExtent(GeoExtent(profile->getSRS(), minX, minY, maxX, maxY), 0, description) );
            }
        }
    }


    return tileMap;
}

#define DOUBLE_PRECISION 25

static XmlDocument*
tileMapToXmlDocument(const TileMap* tileMap)
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    doc->setName( ELEM_TILEMAP );
    doc->getAttrs()[ ATTR_VERSION ] = tileMap->getVersion();
    doc->getAttrs()[ ATTR_TILEMAPSERVICE ] = tileMap->getTileMapService();

    doc->addSubElement( ELEM_TITLE, tileMap->getTitle() );
    doc->addSubElement( ELEM_ABSTRACT, tileMap->getAbstract() );
    doc->addSubElement( ELEM_SRS, tileMap->getSRS() );
    doc->addSubElement( ELEM_VERTICAL_SRS, tileMap->getVerticalSRS() );

    osg::ref_ptr<XmlElement> e_bounding_box = new XmlElement( ELEM_BOUNDINGBOX );
    double minX, minY, maxX, maxY;
    tileMap->getExtents( minX, minY, maxX, maxY );
    e_bounding_box->getAttrs()[ATTR_MINX] = toString(minX, DOUBLE_PRECISION);
    e_bounding_box->getAttrs()[ATTR_MINY] = toString(minY, DOUBLE_PRECISION);
    e_bounding_box->getAttrs()[ATTR_MAXX] = toString(maxX, DOUBLE_PRECISION);
    e_bounding_box->getAttrs()[ATTR_MAXY] = toString(maxY, DOUBLE_PRECISION);
    doc->getChildren().push_back(e_bounding_box.get() );

    osg::ref_ptr<XmlElement> e_origin = new XmlElement( ELEM_ORIGIN );
    e_origin->getAttrs()[ATTR_X] = toString(tileMap->getOriginX(), DOUBLE_PRECISION);
    e_origin->getAttrs()[ATTR_Y] = toString(tileMap->getOriginY(), DOUBLE_PRECISION);
    doc->getChildren().push_back(e_origin.get());

    osg::ref_ptr<XmlElement> e_tile_format = new XmlElement( ELEM_TILE_FORMAT );
    e_tile_format->getAttrs()[ ATTR_EXTENSION ] = tileMap->getFormat().getExtension();
    e_tile_format->getAttrs()[ ATTR_MIME_TYPE ] = tileMap->getFormat().getMimeType();
    e_tile_format->getAttrs()[ ATTR_WIDTH ] = toString<unsigned int>(tileMap->getFormat().getWidth());
    e_tile_format->getAttrs()[ ATTR_HEIGHT ] = toString<unsigned int>(tileMap->getFormat().getHeight());
    doc->getChildren().push_back(e_tile_format.get());

    osg::ref_ptr< const osgEarth::Profile > profile = tileMap->createProfile();

    osg::ref_ptr<XmlElement> e_tile_sets = new XmlElement ( ELEM_TILESETS );
    std::string profileString = "none";
    if (profile->isEquivalentTo(osgEarth::Registry::instance()->getGlobalGeodeticProfile()))
    {
        profileString = "global-geodetic";
    }
    else if (profile->isEquivalentTo(osgEarth::Registry::instance()->getSphericalMercatorProfile()))
    {
        profileString = "global-mercator";
    }
    else
    {
        profileString = "local";
    }
    e_tile_sets->getAttrs()[ ATTR_PROFILE ] = profileString;


    for (TileMap::TileSetList::const_iterator itr = tileMap->getTileSets().begin(); itr != tileMap->getTileSets().end(); ++itr)
    {
        osg::ref_ptr<XmlElement> e_tile_set = new XmlElement( ELEM_TILESET );
        e_tile_set->getAttrs()[ATTR_HREF] = itr->getHref();
        e_tile_set->getAttrs()[ATTR_ORDER] = toString<unsigned int>(itr->getOrder());
        e_tile_set->getAttrs()[ATTR_UNITSPERPIXEL] = toString(itr->getUnitsPerPixel(), DOUBLE_PRECISION);
        e_tile_sets->getChildren().push_back( e_tile_set.get() );
    }
    doc->getChildren().push_back(e_tile_sets.get());

    //Write out the data areas
    if (tileMap->getDataExtents().size() > 0)
    {
        osg::ref_ptr<XmlElement> e_data_extents = new XmlElement( ELEM_DATA_EXTENTS );
        for (DataExtentList::const_iterator itr = tileMap->getDataExtents().begin(); itr != tileMap->getDataExtents().end(); ++itr)
        {
            osg::ref_ptr<XmlElement> e_data_extent = new XmlElement( ELEM_DATA_EXTENT );
            e_data_extent->getAttrs()[ATTR_MINX] = toString(itr->xMin(), DOUBLE_PRECISION);
            e_data_extent->getAttrs()[ATTR_MINY] = toString(itr->yMin(), DOUBLE_PRECISION);
            e_data_extent->getAttrs()[ATTR_MAXX] = toString(itr->xMax(), DOUBLE_PRECISION);
            e_data_extent->getAttrs()[ATTR_MAXY] = toString(itr->yMax(), DOUBLE_PRECISION);
            if ( itr->minLevel().isSet() )
                e_data_extent->getAttrs()[ATTR_MIN_LEVEL] = toString<unsigned int>(*itr->minLevel());
            if ( itr->maxLevel().isSet() )
                e_data_extent->getAttrs()[ATTR_MAX_LEVEL] = toString<unsigned int>(*itr->maxLevel());
            if ( itr->description().isSet() )
                e_data_extent->getAttrs()[ATTR_DESCRIPTION] = *itr->description();
            e_data_extents->getChildren().push_back( e_data_extent );
        }
        doc->getChildren().push_back( e_data_extents.get() );
    }
    return doc.release();
}

void
TileMapReaderWriter::write(const TileMap* tileMap, const std::string &location)
{
    std::string path = osgDB::getFilePath(location);
    if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        OE_WARN << LC << "Couldn't create path " << std::endl;
    }
    std::ofstream out(location.c_str());
    write(tileMap, out);
}

void
TileMapReaderWriter::write(const TileMap* tileMap, std::ostream &output)
{
    osg::ref_ptr<XmlDocument> doc = tileMapToXmlDocument(tileMap);
    doc->store(output);
}


//----------------------------------------------------------------------------

TileMapEntry::TileMapEntry( const std::string& _title, const std::string& _href, const std::string& _srs, const std::string& _profile ):
title( _title ),
href( _href ),
srs( _srs ),
profile( _profile )
{
}

//----------------------------------------------------------------------------

TileMapServiceReader::TileMapServiceReader()
{
}

TileMapServiceReader::TileMapServiceReader(const TileMapServiceReader& rhs)
{
}

bool
TileMapServiceReader::read( const std::string &location, const osgDB::ReaderWriter::Options* options, TileMapEntryList& tileMaps )
{
    ReadResult r = URI(location).readString();
    if ( r.failed() )
    {
        OE_WARN << LC << "Failed to read TileMapServices from " << location << std::endl;
        return 0L;
    }

    // Read tile map into a Config:
    Config conf;
    std::stringstream buf( r.getString() );
    conf.fromXML( buf );

    // parse that into a tile map:
    return read( conf, tileMaps );
}

bool
TileMapServiceReader::read( const Config& conf, TileMapEntryList& tileMaps)
{
    const Config* TileMapServiceConf = conf.find("tilemapservice");

    if (!TileMapServiceConf)
    {
        OE_NOTICE << "Couldn't find root TileMapService element" << std::endl;
        return false;
    }

    const Config* TileMapsConf = TileMapServiceConf->find("tilemaps");
    if (TileMapsConf)
    {
        const ConfigSet& TileMaps = TileMapsConf->children("tilemap");
        if (TileMaps.size() == 0)
        {
            return false;
        }

        for (ConfigSet::const_iterator itr = TileMaps.begin(); itr != TileMaps.end(); ++itr)
        {
            std::string href = itr->value("href");
            std::string title = itr->value("title");
            std::string profile = itr->value("profile");
            std::string srs = itr->value("srs");

            tileMaps.push_back( TileMapEntry( title, href, srs, profile ) );
        }

        return true;
    }
    return false;
}

void
TMS::Driver::close()
{
    _tileMap = NULL;
    _writer = NULL;
    _forceRGBWrites = false;
}

Status
TMS::Driver::open(const URI& uri,
                  osg::ref_ptr<const Profile>& profile,
                  const std::string& format,
                  bool isCoverage,
                  DataExtentList& dataExtents,
                  const osgDB::Options* readOptions)
{
    _isCoverage = isCoverage;

    // URI is mandatory.
    if (uri.empty())
    {
        return Status::Error( Status::ConfigurationError, "TMS driver requires a valid \"url\" property" );
    }

    // A repo is writable only if it's local.
    if ( uri.isRemote() )
    {
        OE_DEBUG << LC << "Repo is remote; opening in read-only mode" << std::endl;
    }

    // Is this a new repo? (You can only create a new repo at a local non-archive URI.)
    bool isNewRepo = false;

    if (!uri.isRemote() &&
        !osgEarth::isPathToArchivedFile(uri.full()) &&
        !osgDB::fileExists(uri.full()) )
    {
        isNewRepo = true;

        // new repo REQUIRES a profile:
        if ( !profile.valid() )
        {
            return Status::Error(Status::ConfigurationError, "Fail: profile required to create new TMS repo");
        }
    }

    // Take the override profile if one is given
    if ( profile.valid() )
    {
        OE_INFO << LC
            << "Using express profile \"" << profile->toString()
            << "\" for URI \"" << uri.base() << "\""
            << std::endl;

        DataExtentList extents;

        _tileMap = TMS::TileMap::create(
            uri.full(),
            profile.get(),
            extents,
            format,
            256,
            256);

        // If this is a new repo, write the tilemap file to disk now.
        if ( isNewRepo )
        {
            if ( format.empty() )
            {
                return Status::Error(Status::ConfigurationError, "Missing required \"format\" property: e.g. png, jpg");
            }

            TMS::TileMapReaderWriter::write( _tileMap.get(), uri.full() );
            OE_INFO << LC << "Created new TMS repo at " << uri.full() << std::endl;
        }
    }

    else
    {
        // Attempt to read the tile map parameters from a TMS TileMap XML tile on the server:
        _tileMap = TMS::TileMapReaderWriter::read( uri, readOptions );

        if (!_tileMap.valid())
        {
            return Status::Error( Status::ResourceUnavailable, Stringify() << "Failed to read configuration from " << uri.full() );
        }

        OE_DEBUG << LC
            << "TMS tile map datestamp = "
            << DateTime(_tileMap->getTimeStamp()).asRFC1123()
            << std::endl;

        const Profile* profileFromTileMap = _tileMap->createProfile();
        if ( profileFromTileMap )
        {
            profile = profileFromTileMap;
        }
    }

    // Make sure we've established a profile by this point:
    if ( !profile.valid() )
    {
        return Status::Error( Stringify() << "Failed to establish a profile for " << uri.full() );
    }

    // resolve the writer
    if ( !uri.isRemote() && !resolveWriter(format) )
    {
        OE_WARN << LC << "Cannot create writer; writing disabled" << std::endl;
    }

    // TileMap and profile are valid at this point. Build the tile sets.
    // Automatically set the min and max level of the TileMap
    if ( _tileMap->getTileSets().size() > 0 )
    {
        OE_DEBUG << LC << "TileMap min/max " << _tileMap->getMinLevel() << ", " << _tileMap->getMaxLevel() << std::endl;
        if (_tileMap->getDataExtents().size() > 0)
        {
            for (DataExtentList::iterator itr = _tileMap->getDataExtents().begin(); itr != _tileMap->getDataExtents().end(); ++itr)
            {
                dataExtents.push_back(*itr);
            }
        }
    }
    return STATUS_OK;
}

osgEarth::ReadResult
TMS::Driver::read(const URI& uri,
                  const TileKey& key,
                  bool invertY,
                  ProgressCallback* progress,
                  const osgDB::Options* readOptions) const
{
    if (_tileMap.valid() && key.getLevelOfDetail() <= _tileMap->getMaxLevel())
    {
        std::string image_url = _tileMap->getURL(key, invertY);

        osg::ref_ptr<osg::Image> image;

        if (!image_url.empty())
        {
            URI uri(image_url, uri.context());
            osgEarth::ReadResult rr = uri.readImage(readOptions, progress);
            if (rr.failed())
                return rr;

            image = rr.getImage();
        }

        if (!image.valid())
        {
            if (image_url.empty() || !_tileMap->intersectsKey(key))
            {
                //We couldn't read the image from the URL or the cache, so check to see if the given key is less than the max level
                //of the tilemap and create a transparent image.
                if (key.getLevelOfDetail() <= _tileMap->getMaxLevel())
                {
                    OE_DEBUG << LC << "Returning empty image " << std::endl;
                    if (_isCoverage)
                        return LandCover::createEmptyImage();
                    else
                        return ImageUtils::createEmptyImage();
                }
            }
        }

        if (image.valid())
        {
            if (_isCoverage && !LandCover::isLandCover(image.get()))
            {
                osg::Image* dest = LandCover::createImage(image->s(), image->t());
                ImageUtils::PixelReader read(image.get());
                ImageUtils::PixelWriter write(dest);
                osg::Vec4 value;
                for(int t=0; t<read.t(); ++t)
                {
                    for(int s=0; s<read.s(); ++s)
                    {
                        read(value, s, t);
                        if (value.a() == 0.0)
                            value.r() = NO_DATA_VALUE;
                        write(value, s, t);
                    }
                }
                image = dest;
            }
        }

        return image.release();
    }
    else
    {
        return ReadResult::RESULT_NOT_FOUND;
    }
}

bool
TMS::Driver::write(const URI& uri,
                   const TileKey& key,
                   const osg::Image* image,
                   bool invertY,
                   ProgressCallback* progress,
                   const osgDB::Options* writeOptions) const
{
    if ( !_writer.valid() )
    {
        OE_WARN << LC << "Repo is read-only; store failed" << std::endl;
        return false;
    }

    if (_tileMap.valid() && image)
    {
        // compute the URL from the tile map:
        std::string image_url = _tileMap->getURL(key, invertY);

        // assert the folder exists:
        if (osgEarth::makeDirectoryForFile(image_url))
        {
            osgDB::ReaderWriter::WriteResult result;

            if (_forceRGBWrites && ImageUtils::hasAlphaChannel(image))
            {
                osg::ref_ptr<osg::Image> rgbImage = ImageUtils::convertToRGB8(image);
                result = _writer->writeImage(*(rgbImage.get()), image_url, writeOptions);
            }
            else
            {
                result = _writer->writeImage(*image, image_url, writeOptions);
            }

            if (result.error())
            {
                OE_WARN << LC << "store failed; url=[" << image_url << "] message=[" << result.message() << "]" << std::endl;
                return false;
            }
        }
        else
        {
            OE_WARN << LC << "Failed to make directory for " << image_url << std::endl;
            return false;
        }

        return true;
    }

    return false;
}

bool
TMS::Driver::resolveWriter(const std::string& format)
{
    _writer = osgDB::Registry::instance()->getReaderWriterForMimeType(
        _tileMap->getFormat().getMimeType());

    if (!_writer.valid())
    {
        _writer = osgDB::Registry::instance()->getReaderWriterForExtension(
            _tileMap->getFormat().getExtension());

        if (!_writer.valid())
        {
            _writer = osgDB::Registry::instance()->getReaderWriterForExtension(format);
        }
    }

    // The OSG JPEG writer does not accept RGBA images, so force conversion.
    _forceRGBWrites =
        _writer.valid() &&
        (_writer->acceptsExtension("jpeg") || _writer->acceptsExtension("jpg"));

    if (_forceRGBWrites)
    {
        OE_INFO << LC << "Note: images will be stored as RGB" << std::endl;
    }

    return _writer.valid();
}

//........................................................................

Config
TMS::Options::getMetadata()
{
    return Config::readJSON(OE_MULTILINE(
      { "name" : "TMS (Tile Map Service)",
        "properties" : [
          { "name": "url", "description" : "Location of the TMS repository", "type" : "string", "default" : "" },
          { "name": "tms_type", "description" : "Set to 'google' to invert the Y index", "type" : "string", "default" : "" },
          { "name": "format", "description" : "Image format to assume (e.g. jpeg, png)", "type" : "string", "default" : "" }
        ]
      }
    ));
}

void
TMS::Options::readFrom(const Config& conf)
{
    conf.get("url", _url);
    conf.get("format", _format);
    conf.get("tms_type", _tmsType);
}

void
TMS::Options::writeTo(Config& conf) const
{
    conf.set("url", _url);
    conf.set("tms_type", _tmsType);
    conf.set("format", _format);
}

//........................................................................

Config
TMSImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
TMSImageLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}

//........................................................................

REGISTER_OSGEARTH_LAYER(tmsimage, TMSImageLayer);

OE_LAYER_PROPERTY_IMPL(TMSImageLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(TMSImageLayer, std::string, TMSType, tmsType);
OE_LAYER_PROPERTY_IMPL(TMSImageLayer, std::string, Format, format);

void
TMSImageLayer::init()
{
    ImageLayer::init();
}

Status
TMSImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    osg::ref_ptr<const Profile> profile = getProfile();

    Status status = _driver.open(
        options().url().get(),
        profile,
        options().format().get(),
        options().coverage().get(),
        dataExtents(),
        getReadOptions());

    if (status.isError())
        return status;

    if (profile.get() != getProfile())
    {
        setProfile(profile.get());
    }

    return Status::NoError;
}

Status
TMSImageLayer::closeImplementation()
{
    _driver.close();
    return ImageLayer::closeImplementation();
}

GeoImage
TMSImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    ReadResult r = _driver.read(
        options().url().get(),
        key,
        options().tmsType().get() == "google",
        progress,
        getReadOptions());

    if (r.succeeded())
        return GeoImage(r.releaseImage(), key.getExtent());
    else
        return GeoImage(Status(r.errorDetail()));
}

Status
TMSImageLayer::writeImageImplementation(const TileKey& key, const osg::Image* image, ProgressCallback* progress) const
{
    if (!isWritingRequested())
        return Status::ServiceUnavailable;

    bool ok = _driver.write(
        options().url().get(),
        key,
        image,
        options().tmsType().get() == "google",
        progress,
        getReadOptions());

    if (!ok)
    {
        return Status::ServiceUnavailable;
    }

    return STATUS_OK;
}

//........................................................................

Config
TMSElevationLayer::Options::getConfig() const
{
    Config conf = ElevationLayer::Options::getConfig();
    writeTo(conf);
    return conf;
}

void
TMSElevationLayer::Options::fromConfig(const Config& conf)
{
    readFrom(conf);
}


//........................................................................

REGISTER_OSGEARTH_LAYER(tmselevation, TMSElevationLayer);

OE_LAYER_PROPERTY_IMPL(TMSElevationLayer, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(TMSElevationLayer, std::string, TMSType, tmsType);
OE_LAYER_PROPERTY_IMPL(TMSElevationLayer, std::string, Format, format);

void
TMSElevationLayer::init()
{
    ElevationLayer::init();
}

Status
TMSElevationLayer::openImplementation()
{
    Status parent = ElevationLayer::openImplementation();
    if (parent.isError())
        return parent;

    // Create an image layer under the hood. TMS fetch is the same for image and
    // elevation; we just convert the resulting image to a heightfield
    _imageLayer = new TMSImageLayer(options());

    // Initialize and open the image layer
    _imageLayer->setReadOptions(getReadOptions());
    Status status;
    if (_writingRequested)
    {
        status = _imageLayer->openForWriting();
    }
    else
    {
        status = _imageLayer->open();
    }

    if (status.isError())
        return status;

    setProfile(_imageLayer->getProfile());
    dataExtents() = _imageLayer->getDataExtents();

    return Status::NoError;
}

Status
TMSElevationLayer::closeImplementation()
{
    if (_imageLayer.valid())
    {
        _imageLayer->close();
        _imageLayer = NULL;
    }
    return ElevationLayer::closeImplementation();
}

GeoHeightField
TMSElevationLayer::createHeightFieldImplementation(const TileKey& key, ProgressCallback* progress) const
{
    // Make an image, then convert it to a heightfield
    GeoImage image = _imageLayer->createImageImplementation(key, progress);
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        osg::HeightField* hf = conv.convert( image.getImage() );
        return GeoHeightField(hf, key.getExtent());
    }
    else
    {
        return GeoHeightField(image.getStatus());
    }
}

Status
TMSElevationLayer::writeHeightFieldImplementation(
    const TileKey& key,
    const osg::HeightField* hf,
    ProgressCallback* progress) const
{
    ImageToHeightFieldConverter conv;
    osg::ref_ptr<osg::Image> image = conv.convert(hf);
    return _imageLayer->writeImageImplementation(key, image.get(), progress);
}
