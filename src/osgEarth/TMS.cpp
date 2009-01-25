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


#include <osg/Notify>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgEarth/Common>
#include <osgEarth/HTTPClient>
#include <osgEarth/XmlUtils>
#include <osgEarth/TMS>
#include <osgEarth/Mercator>

#include <limits.h>
#include <iomanip>

using namespace osgEarth;

static std::string toString(double value, int precision = 25)
{
    std::stringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

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
_minLevel(0),
_maxLevel(0),
_originX(0),
_originY(0),
_minX(0.0),
_minY(0.0),
_maxX(0.0),
_maxY(0.0)
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
#define ELEM_BOUNDINGBOX "boundingbox"
#define ELEM_ORIGIN "origin"
#define ELEM_TILE_FORMAT "tileformat"
#define ELEM_TILESETS "tilesets"
#define ELEM_TILESET "tileset"

#define ATTR_VERSION "version"
#define ATTR_TILEMAPSERVICE "tilemapservice"

#define ATTR_MINX "minx"
#define ATTR_MINY "miny"
#define ATTR_MAXX "maxx"
#define ATTR_MAXY "maxy"
#define ATTR_X "x"
#define ATTR_Y "y"

#define ATTR_WIDTH "width"
#define ATTR_HEIGHT "height"
#define ATTR_MIME_TYPE "mime-type"
#define ATTR_EXTENSION "extension"

#define ATTR_PROFILE "profile"

#define ATTR_HREF "href"
#define ATTR_ORDER "order"
#define ATTR_UNITSPERPIXEL "units-per-pixel"

bool intersects(const double &minXa, const double &minYa, const double &maxXa, const double &maxYa,
                const double &minXb, const double &minYb, const double &maxXb, const double &maxYb)
{
    return  osg::maximum(minXa, minXb) <= osg::minimum(maxXa,maxXb) &&
            osg::maximum(minYa, minYb) <= osg::minimum(maxYa, maxYb);
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


std::string
TileMap::getURL(const osgEarth::TileKey *tileKey, bool invertY)
{
    if (!intersectsKey(tileKey))
    {
        //osg::notify(osg::NOTICE) << "No key intersection for tile key " << tileKey->str() << std::endl;
        return "";
    }

    unsigned int zoom = tileKey->getLevelOfDetail();
    int totalTiles = TileKey::getMapSizeTiles(zoom);

    if (tileKey->isGeodetic() )
    {
        //In global-geodetic TMS, level 0 is two tiles that cover the entire earth.
        //Level 0 in osgEarth is a single tile that covers the entire earth and extends down to -270,
        //so osgEarth level 1 is more like TMS level 0.
        zoom--;

        //Only half the vertical tiles are used in TMS global-geodetic
        totalTiles /= 2;
    }

    unsigned int x, y;
    tileKey->getTileXY(x, y);

    //Some TMS like services swap the Y coordinate so 0,0 is the upper left rather than the lower left.  The normal TMS
    //specification has 0,0 at the bottom left, so inverting Y will make 0,0 in the upper left.
    //http://code.google.com/apis/maps/documentation/overlays.html#Google_Maps_Coordinates
    if (!invertY)
    {
        y  = totalTiles - y - 1;
    }

    //osg::notify(osg::NOTICE) << "KEY: " << tileKey->str() << " level " << zoom << " ( " << x << ", " << y << ")" << std::endl;

    //Select the correct TileSet
    if ( _tileSets.size() > 0 )
    {
        for (TileSetList::iterator itr = _tileSets.begin(); itr != _tileSets.end(); ++itr)
        { 
            if (itr->getOrder() == zoom)
            {
                std::stringstream ss;
                std::string path = osgDB::getFilePath(_filename);
                ss << path << "/" << zoom << "/" << x << "/" << y << "." << _format.getExtension();
                //osg::notify(osg::NOTICE) << "Returning URL " << ss.str() << std::endl;
                return ss.str();
            }
        }
    }
    else // Just go with it. No way of knowing the max level.
    {
        std::stringstream ss;
        std::string path = osgDB::getFilePath(_filename);
        ss << path << "/" << zoom << "/" << x << "/" << y << "." << _format.getExtension();
        return ss.str();        
    }

    return "";
}

bool
TileMap::intersectsKey(const TileKey *tileKey)
{
    double keyMinX, keyMinY, keyMaxX, keyMaxY;

    //Check to see if the key overlaps the bounding box using lat/lon.  This is necessary to check even in 
    //Mercator situations in case the BoundingBox is described using lat/lon coordinates such as those produced by GDAL2Tiles
    //This should be considered a bug on the TMS production side, but we can work around it for now...
    tileKey->getGeoExtents(keyMinX, keyMinY, keyMaxX, keyMaxY);

    bool inter = intersects(_minX, _minY, _maxX, _maxY, keyMinX, keyMinY, keyMaxX, keyMaxY);

    if (!inter)
    {
        tileKey->getNativeExtents(keyMinX, keyMinY, keyMaxX, keyMaxY);
        inter = intersects(_minX, _minY, _maxX, _maxY, keyMinX, keyMinY, keyMaxX, keyMaxY);
    }

    return inter;
}

TileMap*
TileMap::create(const std::string& url,
                osgEarth::TileGridProfile::ProfileType type,
                const std::string& format,
                int tile_width,
                int tile_height)
{
    TileMap* tileMap = NULL;

    switch( type )
    {
    case osgEarth::TileGridProfile::GLOBAL_MERCATOR:
        tileMap = new TileMap();
        tileMap->_srs = "EPSG:41001";
        tileMap->_originX = -20037508.340000;
        tileMap->_originY = -20037508.340000;
        tileMap->_minX = -20037508.340000;
        tileMap->_minY = -20037508.340000;
        tileMap->_maxX = 20037508.340000;
        tileMap->_maxY = 20037508.340000;
        break;

    case osgEarth::TileGridProfile::GLOBAL_GEODETIC:
        tileMap = new TileMap();
        tileMap->_srs = "EPSG:4326";
        tileMap->_originX = -180.0;
        tileMap->_originY = -90.0;
        tileMap->_minX = -180.0;
        tileMap->_minY = -90.0;
        tileMap->_maxX = 180.0;
        tileMap->_maxY = 90.0;
        break;
    }

    if ( tileMap )
    {
        tileMap->_filename = url;
        tileMap->_format.setWidth( tile_width );
        tileMap->_format.setHeight( tile_height );
        tileMap->_format.setExtension( format );        
    }

    return tileMap;
}

TileMap* TileMap::create(const TileSource* tileSource)
{
    TileMap* tileMap = new TileMap();

    tileMap->setTitle( tileSource->getName() );
    tileMap->setProfile(tileSource->getProfile().getProfileType());

    switch (tileSource->getProfile().getProfileType())
    {
    case TileGridProfile::GLOBAL_MERCATOR:
        tileMap->_srs = "EPSG:41001";
        tileMap->_originX = -20037508.340000;
        tileMap->_originY = -20037508.340000;
        tileMap->_minX = -20037508.340000;
        tileMap->_minY = -20037508.340000;
        tileMap->_maxX = 20037508.340000;
        tileMap->_maxY = 20037508.340000;
        break;

    case TileGridProfile::GLOBAL_GEODETIC:
        tileMap->_srs = "EPSG:4326";
        tileMap->_originX = -180.0;
        tileMap->_originY = -90.0;
        tileMap->_minX = -180.0;
        tileMap->_minY = -90.0;
        tileMap->_maxX = 180.0;
        tileMap->_maxY = 90.0;
        break;
    case TileGridProfile::PROJECTED:
        tileMap->_srs = tileSource->getProfile().srs();
        tileMap->_originX = tileSource->getProfile().xMin();
        tileMap->_originY = tileSource->getProfile().yMin();
        tileMap->_minX = tileSource->getProfile().xMin();
        tileMap->_minY = tileSource->getProfile().yMin();
        tileMap->_maxX = tileSource->getProfile().xMax();
        tileMap->_maxY = tileSource->getProfile().yMax();
        break;
    }

    tileMap->_format.setWidth( tileSource->getPixelsPerTile() );
    tileMap->_format.setHeight( tileSource->getPixelsPerTile() );
    tileMap->_format.setExtension( tileSource->getExtension() );

    return tileMap;
}



//----------------------------------------------------------------------------


TileMap* 
TileMapReaderWriter::read( const std::string &location )
{
    TileMap *tileMap = NULL;
    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPClient client;
        osg::ref_ptr<HTTPResponse> response = client.get( location );
        if (response.valid() && response->isOK() && response->getNumParts() > 0 )
        {
            tileMap = read( response->getPartStream( 0 ) );
        }
    }
    else
    {
        if ((osgDB::fileExists(location)) && (osgDB::fileType(location) == osgDB::REGULAR_FILE))
        {
            std::ifstream in( location.c_str() );
            tileMap = read( in );
        }
    }
    if (tileMap)
    {
        tileMap->setFilename( location );
    }
    return tileMap;
}

TileMap*
TileMapReaderWriter::read(std::istream &in)
{
    osg::ref_ptr<TileMap> tileMap = new TileMap;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
    if (!doc.valid())
    {
        osg::notify(osg::NOTICE) << "Failed to load TileMap " << std::endl;
        return 0;
    }
   
    //Get the root TileMap element
    osg::ref_ptr<XmlElement> e_tile_map = doc->getSubElement( ELEM_TILEMAP );
    if (!e_tile_map.valid())
    {
        osg::notify(osg::NOTICE) << "Could not find root TileMap element " << std::endl;
        return 0;
    }

    tileMap->setVersion( e_tile_map->getAttr( ATTR_VERSION ) );
    tileMap->setTileMapService( e_tile_map->getAttr( ATTR_TILEMAPSERVICE ) );

    tileMap->setTitle( e_tile_map->getSubElementText(ELEM_TITLE) );
    tileMap->setAbstract( e_tile_map->getSubElementText(ELEM_ABSTRACT) );
    tileMap->setSRS( e_tile_map->getSubElementText(ELEM_SRS) );

    //Read the bounding box
    osg::ref_ptr<XmlElement> e_bounding_box = e_tile_map->getSubElement(ELEM_BOUNDINGBOX);
    if (e_bounding_box.valid())
    {
        double minX = as<double>(e_bounding_box->getAttr( ATTR_MINX ), 0.0);
        double minY = as<double>(e_bounding_box->getAttr( ATTR_MINY ), 0.0);
        double maxX = as<double>(e_bounding_box->getAttr( ATTR_MAXX ), 0.0);
        double maxY = as<double>(e_bounding_box->getAttr( ATTR_MAXY ), 0.0);
        tileMap->setExtents( minX, minY, maxX, maxY);
    }

    //Read the origin
    osg::ref_ptr<XmlElement> e_origin = e_tile_map->getSubElement(ELEM_ORIGIN);
    if (e_origin.valid())
    {
        tileMap->setOriginX( as<double>(e_origin->getAttr( ATTR_X ), 0.0) );
        tileMap->setOriginY( as<double>(e_origin->getAttr( ATTR_Y ), 0.0) );
    }

    //Read the tile format
    osg::ref_ptr<XmlElement> e_tile_format = e_tile_map->getSubElement(ELEM_TILE_FORMAT);
    if (e_tile_format.valid())
    {
        tileMap->getFormat().setExtension( e_tile_format->getAttr( ATTR_EXTENSION ) );
        tileMap->getFormat().setMimeType( e_tile_format->getAttr( ATTR_MIME_TYPE) );
        tileMap->getFormat().setWidth( as<unsigned int>(e_tile_format->getAttr( ATTR_WIDTH ), 0) );
        tileMap->getFormat().setHeight( as<unsigned int>(e_tile_format->getAttr( ATTR_HEIGHT ), 0) );
    }

    //Read the tilesets
    osg::ref_ptr<XmlElement> e_tile_sets = e_tile_map->getSubElement(ELEM_TILESETS);
    if (e_tile_sets.valid())
    {
        //Read the profile
        std::string profile = e_tile_sets->getAttr( ATTR_PROFILE );
        if (profile == STR_GLOBAL_GEODETIC) tileMap->setProfile( TileGridProfile::GLOBAL_GEODETIC );
        else if (profile == STR_GLOBAL_MERCATOR) tileMap->setProfile( TileGridProfile::GLOBAL_MERCATOR );
        else if (profile == STR_LOCAL) tileMap->setProfile( TileGridProfile::PROJECTED );
        else tileMap->setProfile( TileGridProfile::UNKNOWN );

        //Read each TileSet
        XmlNodeList tile_sets = e_tile_sets->getSubElements( ELEM_TILESET );
        for( XmlNodeList::const_iterator i = tile_sets.begin(); i != tile_sets.end(); i++ )
        {
            osg::ref_ptr<XmlElement> e_tile_set = static_cast<XmlElement*>( i->get() );
            TileSet tileset;
            tileset.setHref( e_tile_set->getAttr( ATTR_HREF ) );
            tileset.setOrder( as<unsigned int>(e_tile_set->getAttr( ATTR_ORDER ), -1) );
            tileset.setUnitsPerPixel( as<double>(e_tile_set->getAttr( ATTR_UNITSPERPIXEL ), 0.0 ) );
            tileMap->getTileSets().push_back(tileset);
        }
    }

    //Try to compute the profile based on the SRS if there was no PROFILE tag given
    if (tileMap->getProfile() == TileGridProfile::UNKNOWN && !tileMap->getSRS().empty())
    {
        tileMap->setProfile( TileGridProfile::getProfileTypeFromSRS(tileMap->getSRS()) );
    }

    tileMap->computeMinMaxLevel();

    return tileMap.release();
}

XmlDocument* tileMapToXmlDocument(const TileMap* tileMap)
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    
    //Create the root node
    osg::ref_ptr<XmlElement> e_tile_map = new XmlElement( ELEM_TILEMAP );
    doc->getChildren().push_back( e_tile_map.get() );

    e_tile_map->getAttrs()[ ATTR_VERSION ] = tileMap->getVersion();
    e_tile_map->getAttrs()[ ATTR_TILEMAPSERVICE ] = tileMap->getTileMapService();
  
    e_tile_map->addSubElement( ELEM_TITLE, tileMap->getTitle() );
    e_tile_map->addSubElement( ELEM_ABSTRACT, tileMap->getAbstract() );
    e_tile_map->addSubElement( ELEM_SRS, tileMap->getSRS() );

    osg::ref_ptr<XmlElement> e_bounding_box = new XmlElement( ELEM_BOUNDINGBOX );
    double minX, minY, maxX, maxY;
    tileMap->getExtents( minX, minY, maxX, maxY );
    e_bounding_box->getAttrs()[ATTR_MINX] = toString(minX);
    e_bounding_box->getAttrs()[ATTR_MINY] = toString(minY);
    e_bounding_box->getAttrs()[ATTR_MAXX] = toString(maxX);
    e_bounding_box->getAttrs()[ATTR_MAXY] = toString(maxY);
    e_tile_map->getChildren().push_back(e_bounding_box.get() );

    osg::ref_ptr<XmlElement> e_origin = new XmlElement( ELEM_ORIGIN );
    e_origin->getAttrs()[ATTR_X] = toString(tileMap->getOriginX());
    e_origin->getAttrs()[ATTR_Y] = toString(tileMap->getOriginY());
    e_tile_map->getChildren().push_back(e_origin.get());

    osg::ref_ptr<XmlElement> e_tile_format = new XmlElement( ELEM_TILE_FORMAT );
    e_tile_format->getAttrs()[ ATTR_EXTENSION ] = tileMap->getFormat().getExtension();
    e_tile_format->getAttrs()[ ATTR_MIME_TYPE ] = tileMap->getFormat().getMimeType();
    e_tile_format->getAttrs()[ ATTR_WIDTH ] = toString<unsigned int>(tileMap->getFormat().getWidth());
    e_tile_format->getAttrs()[ ATTR_HEIGHT ] = toString<unsigned int>(tileMap->getFormat().getHeight());
    e_tile_map->getChildren().push_back(e_tile_format.get());

    osg::ref_ptr<XmlElement> e_tile_sets = new XmlElement ( ELEM_TILESETS );
    std::string profileString = "none";
    if (tileMap->getProfile() == TileGridProfile::GLOBAL_GEODETIC)
    {
        profileString = STR_GLOBAL_GEODETIC;
    }
    else if (tileMap->getProfile() == TileGridProfile::GLOBAL_MERCATOR)
    {
        profileString = STR_GLOBAL_MERCATOR;
    }
    else if (tileMap->getProfile() == TileGridProfile::PROJECTED)
    {
        profileString = STR_LOCAL;
    }
    e_tile_sets->getAttrs()[ ATTR_PROFILE ] = profileString;


    //TODO:  Write Tilesets
    for (TileMap::TileSetList::const_iterator itr = tileMap->getTileSets().begin(); itr != tileMap->getTileSets().end(); ++itr)
    {
        osg::ref_ptr<XmlElement> e_tile_set = new XmlElement( ELEM_TILESET );
        e_tile_set->getAttrs()[ATTR_HREF] = itr->getHref();
        e_tile_set->getAttrs()[ATTR_ORDER] = toString<unsigned int>(itr->getOrder());
        e_tile_set->getAttrs()[ATTR_UNITSPERPIXEL] = toString(itr->getUnitsPerPixel());
        e_tile_sets->getChildren().push_back( e_tile_set.get() );
    }
    e_tile_map->getChildren().push_back(e_tile_sets.get());

    return doc.release();
}

void
TileMapReaderWriter::write(const TileMap* tileMap, const std::string &location)
{
    std::string path = osgDB::getFilePath(location);
    if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        osg::notify(osg::NOTICE) << "Couldn't create path " << std::endl;
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


