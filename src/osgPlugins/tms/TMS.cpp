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

#include "TMS"
#include <osg/Notify>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgEarth/Common>
#include <osgEarth/HTTPClient>
#include <osgEarth/XmlUtils>

using namespace osgEarth;

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
        if (itr->_order < _minLevel) _minLevel = itr->_order;
        if (itr->_order > _maxLevel) _maxLevel = itr->_order;
    }
}


std::string
TileMap::getURL(const osgEarth::TileKey *tileKey)
{
    if (!intersectsKey(tileKey)) return "";

    unsigned int zoom = tileKey->getLevelOfDetail();

    const PlateCarreTileKey* pc = dynamic_cast<const PlateCarreTileKey*>(tileKey);
    if (pc)
    {
        zoom--;
    }

    unsigned int x, y;
    tileKey->getTileXY(x, y);

    int totalTiles = sqrt(pow(4.0, (double)(zoom)));

    y  = totalTiles - y - 1;

    //osg::notify(osg::NOTICE) << "KEY: " << tileKey->str() << " level " << zoom << " ( " << x << ", " << y << ")" << std::endl;

    //Select the correct TileSet
    for (TileSetList::iterator itr = _tileSets.begin(); itr != _tileSets.end(); ++itr)
    { 
        if (itr->_order == zoom)
        {
            std::stringstream ss;
            std::string path = osgDB::getFilePath(_filename);
            ss << path << "/" << zoom << "/" << x << "/" << y << "." << _format._extension;
            //osg::notify(osg::NOTICE) << "Returning URL " << ss.str() << std::endl;
            return ss.str();
        }
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
        const MercatorTileKey* mk = dynamic_cast<const MercatorTileKey*>(tileKey);
        if (mk)
        {
            mk->getMeterExtents(keyMinX, keyMinY, keyMaxX, keyMaxY);
            inter = intersects(_minX, _minY, _maxX, _maxY, keyMinX, keyMinY, keyMaxX, keyMaxY);
        }
    }

    return inter;
}

TileMap* 
TileMapReader::read( const std::string &location )
{
    TileMap *tileMap = NULL;
    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPClient client;
        osg::ref_ptr<HTTPResponse> response = client.get( location );
        if ( response->isOK() && response->getNumParts() > 0 )
        {
            tileMap = read( response->getPartStream( 0 ) );
        }
    }
    else
    {
        std::ifstream in( location.c_str() );
        tileMap = read( in );
    }
    if (tileMap)
    {
        tileMap->_filename = location;
    }
    return tileMap;
}

TileMap*
TileMapReader::read(std::istream &in)
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

    tileMap->_version = e_tile_map->getAttr( ATTR_VERSION );
    tileMap->_tileMapService = e_tile_map->getAttr( ATTR_TILEMAPSERVICE );

    tileMap->_title = e_tile_map->getSubElementText(ELEM_TITLE);
    tileMap->_abstract = e_tile_map->getSubElementText(ELEM_ABSTRACT);
    tileMap->_srs = e_tile_map->getSubElementText(ELEM_SRS);

    //Read the bounding box
    osg::ref_ptr<XmlElement> e_bounding_box = e_tile_map->getSubElement(ELEM_BOUNDINGBOX);
    if (e_bounding_box.valid())
    {
        tileMap->_minX = as<double>(e_bounding_box->getAttr( ATTR_MINX ), 0.0);
        tileMap->_minY = as<double>(e_bounding_box->getAttr( ATTR_MINY ), 0.0);
        tileMap->_maxX = as<double>(e_bounding_box->getAttr( ATTR_MAXX ), 0.0);
        tileMap->_maxY = as<double>(e_bounding_box->getAttr( ATTR_MAXY ), 0.0);
    }

    //Read the origin
    osg::ref_ptr<XmlElement> e_origin = e_tile_map->getSubElement(ELEM_ORIGIN);
    if (e_origin.valid())
    {
        tileMap->_originX = as<double>(e_origin->getAttr( ATTR_X ), 0.0);
        tileMap->_originY = as<double>(e_origin->getAttr( ATTR_Y ), 0.0);
    }

    //Read the tile format
    osg::ref_ptr<XmlElement> e_tile_format = e_tile_map->getSubElement(ELEM_TILE_FORMAT);
    if (e_tile_format.valid())
    {
        tileMap->_format._extension = e_tile_format->getAttr( ATTR_EXTENSION );
        tileMap->_format._mimeType = e_tile_format->getAttr( ATTR_MIME_TYPE);
        tileMap->_format._width = as<unsigned int>(e_tile_format->getAttr( ATTR_WIDTH ), 0);
        tileMap->_format._height = as<unsigned int>(e_tile_format->getAttr( ATTR_HEIGHT ), 0);
    }

    //Read the tilesets
    osg::ref_ptr<XmlElement> e_tile_sets = e_tile_map->getSubElement(ELEM_TILESETS);
    if (e_tile_sets.valid())
    {
        //Read the profile
        std::string profile = e_tile_sets->getAttr( ATTR_PROFILE );
        if (profile == "global-geodetic") tileMap->_profile = GLOBAL_GEODETIC;
        else if (profile == "global-mercator") tileMap->_profile = GLOBAL_MERCATOR;
        else if (profile == "local") tileMap->_profile = LOCAL;
        else tileMap->_profile = NONE;

        //Read each TileSet
        XmlNodeList tile_sets = e_tile_sets->getSubElements( ELEM_TILESET );
        for( XmlNodeList::const_iterator i = tile_sets.begin(); i != tile_sets.end(); i++ )
        {
            osg::ref_ptr<XmlElement> e_tile_set = static_cast<XmlElement*>( i->get() );
            TileSet tileset;
            tileset._href = e_tile_set->getAttr( ATTR_HREF );
            tileset._order = as<unsigned int>(e_tile_set->getAttr( ATTR_ORDER ), -1);
            tileset._unitsPerPixel = as<double>(e_tile_set->getAttr( ATTR_UNITSPERPIXEL ), 0.0 );
            tileMap->_tileSets.push_back(tileset);
        }
    }

    tileMap->computeMinMaxLevel();


    return tileMap.release();
}