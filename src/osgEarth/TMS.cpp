/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/GeoData>
#include <osgEarth/HTTPClient>
#include <osgEarth/XmlUtils>
#include <osgEarth/TMS>
#include <osgEarth/TileKey>
#include <osgEarth/TileSource>
#include <osgEarth/Registry>

#include <limits.h>
#include <iomanip>

using namespace osgEarth;

#define LC "[TMS] "

static std::string toString(double value, int precision = 25)
{
    std::stringstream out;
    out << std::fixed << std::setprecision(precision) << value;
	std::string outStr;
	outStr = out.str();
    return outStr;
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
_originX(0),
_originY(0),
_minX(0.0),
_minY(0.0),
_maxX(0.0),
_maxY(0.0),
_minLevel(0),
_maxLevel(0),
_numTilesWide(-1),
_numTilesHigh(-1)
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
    osg::ref_ptr< SpatialReference > spatialReference =  osgEarth::SpatialReference::create(_srs);
    
    if (spatialReference->isMercator())
    {
        //HACK:  Some TMS sources, most notably TileCache, use a global mercator extent that is very slightly different than
        //       the automatically computed mercator bounds which can cause rendering issues due to the some texture coordinates
        //       crossing the dateline.  If the incoming bounds are nearly the same as our definion of global mercator, just use our definition.
        double eps = 0.01;
        osg::ref_ptr< const Profile > merc = osgEarth::Registry::instance()->getGlobalMercatorProfile();
        if (_numTilesWide == 1 && _numTilesHigh == 1 &&
            osg::equivalent(merc->getExtent().xMin(), _minX, eps) && 
            osg::equivalent(merc->getExtent().yMin(), _minY, eps) &&
            osg::equivalent(merc->getExtent().xMax(), _maxX, eps) &&
            osg::equivalent(merc->getExtent().yMax(), _maxY, eps))
        {            
            return osgEarth::Registry::instance()->getGlobalMercatorProfile();
        }
    }

    

    return Profile::create(
        _srs,
        _minX, _minY, _maxX, _maxY,
        _vsrs,
        osg::maximum(_numTilesWide, (unsigned int)1),
        osg::maximum(_numTilesHigh, (unsigned int)1) );
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
                std::string path = osgDB::getFilePath(_filename);
                ss << path << "/" << zoom << "/" << x << "/" << y << "." << _format.getExtension();
                //OE_NOTICE << LC << "Returning URL " << ss.str() << std::endl;
                std::string ssStr;
				ssStr = ss.str();
				return ssStr;
            }
        }
    }
    else // Just go with it. No way of knowing the max level.
    {
        std::stringstream ss;
        std::string path = osgDB::getFilePath(_filename);
        ss << path << "/" << zoom << "/" << x << "/" << y << "." << _format.getExtension();
        std::string ssStr;
		ssStr = ss.str();
		return ssStr;        
    }

    return "";
}

bool
TileMap::intersectsKey(const TileKey& tilekey)
{
    double keyMinX, keyMinY, keyMaxX, keyMaxY;

    //Check to see if the key overlaps the bounding box using lat/lon.  This is necessary to check even in 
    //Mercator situations in case the BoundingBox is described using lat/lon coordinates such as those produced by GDAL2Tiles
    //This should be considered a bug on the TMS production side, but we can work around it for now...
    tilekey.getExtent().getBounds(keyMinX, keyMinY, keyMaxX, keyMaxY);

    bool inter = intersects(_minX, _minY, _maxX, _maxY, keyMinX, keyMinY, keyMaxX, keyMaxY);

    if (!inter && tilekey.isMercator())
    {
        tilekey.getProfile()->getSRS()->transform(keyMinX, keyMinY, tilekey.getProfile()->getSRS()->getGeographicSRS(), keyMinX, keyMinY);
        tilekey.getProfile()->getSRS()->transform(keyMaxX, keyMaxY, tilekey.getProfile()->getSRS()->getGeographicSRS(), keyMaxX, keyMaxY);
        inter = intersects(_minX, _minY, _maxX, _maxY, keyMinX, keyMinY, keyMaxX, keyMaxY);
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

std::string getSRSString(const osgEarth::SpatialReference* srs)
{
    if (srs->isMercator())
    {
        return "EPSG:900913";
    }
    else if (srs->isGeographic())
    {
        return "EPSG:4326";
    }
    else
    {
        return srs->getInitString(); //srs();
    }	
}


TileMap*
TileMap::create(const std::string& url,
                const Profile* profile,
                const std::string& format,
                int tile_width,
                int tile_height)
{
    //Profile profile(type);

    const GeoExtent& ex = profile->getExtent();

    TileMap* tileMap = new TileMap();
    tileMap->setProfileType(profile->getProfileType()); //type);
    tileMap->setExtents(ex.xMin(), ex.yMin(), ex.xMax(), ex.yMax());
    tileMap->setOrigin(ex.xMin(), ex.yMin());
    tileMap->_filename = url;
    tileMap->_srs = getSRSString(profile->getSRS());
    tileMap->_vsrs = profile->getVerticalSRS() ? profile->getVerticalSRS()->getInitString() : "";
    tileMap->_format.setWidth( tile_width );
    tileMap->_format.setHeight( tile_height );
    tileMap->_format.setExtension( format );
	profile->getNumTiles( 0, tileMap->_numTilesWide, tileMap->_numTilesHigh );

	tileMap->generateTileSets();
	tileMap->computeMinMaxLevel();
        
    return tileMap;
}

TileMap* TileMap::create(const TileSource* tileSource, const Profile* profile)
{
    TileMap* tileMap = new TileMap();

    tileMap->setTitle( tileSource->getName() );
    tileMap->setProfileType( profile->getProfileType() );

    const GeoExtent& ex = profile->getExtent();
    
    tileMap->_srs = getSRSString(profile->getSRS()); //srs();
    tileMap->_vsrs = profile->getVerticalSRS() ? profile->getVerticalSRS()->getInitString() : 0L;
    tileMap->_originX = ex.xMin();
    tileMap->_originY = ex.yMin();
    tileMap->_minX = ex.xMin();
    tileMap->_minY = ex.yMin();
    tileMap->_maxX = ex.xMax();
    tileMap->_maxY = ex.yMax();
    profile->getNumTiles( 0, tileMap->_numTilesWide, tileMap->_numTilesHigh );

    tileMap->_format.setWidth( tileSource->getPixelsPerTile() );
    tileMap->_format.setHeight( tileSource->getPixelsPerTile() );
    tileMap->_format.setExtension( tileSource->getExtension() );

    tileMap->generateTileSets();

    return tileMap;
}



//----------------------------------------------------------------------------


TileMap* 
TileMapReaderWriter::read( const std::string &location, const osgDB::ReaderWriter::Options* options )
{
    TileMap *tileMap = NULL;
    if ( osgDB::containsServerAddress( location ) )
    {
        HTTPResponse response = HTTPClient::get( location, options );
        if (response.isOK() && response.getNumParts() > 0 )
        {
            tileMap = read( response.getPartStream( 0 ) );
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
        OE_DEBUG << LC << "Failed to load TileMap " << std::endl;
        return 0;
    }
   
    //Get the root TileMap element
    osg::ref_ptr<XmlElement> e_tile_map = doc->getSubElement( ELEM_TILEMAP );
    if (!e_tile_map.valid())
    {
        OE_WARN << LC << "Could not find root TileMap element " << std::endl;
        return 0;
    }

    tileMap->setVersion( e_tile_map->getAttr( ATTR_VERSION ) );
    tileMap->setTileMapService( e_tile_map->getAttr( ATTR_TILEMAPSERVICE ) );

    tileMap->setTitle( e_tile_map->getSubElementText(ELEM_TITLE) );
    tileMap->setAbstract( e_tile_map->getSubElementText(ELEM_ABSTRACT) );
    tileMap->setSRS( e_tile_map->getSubElementText(ELEM_SRS) );
    tileMap->setVerticalSRS( e_tile_map->getSubElementText(ELEM_VERTICAL_SRS) );

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
        if (profile == "global-geodetic") tileMap->setProfileType( Profile::TYPE_GEODETIC );
        else if (profile == "global-mercator") tileMap->setProfileType( Profile::TYPE_MERCATOR );
        else if (profile == "local") tileMap->setProfileType( Profile::TYPE_LOCAL );
        else tileMap->setProfileType( Profile::TYPE_UNKNOWN );

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
    if (tileMap->getProfileType() == Profile::TYPE_UNKNOWN && !tileMap->getSRS().empty())
    {
        tileMap->setProfileType( Profile::getProfileTypeFromSRS(tileMap->getSRS()) );
    }

    tileMap->computeMinMaxLevel();
    tileMap->computeNumTiles();

    //Read the data areas
    osg::ref_ptr<XmlElement> e_data_extents = e_tile_map->getSubElement(ELEM_DATA_EXTENTS);
    if (e_data_extents.valid())
    {
        osg::ref_ptr< const osgEarth::Profile > profile = tileMap->createProfile();
        OE_DEBUG << LC << "Found DataExtents " << std::endl;
        XmlNodeList data_extents = e_data_extents->getSubElements( ELEM_DATA_EXTENT );
        for( XmlNodeList::const_iterator i = data_extents.begin(); i != data_extents.end(); i++ )
        {
            osg::ref_ptr<XmlElement> e_data_extent = static_cast<XmlElement*>( i->get() );
            double minX = as<double>(e_data_extent->getAttr( ATTR_MINX ), 0.0);
            double minY = as<double>(e_data_extent->getAttr( ATTR_MINY ), 0.0);
            double maxX = as<double>(e_data_extent->getAttr( ATTR_MAXX ), 0.0);
            double maxY = as<double>(e_data_extent->getAttr( ATTR_MAXY ), 0.0);
            //unsigned int minLevel = as<unsigned int>(e_data_extent->getAttr( ATTR_MIN_LEVEL ), 0);
            unsigned int maxLevel = as<unsigned int>(e_data_extent->getAttr( ATTR_MAX_LEVEL ), 0);            

            //OE_DEBUG << LC << "Read area " << minX << ", " << minY << ", " << maxX << ", " << maxY << ", minlevel=" << minLevel << " maxlevel=" << maxLevel << std::endl;
            tileMap->getDataExtents().push_back( DataExtent(GeoExtent(profile->getSRS(), minX, minY, maxX, maxY), 0, maxLevel));
        }
    }


    return tileMap.release();
}

static XmlDocument*
tileMapToXmlDocument(const TileMap* tileMap)
{
    //Create the root XML document
    osg::ref_ptr<XmlDocument> doc = new XmlDocument();
    doc->setName( ELEM_TILEMAP );
    
    //Create the root node
    //osg::ref_ptr<XmlElement> e_tile_map = new XmlElement( ELEM_TILEMAP );
    //doc->getChildren().push_back( e_tile_map.get() );

    doc->getAttrs()[ ATTR_VERSION ] = tileMap->getVersion();
    doc->getAttrs()[ ATTR_TILEMAPSERVICE ] = tileMap->getTileMapService();
  
    doc->addSubElement( ELEM_TITLE, tileMap->getTitle() );
    doc->addSubElement( ELEM_ABSTRACT, tileMap->getAbstract() );
    doc->addSubElement( ELEM_SRS, tileMap->getSRS() );
    doc->addSubElement( ELEM_VERTICAL_SRS, tileMap->getVerticalSRS() );

    osg::ref_ptr<XmlElement> e_bounding_box = new XmlElement( ELEM_BOUNDINGBOX );
    double minX, minY, maxX, maxY;
    tileMap->getExtents( minX, minY, maxX, maxY );
    e_bounding_box->getAttrs()[ATTR_MINX] = toString(minX);
    e_bounding_box->getAttrs()[ATTR_MINY] = toString(minY);
    e_bounding_box->getAttrs()[ATTR_MAXX] = toString(maxX);
    e_bounding_box->getAttrs()[ATTR_MAXY] = toString(maxY);
    doc->getChildren().push_back(e_bounding_box.get() );

    osg::ref_ptr<XmlElement> e_origin = new XmlElement( ELEM_ORIGIN );
    e_origin->getAttrs()[ATTR_X] = toString(tileMap->getOriginX());
    e_origin->getAttrs()[ATTR_Y] = toString(tileMap->getOriginY());
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
    else if (profile->isEquivalentTo(osgEarth::Registry::instance()->getGlobalMercatorProfile()))
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
        e_tile_set->getAttrs()[ATTR_UNITSPERPIXEL] = toString(itr->getUnitsPerPixel());
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
            e_data_extent->getAttrs()[ATTR_MINX] = toString(itr->xMin());
            e_data_extent->getAttrs()[ATTR_MINY] = toString(itr->yMin());
            e_data_extent->getAttrs()[ATTR_MAXX] = toString(itr->xMax());
            e_data_extent->getAttrs()[ATTR_MAXY] = toString(itr->yMax());
            e_data_extent->getAttrs()[ATTR_MIN_LEVEL] = toString<unsigned int>(itr->getMinLevel());
            e_data_extent->getAttrs()[ATTR_MAX_LEVEL] = toString<unsigned int>(itr->getMaxLevel());
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


