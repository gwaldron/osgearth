/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include "TileService"

#include <osgEarth/XmlUtils>

#include <osg/io_utils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace std;

namespace
{
    std::string extractBetween(const std::string& str, const string &lhs, const string &rhs)
    {
        std::string result;
        string::size_type start = str.find(lhs);
        if (start != string::npos)
        {
            start += lhs.length();
            string::size_type count = str.size() - start;
            string::size_type end = str.find(rhs, start); 
            if (end != string::npos) count = end-start;
            result = str.substr(start, count);
        }
        return result;
    }
}

TilePattern::TilePattern(const std::string& pattern)
{
    _pattern = pattern;
    init();
}



void TilePattern::init()
{
    _dataMin.x() = -180;
    _dataMin.y() = -90;
    _dataMax.x() = 180;
    _dataMax.y() = 90;

    //request=GetMap&layers=global_mosaic&srs=EPSG:4326&format=image/jpeg&styles=visual&width=512&height=512&bbox=-180,-166,76,90
    //Convert the filename to lower case
    std::string lower = osgDB::convertToLowerCase( _pattern );

    _layers = extractBetween(lower, "layers=", "&");
    _styles = extractBetween(lower, "styles=", "&");
    _srs = extractBetween(lower, "srs=", "&");
    _format = extractBetween(lower, "format=image/", "&");
    _imageWidth = as<int>(extractBetween(lower, "width=", "&"), 0);
    _imageHeight = as<int>(extractBetween(lower, "height=", "&"), 0);

    //Read the coordinates of the top left tile
    std::string bbox = extractBetween(lower, "bbox=", "&");
    sscanf(bbox.c_str(), "%lf,%lf,%lf,%lf", &_topLeftMin.x(), &_topLeftMin.y(), &_topLeftMax.x(), &_topLeftMax.y());

    //Compute the tile dimensions
    _tileWidth = _topLeftMax.x() - _topLeftMin.x();
    _tileHeight = _topLeftMax.y() - _topLeftMin.y();

    //Create the prototype
    string::size_type len = lower.find( bbox );
    if (len != string::npos)
    {
        string beforeBB = _pattern.substr(0, len);

        string::size_type after = len + bbox.length();
        string afterBB = "";
        if (after < _pattern.length()-1)
        {
            afterBB = _pattern.substr(after, _pattern.length() - after);
        }
        _prototype = beforeBB + std::string("%lf,%lf,%lf,%lf") + afterBB;
    }
}

void TilePattern::getTileBounds(const int &x, const int &y, double &minX, double &minY, double &maxX, double &maxY)
{
    minX = _topLeftMin.x() + (double)x * _tileWidth;
    maxX = minX + _tileWidth;

    maxY = _topLeftMax.y() - (double)y * _tileHeight;
    minY = maxY - _tileHeight;
}

std::string TilePattern::getRequestString(const int &x, const int &y)
{
    double minX, minY, maxX, maxY;
    getTileBounds(x, y, minX, minY, maxX, maxY);

    char buf[2048];
    sprintf(buf, _prototype.c_str(), minX, minY, maxX, maxY);
    return buf;
}


TileService::TileService():
_dataMin(-180, -90),
_dataMax(180, 90)
{
}

void TileService::getMatchingPatterns(const std::string &layers, const std::string &format,
                                      const std::string &styles, const std::string &srs,
                                      unsigned int imageWidth, unsigned int imageHeight,
                                      TilePatternList& out_patterns)
{
    out_patterns.clear();
    for (TilePatternList::iterator i = _patterns.begin(); i < _patterns.end(); ++i)
    {
        if (osgDB::equalCaseInsensitive(i->getLayers(), layers) &&
            osgDB::equalCaseInsensitive(i->getFormat(),format) &&
            osgDB::equalCaseInsensitive(i->getStyles(), styles) &&
            osgDB::equalCaseInsensitive(i->getSRS(), srs) &&
            (i->getImageWidth()  == (int)imageWidth) &&
            (i->getImageHeight() == (int)imageHeight))
        {
            out_patterns.push_back(*i);
        }
    }
}

const Profile*
TileService::createProfile(TilePatternList &patterns)
{
    //Assume that all the values in the patterns are equal except for the bounding boxes
    const Profile* profile = NULL;

    if (patterns.size() > 0)
    {
      double maxWidth = -1;
      double maxHeight = -1;

      osg::Vec2d topLeftMin;
      osg::Vec2d topLeftMax;

      //Find the lowest resolution pattern.
      for (unsigned int i = 0; i < patterns.size(); ++i)
      {
          if (maxWidth < patterns[i].getTileWidth() &&
              maxHeight < patterns[i].getTileHeight())
          {
              maxWidth = patterns[i].getTileWidth();
              maxHeight = patterns[i].getTileHeight();

              topLeftMin = patterns[i].getTopLeftMin();
              topLeftMax = patterns[i].getTopLeftMax();
          }
      }

      double dataWidth = _dataMax.x() - _dataMin.x();
      double dataHeight = _dataMax.y() - _dataMin.y();

      double tileWidth = topLeftMax.x() - topLeftMin.x();
      double tileHeight = topLeftMax.y() - topLeftMin.y();

      unsigned int w = (unsigned int) ceil(dataWidth / tileWidth );
      unsigned int h = (unsigned int) ceil(dataHeight / tileHeight);

      double xmin = topLeftMin.x();
      double xmax = xmin + (double)w * tileWidth;
      double ymax = topLeftMax.y();
      double ymin = ymax - (double)h * tileHeight;
      
      profile = Profile::create( patterns[0].getSRS(), xmin, ymin, xmax, ymax, "", w, h);
    }

    return profile;
}

#define ELEM_WMS_TILE_SERVICE "wms_tile_service"
#define ELEM_SERVICE          "service"
#define ATTR_VERSION          "version"
#define ELEM_NAME             "name"
#define ELEM_TITLE            "title"
#define ELEM_ABSTRACT         "abstract"
#define ELEM_ACCESSCONSTRAINTS "accessconstraints"

#define ELEM_TILEDPATTERNS     "tiledpatterns"
#define ELEM_TILEPATTERN       "tilepattern"
#define ELEM_TILEDGROUP        "tiledgroup"
#define ELEM_LATLONBOUNDINGBOX "latlonboundingbox"
#define ATTR_MINX              "minx"
#define ATTR_MINY              "miny"
#define ATTR_MAXX              "maxx"
#define ATTR_MAXY              "maxy"

TileService* 
TileServiceReader::read( const std::string &location, const osgDB::ReaderWriter::Options* options )
{
    TileService *tileService = NULL;

    ReadResult r = URI(location).readString( options );
    if ( r.succeeded() )
    {
        std::istringstream buf( r.getString() );
        tileService = read( buf );
    }

    return tileService;
}

namespace
{
    void readBoundingBox(XmlElement* e_bb, double &minX, double &minY, double &maxX, double &maxY)
    {
        if (e_bb)
        {
            minX = as<double>(e_bb->getAttr( ATTR_MINX ), minX);
            minY = as<double>(e_bb->getAttr( ATTR_MINY ), minY);
            maxX = as<double>(e_bb->getAttr( ATTR_MAXX ), maxX);
            maxY = as<double>(e_bb->getAttr( ATTR_MAXY ), maxY);
        }
    }

    void addTilePatterns(XmlElement* e_root, TileService* tileService)
    {
        //Read all the TilePatterns
        XmlNodeList tile_patterns = e_root->getSubElements( ELEM_TILEPATTERN );
        for( XmlNodeList::const_iterator i = tile_patterns.begin(); i != tile_patterns.end(); i++ )
        {            
            //We only really care about a single access pattern, so extract it
            string txt = static_cast<XmlElement*>( i->get() )->getText();
            //Access patterns are separated by whitespace 
            std::string whitespace (" \t\f\v\n\r");
            string::size_type len = txt.find_first_of(whitespace);
            if (len != string::npos)
            {
                txt = trim(txt.substr(0, len));
            }
            TilePattern pattern(txt);
            tileService->getPatterns().push_back(pattern);
        }

        //Read all TilePatterns in the TiledGroups
        XmlNodeList tiled_groups = e_root->getSubElements(ELEM_TILEDGROUP);
        for( XmlNodeList::const_iterator i = tiled_groups.begin(); i != tiled_groups.end(); i++ )
        {
            addTilePatterns(static_cast<XmlElement*>(i->get()), tileService);
        }
    }
}

TileService*
TileServiceReader::read(std::istream &in)
{
    osg::ref_ptr<TileService> tileService = new TileService;

    osg::ref_ptr<XmlDocument> doc = XmlDocument::load( in );
    if (!doc.valid())
    {
        OE_INFO << "Failed to load TileService " << std::endl;
        return 0;
    }
    
    //Get the root TileMap element
    osg::ref_ptr<XmlElement> e_root = doc->getSubElement( ELEM_WMS_TILE_SERVICE );
    if (!e_root.valid())
    {
        OE_INFO << "Could not find root TileService element " << std::endl;
        return 0;
    }

    tileService->setVersion( e_root->getAttr( ATTR_VERSION ) );

    //Get properties from the Service element.
    osg::ref_ptr<XmlElement> e_service = e_root->getSubElement( ELEM_SERVICE );
    if (!e_service.valid())
    {
        OE_INFO << "Could not find Service element " << std::endl;
        return 0;
    }

    tileService->setName( e_service->getSubElementText( ELEM_NAME ) );
    tileService->setTitle( e_service->getSubElementText( ELEM_TITLE ) );
    tileService->setAbstract( e_service->getSubElementText( ELEM_ABSTRACT ) );
    tileService->setAccessConstraints( e_service->getSubElementText( ELEM_ACCESSCONSTRAINTS ) );

    //Get the TiledPattern element which contains the TiledGroups
    osg::ref_ptr<XmlElement> e_tiledPatterns = e_root->getSubElement( ELEM_TILEDPATTERNS );
    if (!e_tiledPatterns.valid())
    {
        OE_INFO << "Could not find TiledPatterns element" << std::endl;
        return 0;
    }

    //Get the bounding box from the TiledPatterns
    osg::ref_ptr<XmlElement> e_bb = e_tiledPatterns->getSubElement( ELEM_LATLONBOUNDINGBOX );
    if (e_bb.valid())
    {
      double minX, minY, maxX, maxY;
      readBoundingBox(e_bb.get(), minX, minY, maxX, maxY);
      tileService->setDataMin(osg::Vec2d(minX, minY));
      tileService->setDataMax(osg::Vec2d(maxX, maxY));
    }

    addTilePatterns(e_tiledPatterns.get(), tileService.get());

    OE_INFO << "Returning TileService with " << tileService->getPatterns().size() << " patterns " << std::endl;
    return tileService.release();
}
