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

#ifndef OSGEARTH_ARCGIS_MAP_SERVICE_H
#define OSGEARTH_ARCGIS_MAP_SERVICE_H 1

#include <osgEarth/Profile>
#include <osgEarth/URI>
#include <list>
#include "Extent.h"

using namespace osgEarth;

class MapServiceLayer 
{
public:
    MapServiceLayer( int id, const std::string& name );

    int getId() const;

    const std::string& getName() const;

private:
    int id;
    std::string name;
};

typedef std::list<MapServiceLayer> MapServiceLayerList;


/**
 * The tiling scheme defined by the map service.
 */
class TileInfo
{
public:
    TileInfo();

    TileInfo( int tile_size, const std::string& format, int min_level, int max_level, int _num_tiles_wide, int _num_tiles_high );

    TileInfo( const TileInfo& rhs );

    bool isValid() const;

    int getTileSize() const;

    const std::string& getFormat() const;

    int getMinLevel() const;

    int getMaxLevel() const;

    int getNumTilesWide() const;

    int getNumTilesHigh() const;

private:
    std::string format;
    int tile_size;
    int min_level, max_level;
    bool is_valid;
    int num_tiles_wide;
    int num_tiles_high;
};


/**
 * ESRI ArcGIS Server Map Service interface.
 */
class MapService
{
public:
    MapService();

    /**
     * Initializes a map service interface and populates its metadata from the
     * provided REST API URL (e.g.: http://server/ArcGIS/rest/services/MyMapService)
     * Call isValid() to verify success.
     */
    bool init( const URI& uri, const osgDB::Options* options =0L );

    /**
     * Returns true if the map service initialized succesfully.
     */
    bool isValid() const;

    bool isTiled() const;

    /** 
     * If isValid() returns false, this method will return the error message.
     */
    const std::string& getError() const;

    /**
     * Gets the data profile associated with this map service.
     */
    const Profile* getProfile() const;

    /**
     * Gets the tile information for this service.
     */
    const TileInfo& getTileInfo() const;

private:
    bool is_valid;
    URI uri;
    osg::ref_ptr<const Profile> profile;
    std::string error_msg;
    MapServiceLayerList layers;
    bool tiled;
    TileInfo tile_info;

    bool setError( const std::string& );
};

#endif // OSGEARTH_ARCGIS_MAP_SERVICE_H
