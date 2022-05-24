/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#ifdef OSGEARTH_HAVE_MVT

#include <osgEarth/MVT>

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/GeoData>
#include <osgEarth/FeatureSource>
#include <osgDB/Registry>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include "vector_tile.pb.h"

#ifdef OSGEARTH_HAVE_SQLITE3
#include <sqlite3.h>
#endif

using namespace osgEarth;
using namespace osgEarth::MVT;

#define LC "[MVT] "

#define CMD_BITS 3
#define CMD_MOVETO 1
#define CMD_LINETO 2
#define CMD_CLOSEPATH 7

namespace osgEarth { namespace MVT
{
    // https://github.com/mapbox/mapnik-vector-tile/blob/master/examples/c%2B%2B/tileinfo.cpp
    enum CommandType {
        SEG_END    = 0,
        SEG_MOVETO = 1,
        SEG_LINETO = 2,
        SEG_CLOSE = (0x40 | 0x0f)
    };

    enum eGeomType {
        Unknown = 0,
        Point = 1,
        LineString = 2,
        Polygon = 3
    };

    int zig_zag_decode(int n)
    {
        return (n >> 1) ^ (-(n & 1));
    }

    Geometry* decodeLine(const mapnik::vector::tile_feature& feature, const TileKey& key, unsigned int tileres)
    {
        unsigned int length = 0;
        int cmd = -1;
        const int cmd_bits = 3;

        int x = 0;
        int y = 0;

        std::vector< osg::ref_ptr< osgEarth::LineString > > lines;
        osg::ref_ptr< osgEarth::LineString > currentLine;

        GeoExtent extent = key.getExtent();

        for (int k = 0; k < feature.geometry_size();)
        {
            if (!length)
            {
                unsigned int cmd_length = feature.geometry(k++);
                cmd = cmd_length & ((1 << cmd_bits) - 1);
                length = cmd_length >> cmd_bits;
            }
            if (length > 0)
            {
                length--;

                if (cmd == SEG_MOVETO || cmd == SEG_LINETO)
                {
                    if (cmd == SEG_MOVETO)
                    {
                        currentLine = new osgEarth::LineString;
                        lines.push_back( currentLine.get() );
                    }
                    int px = feature.geometry(k++);
                    int py = feature.geometry(k++);
                    px = zig_zag_decode(px);
                    py = zig_zag_decode(py);

                    x += px;
                    y += py;

                    double width = extent.width();
                    double height = extent.height();

                    double geoX = extent.xMin() + (width/(double)tileres) * (double)x;
                    double geoY = extent.yMax() - (height/(double)tileres) * (double)y;

                    if (currentLine.valid())
                    {
                        currentLine->push_back(geoX, geoY, 0);
                    }
                }
            }
        }

        currentLine = 0;

        if (lines.size() == 0)
        {
            return 0;
        }
        else if (lines.size() == 1)
        {
            // Just return a simple LineString
            return lines[0].release();
        }
        else
        {
            // Return a multilinestring
            MultiGeometry* multi = new MultiGeometry;
            for (unsigned int i = 0; i < lines.size(); i++)
            {
                multi->add(lines[i].get());
            }
            return multi;
        }
    }

    Geometry* decodePoint(const mapnik::vector::tile_feature& feature, const TileKey& key, unsigned int tileres)
    {
        unsigned int length = 0;
        int cmd = -1;
        const int cmd_bits = 3;

        int x = 0;
        int y = 0;

        osgEarth::PointSet *geometry = new osgEarth::PointSet();

        GeoExtent extent = key.getExtent();

        for (int k = 0; k < feature.geometry_size();)
        {
            if (!length)
            {
                unsigned int cmd_length = feature.geometry(k++);
                cmd = cmd_length & ((1 << cmd_bits) - 1);
                length = cmd_length >> cmd_bits;
            }
            if (length > 0)
            {
                length--;
                if (cmd == SEG_MOVETO || cmd == SEG_LINETO)
                {
                    int px = feature.geometry(k++);
                    int py = feature.geometry(k++);
                    px = zig_zag_decode(px);
                    py = zig_zag_decode(py);

                    x += px;
                    y += py;

                    double width = extent.width();
                    double height = extent.height();

                    double geoX = extent.xMin() + (width/(double)tileres) * (double)x;
                    double geoY = extent.yMax() - (height/(double)tileres) * (double)y;
                    geometry->push_back(geoX, geoY, 0);
                }
            }
        }

        return geometry;
    }

    Geometry* decodePolygon(const mapnik::vector::tile_feature& feature, const  TileKey& key, unsigned int tileres)
    {
        /*
         https://github.com/mapbox/vector-tile-spec/tree/master/2.1
         Decoding polygons is a bit more difficult than lines or points.
         A Polygon geometry is either a single polygon or a multipolygon.  Each polygon has one exterior ring and zero or more interior rings.
         The rings are in sequence and you must check the orientation of the ring to know if it's an exterior ring (new polygon) or an
         interior ring (inner polygon of the current polygon).
         */


        unsigned int length = 0;
        int cmd = -1;
        const int cmd_bits = 3;

        int x = 0;
        int y = 0;

        // The list of polygons we've collected
        std::vector< osg::ref_ptr< osgEarth::Polygon > > polygons;

        osg::ref_ptr< osgEarth::Polygon > currentPolygon;

        osg::ref_ptr< osgEarth::Ring > currentRing;

        GeoExtent extent = key.getExtent();

        for (int k = 0; k < feature.geometry_size();)
        {
            if (!length)
            {
                unsigned int cmd_length = feature.geometry(k++);
                cmd = cmd_length & ((1 << cmd_bits) - 1);
                length = cmd_length >> cmd_bits;
            }
            if (length > 0)
            {
                length--;
                if (cmd == SEG_MOVETO || cmd == SEG_LINETO)
                {
                    if (!currentRing)
                    {
                        currentRing = new osgEarth::Ring();
                    }

                    int px = feature.geometry(k++);
                    int py = feature.geometry(k++);
                    px = zig_zag_decode(px);
                    py = zig_zag_decode(py);

                    x += px;
                    y += py;

                    double width = extent.width();
                    double height = extent.height();

                    double geoX = extent.xMin() + (width/(double)tileres) * (double)x;
                    double geoY = extent.yMax() - (height/(double)tileres) * (double)y;
                    currentRing->push_back(geoX, geoY, 0);
                }
                else if (cmd == (SEG_CLOSE & ((1 << cmd_bits) - 1)))
                {                    
                    double area = currentRing->getSignedArea2D();

                    // Close the ring.
                    currentRing->close();

                    // New polygon
                    if (area > 0)
                    {
                        currentRing->rewind(Geometry::ORIENTATION_CCW);
                        currentPolygon = new osgEarth::Polygon(&currentRing->asVector());
                        polygons.push_back(currentPolygon.get());
                    }
                    // Hole
                    else if (area < 0)
                    {
                        if (currentPolygon.valid())
                        {
                            currentRing->rewind(Geometry::ORIENTATION_CW);
                            currentPolygon->getHoles().push_back( currentRing );
                        }
                        else
                        {
                            // this means we encountered a "hole" without a parent outer ring,
                            // discard for now -gw
                            OE_INFO << LC << "Discarding improperly wound polygon (hole without an outer ring)\n";
                        }
                    }

                    // Start a new ring
                    currentRing = 0;
                }
            }
        }

        currentRing = 0;
        currentPolygon = 0;

        if (polygons.size() == 0)
        {
            return 0;
        }
        else if (polygons.size() == 1)
        {
            // Just return a simple polygon
            return polygons[0].release();
        }
        else
        {
            // Return a multipolygon
            MultiGeometry* multi = new MultiGeometry;
            for (unsigned int i = 0; i < polygons.size(); i++)
            {
                multi->add(polygons[i].get());
            }
            return multi;
        }
    }

    bool readTile(std::istream& in, const TileKey& key, FeatureList& features)
    {
        features.clear();

        // Get the compressor
        osg::ref_ptr< osgDB::BaseCompressor> compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
        if (!compressor.valid())
        {
            return false;
        }

        // Decompress the tile
        std::string original((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.seekg (0, std::ios::beg);
        std::string value;
        if (!compressor->decompress(in, value))
        {
            value = original;
        }


        mapnik::vector::tile tile;

        if (tile.ParseFromString(value))
        {
            for (int i = 0; i < tile.layers().size(); i++)
            {
                const mapnik::vector::tile_layer &layer = tile.layers().Get(i);

                for (int j = 0; j < layer.features().size(); j++)
                {
                    const mapnik::vector::tile_feature &feature = layer.features().Get(j);

                    osg::ref_ptr< Feature > oeFeature = new Feature(0, key.getProfile()->getSRS());

                    // Set the layer name as "mvt_layer" so we can filter it later
                    oeFeature->set("mvt_layer", layer.name());

                    // Read attributes
                    for (int k = 0; k < feature.tags().size(); k+=2)
                    {
                        std::string key = layer.keys().Get(feature.tags().Get(k));
                        mapnik::vector::tile_value value = layer.values().Get(feature.tags().Get(k+1));

                        if (value.has_bool_value())
                        {
                            oeFeature->set(key, value.bool_value());
                        }
                        else if (value.has_double_value())
                        {
                            oeFeature->set(key, value.double_value());
                        }
                        else if (value.has_float_value())
                        {
                            oeFeature->set(key, value.float_value());
                        }
                        else if (value.has_int_value())
                        {
                            oeFeature->set(key, (long long)value.int_value());
                        }
                        else if (value.has_sint_value())
                        {
                            oeFeature->set(key, (long long)value.sint_value());
                        }
                        else if (value.has_string_value())
                        {
                            oeFeature->set(key, value.string_value());
                        }
                        else if (value.has_uint_value())
                        {
                            oeFeature->set(key, (long long)value.uint_value());
                        }

                        // Special path for getting heights from our test dataset.
                        if (key == "other_tags")
                        {
                            std::string other_tags = value.string_value();

                            StringTokenizer tok("=>");
                            StringVector tized;
                            tok.tokenize(other_tags, tized);
                            if (tized.size() == 3)
                            {
                                if (tized[0] == "height")
                                {
                                    std::string value = tized[2];
                                    // Remove quotes from the height
                                    float height = as<float>(value, FLT_MAX);
                                    if (height != FLT_MAX)
                                    {
                                        oeFeature->set("height", height);
                                    }
                                }
                            }
                        }
                    }



                    osg::ref_ptr< osgEarth::Geometry > geometry;

                    eGeomType geomType = static_cast<eGeomType>(feature.type());
                    if (geomType == MVT::Polygon)
                    {
                        geometry = decodePolygon(feature, key, layer.extent());
                    }
                    else if (geomType == MVT::LineString)
                    {
                        geometry = decodeLine(feature, key, layer.extent());
                    }
                    else if (geomType == MVT::Point)
                    {
                        geometry = decodePoint(feature, key, layer.extent());

                        // This is a bit of a hack, but if a point is outside of the extents we remove it.
                        // Lines and Polygons that extend outside of the tileset we keep though b/c we assume that they are just slightly going outside of the
                        // extent.  Should probably make this an option somewhere.
                        if (geometry)
                        {
                            if (!key.getExtent().contains(geometry->getBounds().center()))
                            {
                                geometry = NULL;
                            }
                        }
                    }
                    else
                    {
                        geometry = decodeLine(feature, key, layer.extent());
                    }

                    if (geometry)
                    {
                        oeFeature->setGeometry( geometry.get() );
                        features.push_back(oeFeature.get());
                    }

                }
            }
        }
        else
        {
            OE_WARN << "Failed to parse mvt" << key.str() << std::endl;
            return false;
        }

        return true;
    }

}} // namespace osgEarth::MVT

//........................................................................

Config
MVTFeatureSourceOptions::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    conf.set("url", url());
    return conf;
}

void
MVTFeatureSourceOptions::fromConfig(const Config& conf)
{
    conf.get("url", url());
}

//........................................................................

REGISTER_OSGEARTH_LAYER(mvtfeatures, MVTFeatureSource);

OE_LAYER_PROPERTY_IMPL(MVTFeatureSource, URI, URL, url);


Status
MVTFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    std::string fullFilename = options().url()->full();

    sqlite3** dbptr = (sqlite3**)(&_database);
    int rc = sqlite3_open_v2(fullFilename.c_str(), dbptr, SQLITE_OPEN_READONLY, 0L);
    if (rc != 0)
    {
        return Status(Status::ResourceUnavailable, Stringify() << "Failed to open database, " << sqlite3_errmsg((sqlite3*)_database));
    }

    setFeatureProfile(createFeatureProfile());

    return Status::NoError;
}

void
MVTFeatureSource::init()
{
    FeatureSource::init();

    _minLevel = 0u;
    _maxLevel = 14u;
    _database = 0L;

    _compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
    if (!_compressor.valid())
    {
        OE_WARN << LC << "Failed to get zlib compressor" << std::endl;
    }
}

FeatureCursor*
MVTFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress)
{
    if (!query.tileKey().isSet())
    {
        OE_WARN << LC << "No tile key in query; no features will be returned\n";
        return 0L;
    }

    TileKey key = *query.tileKey();

    int z = key.getLevelOfDetail();
    int tileX = key.getTileX();
    int tileY = key.getTileY();

    unsigned int numRows, numCols;
    key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
    tileY = numRows - tileY - 1;

    //Get the image
    sqlite3_stmt* select = NULL;
    std::string queryStr = "SELECT tile_data from tiles where zoom_level = ? AND tile_column = ? AND tile_row = ?";
    int rc = sqlite3_prepare_v2((sqlite3*)_database, queryStr.c_str(), -1, &select, 0L);
    if (rc != SQLITE_OK)
    {
        OE_WARN << LC << "Failed to prepare SQL: " << queryStr << "; "
            << sqlite3_errmsg((sqlite3*)_database) << std::endl;
        return NULL;
    }

    bool valid = true;

    sqlite3_bind_int(select, 1, z);
    sqlite3_bind_int(select, 2, tileX);
    sqlite3_bind_int(select, 3, tileY);

    rc = sqlite3_step(select);

    FeatureList features;

    if (rc == SQLITE_ROW)
    {
        // the pointer returned from _blob gets freed internally by sqlite, supposedly
        const char* data = (const char*)sqlite3_column_blob(select, 0);
        int dataLen = sqlite3_column_bytes(select, 0);
        std::string dataBuffer(data, dataLen);
        std::stringstream in(dataBuffer);
        MVT::readTile(in, key, features);
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << queryStr << ": " << std::endl;
        valid = false;
    }

    sqlite3_finalize(select);

    // apply filters before returning.
    applyFilters(features, query.tileKey()->getExtent());

    // If we have any features and we have an fid attribute, override the fid of the features
    if (options().fidAttribute().isSet())
    {
        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
        {
            std::string attr = itr->get()->getString(options().fidAttribute().get());
            FeatureID fid = as<FeatureID>(attr, 0);
            itr->get()->setFID(fid);
        }
    }

    if (!features.empty())
    {
        //OE_NOTICE << "Returning " << features.size() << " features" << std::endl;
        return new FeatureListCursor(features);
    }

    return 0;
}

void
MVTFeatureSource::iterateTiles(int zoomLevel, int limit, int offset, const GeoExtent& extent, FeatureTileCallback callback, void* context)
{
    const Profile* profile = getFeatureProfile()->getTilingProfile();
    unsigned int numRows, numCols;
    profile->getNumTiles(zoomLevel, numCols, numRows);

    sqlite3_stmt* select = NULL;
    std::stringstream buf;
    buf << "SELECT zoom_level, tile_column, tile_row, tile_data from tiles";

    if (extent.isValid())
    {
        GeoExtent featureExtent = extent.transform(profile->getSRS());

        // Limit the x and y values based on the extent
        osgEarth::TileKey ll = profile->createTileKey(featureExtent.xMin(), featureExtent.yMin(), zoomLevel);
        osgEarth::TileKey ur = profile->createTileKey(featureExtent.xMax(), featureExtent.yMax(), zoomLevel);

        unsigned int minX = ll.getTileX();
        unsigned int maxX = ur.getTileX();
        unsigned int minY = numRows - ll.getTileY() - 1;
        unsigned int maxY = numRows - ur.getTileY() - 1;

        buf << " WHERE tile_column >= " << minX << " AND tile_column <= " << maxX << " AND tile_row >= " << minY << " AND tile_row <= " << maxY;
    }

    if (limit > 0)
    {
        buf << " LIMIT " << limit;
    }

    if (offset > 0)
    {
        buf << " OFFSET " << offset;
    }

    std::string queryStr = buf.str();
    int rc = sqlite3_prepare_v2((sqlite3*)_database, queryStr.c_str(), -1, &select, 0L);
    if (rc != SQLITE_OK)
    {
        OE_WARN << LC << "Failed to prepare SQL: " << queryStr << "; "
            << sqlite3_errmsg((sqlite3*)_database) << std::endl;
    }

    while ((rc = sqlite3_step(select)) == SQLITE_ROW) {
        int zoom = sqlite3_column_int(select, 0);
        int tile_column = sqlite3_column_int(select, 1);
        int tile_row = sqlite3_column_int(select, 2);

        TileKey key(zoom, tile_column, numRows - tile_row - 1, profile);

        // the pointer returned from _blob gets freed internally by sqlite, supposedly
        const char* data = (const char*)sqlite3_column_blob(select, 3);
        int dataLen = sqlite3_column_bytes(select, 3);
        std::string dataBuffer(data, dataLen);
        std::stringstream in(dataBuffer);

        FeatureList features;

        // If we have any features and we have an fid attribute, override the fid of the features
        if (options().fidAttribute().isSet())
        {
            for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
            {
                std::string attr = itr->get()->getString(options().fidAttribute().get());
                FeatureID fid = as<long>(attr, 0);
                itr->get()->setFID(fid);

            }
        }


        MVT::readTile(in, key, features);

        // apply filters before returning.
        applyFilters(features, key.getExtent());

        if (features.size() > 0)
        {
            //tiles[key] = features;
            callback(key, features, context);
        }
    }

    sqlite3_finalize(select);
}


const FeatureProfile*
MVTFeatureSource::createFeatureProfile()
{
    const osgEarth::Profile* profile = nullptr;

    std::string profileStr;
    getMetaData("profile", profileStr);

    if (!profileStr.empty())
    {
        // try to parse it as a JSON config
        Config pconf;
        pconf.fromJSON(profileStr);
        profile = Profile::create(ProfileOptions(pconf));

        // if that didn't work, try parsing it directly
        if (!profile)
        {
            profile = Profile::create(profileStr);
        }

        if (!profile)
        {
            OE_WARN << LC << "Failed to create Profile from string " << profileStr << std::endl;
        }
    }

    // Default to spherical mercator if nothing was specified in the profile config.
    if (!profile)
    {
        profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
    }


    FeatureProfile* result = new FeatureProfile(profile->getExtent());
    computeLevels();
    OE_INFO << LC << "Got levels from database " << _minLevel << ", " << _maxLevel << std::endl;


    // Use the max level for now as the min level.
    result->setFirstLevel(_minLevel);
    result->setMaxLevel(_maxLevel);
    result->setTilingProfile(profile);
    result->geoInterp() = osgEarth::GEOINTERP_GREAT_CIRCLE;
    return result;
}

void
MVTFeatureSource::computeLevels()
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();
    sqlite3_stmt* select = NULL;
    // Get min and max as separate queries to allow the SQLite query planner to convert it to a fast equivalent.
    std::string query = "SELECT (SELECT min(zoom_level) FROM tiles), (SELECT max(zoom_level) FROM tiles); ";
    int rc = sqlite3_prepare_v2((sqlite3*)_database, query.c_str(), -1, &select, 0L);
    if (rc != SQLITE_OK)
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg((sqlite3*)_database) << std::endl;
    }

    rc = sqlite3_step(select);
    if (rc == SQLITE_ROW)
    {
        _minLevel = sqlite3_column_int(select, 0);
        _maxLevel = sqlite3_column_int(select, 1);
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
    }
    sqlite3_finalize(select);
    osg::Timer_t endTime = osg::Timer::instance()->tick();
    OE_DEBUG << LC << "Computing levels took " << osg::Timer::instance()->delta_s(startTime, endTime) << " s" << std::endl;
}

bool
MVTFeatureSource::getMetaData(const std::string& key, std::string& value)
{
    //get the metadata
    sqlite3_stmt* select = NULL;
    std::string query = "SELECT value from metadata where name = ?";
    int rc = sqlite3_prepare_v2((sqlite3*)_database, query.c_str(), -1, &select, 0L);
    if (rc != SQLITE_OK)
    {
        OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg((sqlite3*)_database) << std::endl;
        return false;
    }

    bool valid = true;
    std::string keyStr = std::string(key);
    rc = sqlite3_bind_text(select, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC);
    if (rc != SQLITE_OK)
    {
        OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg((sqlite3*)_database) << std::endl;
        return false;
    }

    rc = sqlite3_step(select);
    if (rc == SQLITE_ROW)
    {
        value = (char*)sqlite3_column_text(select, 0);
    }
    else
    {
        OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
        valid = false;
    }

    sqlite3_finalize(select);
    return valid;
}

#endif // OSGEARTH_HAVE_MVT
