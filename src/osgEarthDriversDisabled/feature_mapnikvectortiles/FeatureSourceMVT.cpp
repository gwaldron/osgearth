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
#include "MVTFeatureOptions"

#include <osgEarth/Registry>
#include <osgEarth/XmlUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/GeoData>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ScaleFilter>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>

#include "vector_tile.pb.h"

#define LC "[MVT FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define CMD_BITS 3
#define CMD_MOVETO 1
#define CMD_LINETO 2
#define CMD_CLOSEPATH 7

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

class MVTFeatureSource : public FeatureSource
{
public:
    MVTFeatureSource(const MVTFeatureOptions& options ) :
      FeatureSource( options ),
      _options     ( options )
    {
        _compressor = osgDB::Registry::instance()->getObjectWrapperManager()->findCompressor("zlib");
        if (!_compressor.valid())
        {
           OE_WARN << LC << "Failed to get zlib compressor" << std::endl;
        }
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~MVTFeatureSource()
    {               
        //nop
    }

    //override
    void initialize( const osgDB::Options* dbOptions )
    {
        _dbOptions = dbOptions ? osg::clone(dbOptions) : 0L;
        std::string fullFilename = _options.url()->full();

        int rc = sqlite3_open_v2( fullFilename.c_str(), &_database, SQLITE_OPEN_READONLY, 0L );
        if ( rc != 0 )
        {          
            OE_WARN << LC << "Failed to open database " << sqlite3_errmsg(_database);
        }
    }


    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        const osgEarth::Profile* profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
        FeatureProfile* result = new FeatureProfile(profile->getExtent());
        result->setTiled(true);
        std::string minLevelStr, maxLevelStr;
        if (getMetaData("minzoom", minLevelStr) && getMetaData("maxzoom", maxLevelStr))
        {
            _minLevel = as<int>(minLevelStr, 0);
            _maxLevel = as<int>(maxLevelStr, 0);
            OE_INFO << LC << "Got levels from metadata " << _minLevel << ", " << _maxLevel << std::endl;
        }
        else
        {
            computeLevels();
            OE_INFO << LC << "Got levels from database " << _minLevel << ", " << _maxLevel << std::endl;
        }


        // We use the max level for the first level as well since we don't really support
        // non-additive feature sources yet.  Use the proper min level in the future.
        result->setFirstLevel(_maxLevel);
        result->setMaxLevel(_maxLevel);
        result->setProfile(profile);
        result->geoInterp() = osgEarth::GEOINTERP_RHUMB_LINE;
        return result;
    }

    int zig_zag_decode(int n)
    {
        return (n >> 1) ^ (-(n & 1));
    }

    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
        //OE_NOTICE << "Getting feature cursor" << query.tileKey()->str() << std::endl;

        TileKey key = *query.tileKey();

        int z = key.getLevelOfDetail();
        int tileX = key.getTileX();
        int tileY = key.getTileY();

        GeoPoint ll(key.getProfile()->getSRS(), key.getExtent().xMin(), key.getExtent().yMin(), 0, ALTMODE_ABSOLUTE);
        ll = ll.transform(SpatialReference::create("epsg:4326"));
        //OE_NOTICE << "Requesting key with ll " << ll.x() << ", " << ll.y() << std::endl;


        unsigned int numRows, numCols;
        key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
        tileY  = numRows - tileY - 1;

        //Get the image
        sqlite3_stmt* select = NULL;
        std::string queryStr = "SELECT tile_data from tiles where zoom_level = ? AND tile_column = ? AND tile_row = ?";
        int rc = sqlite3_prepare_v2( _database, queryStr.c_str(), -1, &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << queryStr << "; " << sqlite3_errmsg(_database) << std::endl;
            return NULL;
        }

        bool valid = true;        

        sqlite3_bind_int( select, 1, z );
        sqlite3_bind_int( select, 2, tileX );
        sqlite3_bind_int( select, 3, tileY );

        rc = sqlite3_step( select );

        FeatureList features;

        if ( rc == SQLITE_ROW)
        {                     
            // the pointer returned from _blob gets freed internally by sqlite, supposedly
            const char* data = (const char*)sqlite3_column_blob( select, 0 );
            int dataLen = sqlite3_column_bytes( select, 0 );

            std::string dataBuffer( data, dataLen );

            // decompress if necessary:
            if ( _compressor.valid() )
            {
                std::istringstream inputStream(dataBuffer);
                std::string value;
                if ( !_compressor->decompress(inputStream, value) )
                {
                    OE_WARN << LC << "Decompression failed" << std::endl;
                    valid = false;
                }
                else
                {
                    dataBuffer = value;
                }
            }

            
            
            mapnik::vector::tile tile;

            if (tile.ParseFromString(dataBuffer))
            {
                // Get the layer in question
                for (unsigned int i = 0; i < tile.layers().size(); i++)
                {
                    const mapnik::vector::tile_layer &layer = tile.layers().Get(i);

                    for (unsigned int j = 0; j < layer.features().size(); j++)
                    {
                        const mapnik::vector::tile_feature &feature = layer.features().Get(j);

                        osg::ref_ptr< osgEarth::Symbology::Geometry > geometry; 

                        eGeomType geomType = static_cast<eGeomType>(feature.type());
                        if (geomType == ::Polygon)
                        {
                            //OE_NOTICE << "Polygon " << std::endl;
                            geometry = new osgEarth::Symbology::Polygon();
                        }
                        else if (geomType == ::LineString)
                        {
                            //OE_NOTICE << "LineString" << std::endl;
                            geometry = new osgEarth::Symbology::LineString();
                        }
                        else if (geomType == ::Point)
                        {
                            //OE_NOTICE << "Point" << std::endl;
                            geometry = new osgEarth::Symbology::PointSet();
                        }
                        else
                        {
                            //OE_NOTICE << "uknown" << std::endl;
                            geometry = new osgEarth::Symbology::LineString();
                        }

                        osg::ref_ptr< Feature > oeFeature = new Feature(geometry, key.getProfile()->getSRS());
                        features.push_back(oeFeature.get());

                        unsigned int length = 0;
                        unsigned int g_length = 0;
                        int cmd = -1;
                        const int cmd_bits = 3;

                        unsigned int tileres = layer.extent();

                        int x = 0;
                        int y = 0;

                        for (int k = 0; k < feature.geometry_size();)
                        {
                            if (!length)
                            {
                                unsigned int cmd_length = feature.geometry(k++);
                                cmd = cmd_length & ((1 << cmd_bits) - 1);
                                length = cmd_length >> cmd_bits;
                                g_length = 0;
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
                                    if (cmd == SEG_MOVETO)
                                    {
                                        x += px;
                                        y += py;
                                    }
                                    else if (cmd == SEG_LINETO)
                                    {
                                        x += px;
                                        y += py;

                                        double width = key.getExtent().width();
                                        double height = key.getExtent().height();

                                        double geoX = key.getExtent().xMin() + (width/(double)tileres) * (double)x;
                                        double geoY = key.getExtent().yMax() - (height/(double)tileres) * (double)y;
                                        geometry->push_back(geoX, geoY, 0);
                                    }
                                }
                                else if (cmd == (SEG_CLOSE & ((1 << cmd_bits) - 1)))
                                {
                                    geometry->push_back(geometry->front());
                                }
                            }
                        }

                        geometry->rewind(osgEarth::Symbology::Geometry::ORIENTATION_CCW);

                       
                    }
                }
            }
            else
            {
                OE_DEBUG << "Failed to parse, not surprising" << std::endl;
            }
        }
        else
        {
            //OE_NOTICE << LC << "SQL QUERY failed for " << queryStr << ": " << std::endl;
            valid = false;
        }

        sqlite3_finalize( select );

        if (!features.empty())
        {
            return new FeatureListCursor(features);
        }

        return 0;
    }

    /**
    * Gets the Feature with the given FID
    * @returns
    *     The Feature with the given FID or NULL if not found.
    */
    virtual Feature* getFeature( FeatureID fid )
    {
        return 0;
    }

    virtual bool isWritable() const
    {
        return false;
    }

    virtual const FeatureSchema& getSchema() const
    {
        //TODO:  Populate the schema from the DescribeFeatureType call
        return _schema;
    }

    virtual osgEarth::Symbology::Geometry::Type getGeometryType() const
    {
        return Geometry::TYPE_UNKNOWN;
    }

    bool getMetaData(const std::string& key, std::string& value)
    {
        //get the metadata
        sqlite3_stmt* select = NULL;
        std::string query = "SELECT value from metadata where name = ?";
        int rc = sqlite3_prepare_v2( _database, query.c_str(), -1, &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
            return false;
        }


        bool valid = true;
        std::string keyStr = std::string( key );
        rc = sqlite3_bind_text( select, 1, keyStr.c_str(), keyStr.length(), SQLITE_STATIC );
        if (rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to bind text: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
            return false;
        }

        rc = sqlite3_step( select );
        if ( rc == SQLITE_ROW)
        {                     
            value = (char*)sqlite3_column_text( select, 0 );
        }
        else
        {
            OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
            valid = false;
        }

        sqlite3_finalize( select );
        return valid;
    }

    void computeLevels()
    {        

        osg::Timer_t startTime = osg::Timer::instance()->tick();
        sqlite3_stmt* select = NULL;
        std::string query = "SELECT min(zoom_level), max(zoom_level) from tiles";
        int rc = sqlite3_prepare_v2( _database, query.c_str(), -1, &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
        }

        rc = sqlite3_step( select );
        if ( rc == SQLITE_ROW)
        {                     
            _minLevel = sqlite3_column_int( select, 0 );
            _maxLevel = sqlite3_column_int( select, 1 );
            OE_DEBUG << LC << "Min=" << _minLevel << " Max=" << _maxLevel << std::endl;
        }
        else
        {
            OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
        }        
        sqlite3_finalize( select );        
        osg::Timer_t endTime = osg::Timer::instance()->tick();
        OE_DEBUG << LC << "Computing levels took " << osg::Timer::instance()->delta_s(startTime, endTime ) << " s" << std::endl;
    }



private:
    const MVTFeatureOptions         _options;    
    FeatureSchema                   _schema;
    osg::ref_ptr<osgDB::Options>    _dbOptions;    
    osg::ref_ptr<osgDB::BaseCompressor> _compressor;
    sqlite3* _database;
    unsigned int _minLevel;
    unsigned int _maxLevel;
};


class MVTFeatureSourceFactory : public FeatureSourceDriver
{
public:
    MVTFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_mapnikvectortiles", "Mapnik Vector Tiles feature driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "Mapnik Vector Tiles Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new MVTFeatureSource( getFeatureSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_mapnikvectortiles, MVTFeatureSourceFactory)

