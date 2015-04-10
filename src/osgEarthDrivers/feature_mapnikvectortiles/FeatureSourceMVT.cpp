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
        if (_compressor.valid())
        {
            OE_NOTICE << "Got compressor" << std::endl;
        }
        else
        {
            OE_NOTICE << "No zlib for you" << std::endl;
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
            OE_WARN << "Failed to open database " << sqlite3_errmsg(_database);
        }
        else
        {
            OE_NOTICE << "Opened database " << fullFilename << std::endl;
        }
    }


    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        const osgEarth::Profile* profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
        FeatureProfile* result = new FeatureProfile(profile->getExtent());
        result->setTiled(true);
        result->setFirstLevel(13);
        result->setMaxLevel(13);
        result->setProfile(profile);
        result->geoInterp() = osgEarth::GeoInterpolation::GEOINTERP_RHUMB_LINE;
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
        OE_NOTICE << "Requesting key with ll " << ll.x() << ", " << ll.y() << std::endl;


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

            
            FeatureList features;
            mapnik::vector::tile tile;

            if (tile.ParseFromString(dataBuffer))
            {
                OE_NOTICE << "Wow it parsed!" << std::endl;
                OE_NOTICE << "It has " << tile.layers().size() << " layers" << std::endl;
                // Get the layer in question
                for (unsigned int i = 0; i < tile.layers().size(); i++)
                {
                    const mapnik::vector::tile_layer &layer = tile.layers().Get(i);

                    OE_NOTICE << "Got layer " << layer.name() << " which has " << layer.features().size() << " and extent=" <<  layer.extent() << std::endl;


                    for (unsigned int j = 0; j < layer.features().size(); j++)
                    {
                        const mapnik::vector::tile_feature &feature = layer.features().Get(j);

                        

                        osg::ref_ptr< osgEarth::Symbology::Geometry > geometry; 

                        eGeomType geomType = static_cast<eGeomType>(feature.type());
                        if (geomType == eGeomType::Polygon)
                        {
                            //OE_NOTICE << "Polygon " << std::endl;
                            geometry = new osgEarth::Symbology::Polygon();
                        }
                        else if (geomType == eGeomType::LineString)
                        {
                            //OE_NOTICE << "LineString" << std::endl;
                            geometry = new osgEarth::Symbology::LineString();
                        }
                        else if (geomType == eGeomType::Point)
                        {
                            //OE_NOTICE << "Point" << std::endl;
                            geometry = new osgEarth::Symbology::PointSet();
                        }
                        else
                        {
                            //OE_NOTICE << "uknown" << std::endl;
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

                        int min_x = INT_MAX;
                        int min_y = INT_MAX;
                        int max_x = -INT_MAX;
                        int max_y = -INT_MAX;


                        for (int k = 0; k < feature.geometry_size();)
                        {
                            if (!length)
                            {
                                unsigned int cmd_length = feature.geometry(k++);
                                //OE_NOTICE << "cmd_length=" << cmd_length << std::endl;
                                cmd = cmd_length & ((1 << cmd_bits) - 1);
                                length = cmd_length >> cmd_bits;
                                //OE_NOTICE << "length=" << length << std::endl;
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
                                    //OSG_NOTICE << "cmd " << cmd << ": " << px << ", " << py << std::endl;
                                    if (cmd == SEG_MOVETO)
                                    {
                                        x += px;
                                        y += py;

                                        if (x < min_x) min_x = x;
                                        if (y < min_y) min_y = y;
                                        if (x > max_x) max_x = x;
                                        if (y > max_y) max_y = y;
                                    }
                                    else if (cmd == SEG_LINETO)
                                    {
                                        double width = key.getExtent().width();
                                        double height = key.getExtent().height();

                                        double geoX = key.getExtent().xMin() + (width/(double)tileres) * (double)x;
                                        double geoY = key.getExtent().yMax() - (height/(double)tileres) * (double)y;
                                        //double geoX = key.getExtent().xMin() + (double)x;
                                        //double geoY = key.getExtent().yMax() - (double)y;
                                        //geoY *= -1.0;
                                        //geometry->push_back(geoX, geoY, 0);

                                        /*
                                        if (!key.getExtent().contains(geoX, geoY))
                                        {
                                            OE_NOTICE << "You're dumb" << std::endl;
                                            OE_NOTICE << "Extent " << key.getExtent().toString() << std::endl;
                                            OE_NOTICE << "Point " << geoX << ", " << geoY << std::endl;
                                        }
                                        */

                                        GeoPoint pt(key.getProfile()->getSRS(), geoX, geoY, 0, ALTMODE_ABSOLUTE);
                                        pt = pt.transform(SpatialReference::create("epsg:4326"));
                                        //OE_NOTICE << "Geo " << pt.x() << ", " << pt.y() << std::endl;

                                        x += px;
                                        y += py;

                                        if (x < min_x) min_x = x;
                                        if (y < min_y) min_y = y;
                                        if (x > max_x) max_x = x;
                                        if (y > max_y) max_y = y;

                                        geoX = key.getExtent().xMin() + (width/(double)tileres) * (double)x;
                                        geoY = key.getExtent().yMax() - (height/(double)tileres) * (double)y;
                                        //geoX = key.getExtent().xMin() + (double)x;
                                        //geoY = key.getExtent().yMax() - (double)y;
                                        //geoY *= -1.0;
                                        geometry->push_back(geoX, geoY, 0);
                                        /*
                                        if (!key.getExtent().contains(geoX, geoY))
                                        {
                                            OE_NOTICE << "You're dumb" << std::endl;
                                            OE_NOTICE << "Extent " << key.getExtent().toString() << std::endl;
                                            OE_NOTICE << "Point " << geoX << ", " << geoY << std::endl;
                                        }
                                        */
                                    }
                                    //OSG_NOTICE << "" << px << ", " << py << std::endl;
                                }
                                else if (cmd == (SEG_CLOSE & ((1 << cmd_bits) - 1)))
                                {
                                    geometry->push_back(geometry->front());
                                    //OE_NOTICE << "Seg close" << std::endl;
                                }
                            }
                        }

                        geometry->rewind(osgEarth::Symbology::Geometry::ORIENTATION_CCW);

                       
                    }
                }

                OE_NOTICE << "Created " << features.size() << " features" << std::endl;
                return new FeatureListCursor(features);
            }
            else
            {
                OE_DEBUG << "Failed to parse, not surprising" << std::endl;
            }

            
            /*

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
           
            // decode the raw image data:
            if ( valid )
            {
                std::istringstream inputStream(dataBuffer);
                osgDB::ReaderWriter::ReadResult rr = _rw->readImage( inputStream );
                if (rr.validImage())
                {
                    result = rr.takeImage();                
                }
            }
            */
        }
        else
        {
            //OE_NOTICE << LC << "SQL QUERY failed for " << queryStr << ": " << std::endl;
            valid = false;
        }

        sqlite3_finalize( select );

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



private:
    const MVTFeatureOptions         _options;    
    FeatureSchema                   _schema;
    osg::ref_ptr<osgDB::Options>    _dbOptions;    
    osg::ref_ptr<osgDB::BaseCompressor> _compressor;
    sqlite3* _database;
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

