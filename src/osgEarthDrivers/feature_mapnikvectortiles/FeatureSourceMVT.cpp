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
#include <osgEarth/FileUtils>
#include <osgEarth/GeoData>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/MVT>
#include <osgEarthFeatures/Filter>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>


#define LC "[MVT FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;


class MVTFeatureSource : public FeatureSource
{
public:
    MVTFeatureSource(const MVTFeatureOptions& options ) :
      FeatureSource( options ),
      _options     ( options ),
      _minLevel(0),
      _maxLevel(14),
      _database(0L)
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

    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
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
            std::stringstream in(dataBuffer);
            MVT::read(in, key, features);
        }
        else
        {
            OE_DEBUG << LC << "SQL QUERY failed for " << queryStr << ": " << std::endl;
            valid = false;
        }

        sqlite3_finalize( select );

        // apply filters before returning.
        applyFilters( features, query.tileKey()->getExtent() );

        // If we have any features and we have an fid attribute, override the fid of the features
        if (_options.fidAttribute().isSet())
        {
            for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
            {
                std::string attr = itr->get()->getString(_options.fidAttribute().get());                
                FeatureID fid = as<long>(attr, 0);
                itr->get()->setFID( fid );
            }
        }

        if (!features.empty())
        {
            //OE_NOTICE << "Returning " << features.size() << " features" << std::endl;
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

protected:
    //override
    Status initialize(const osgDB::Options* readOptions)
    {
        _dbOptions = Registry::cloneOrCreateOptions(readOptions);
        std::string fullFilename = _options.url()->full();

        int rc = sqlite3_open_v2( fullFilename.c_str(), &_database, SQLITE_OPEN_READONLY, 0L );
        if ( rc != 0 )
        {          
            return Status::Error(Status::ResourceUnavailable, Stringify() << "Failed to open database, " << sqlite3_errmsg(_database));
        }

        setFeatureProfile(createFeatureProfile());

        return Status::OK();
    }

private:
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
            OE_NOTICE << LC << "Got levels from metadata " << _minLevel << ", " << _maxLevel << std::endl;
        }
        else
        {            
            computeLevels();
            OE_NOTICE << LC << "Got levels from database " << _minLevel << ", " << _maxLevel << std::endl;
        }


        result->setFirstLevel(_minLevel);
        result->setMaxLevel(_maxLevel);
        result->setProfile(profile);
        result->geoInterp() = osgEarth::GEOINTERP_GREAT_CIRCLE;
        return result;
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

    virtual const char* className() const
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

