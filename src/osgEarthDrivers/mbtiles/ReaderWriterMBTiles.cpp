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

#include "MBTilesOptions"

#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <sstream>
#include <iomanip>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Drivers;

#include <sqlite3.h>


#define LC "[MBTilesSource] "

class MBTilesSource : public TileSource
{
public:
    MBTilesSource( const TileSourceOptions& options ) :
      TileSource( options ),
      _options( options ),      
      _database( NULL ),
      _minLevel( 0 ),
      _maxLevel( 20 )
    {
    }

    // override
    Status initialize(const osgDB::Options* dbOptions)
    {
        // no caching of source tiles
        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );
        CachePolicy::NO_CACHE.apply( _dbOptions.get() );

                   

        int flags = SQLITE_OPEN_READONLY;
        int rc = sqlite3_open_v2( _options.filename()->c_str(), &_database, flags, 0L );
        if ( rc != 0 )
        {                        
            std::stringstream buf;
            buf << "Failed to open database \"" << *_options.filename() << "\": " << sqlite3_errmsg(_database);
            return Status::Error(buf.str());
        }

        //Print out some metadata
        std::string name, type, version, description, format, profileStr;
        getMetaData( "name", name );
        getMetaData( "type", type);
        getMetaData( "version", version );
        getMetaData( "description", description );
        getMetaData( "format", format );
        getMetaData( "profile", profileStr );
        OE_NOTICE << "name=" << name << std::endl
                  << "type=" << type << std::endl
                  << "version=" << version << std::endl
                  << "description=" << description << std::endl
                  << "format=" << format << std::endl
                  << "profile=" << profileStr << std::endl;



         //Set the profile
        const Profile* profile = getProfile();        
        if (!profile)
        {
            if (!profileStr.empty())
            {
                profile = Profile::create(profileStr);
            }
            else
            {
                profile = osgEarth::Registry::instance()->getSphericalMercatorProfile();
            }
            setProfile( profile );                    
        }
        

        //Determine the tile format and get a reader writer for it.        
        if (_options.format().isSet())
        {
            //Get an explicitly defined format
            _tileFormat = _options.format().value();
        }
        else if (!format.empty())
        {
            //Try to get it from the database metadata
            _tileFormat = format;
        }
        else
        {
            //Assume it's PNG
            _tileFormat = "png";
        }

        OE_DEBUG << LC <<  "_tileFormat = " << _tileFormat << std::endl;

        //Get the ReaderWriter
        _rw = osgDB::Registry::instance()->getReaderWriterForExtension( _tileFormat );                

        computeLevels();

        _emptyImage = ImageUtils::createEmptyImage( 256, 256 );
        
        return STATUS_OK;
    }    

    // override
    osg::Image* createImage( const TileKey& key,
                             ProgressCallback* progress)
    {             
        int z = key.getLevelOfDetail();
        int x = key.getTileX();
        int y = key.getTileY();

        if (z < (int)_minLevel)
        {
            return _emptyImage.get();            
        }

        if (z > (int)_maxLevel)
        {
            //If we're at the max level, just return NULL
            return NULL;
        }

        unsigned int numRows, numCols;
        key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
        y  = numRows - y - 1;

        //Get the image
        sqlite3_stmt* select = NULL;
        std::string query = "SELECT tile_data from tiles where zoom_level = ? AND tile_column = ? AND tile_row = ?";
        int rc = sqlite3_prepare_v2( _database, query.c_str(), -1, &select, 0L );
        if ( rc != SQLITE_OK )
        {
            OE_WARN << LC << "Failed to prepare SQL: " << query << "; " << sqlite3_errmsg(_database) << std::endl;
            return NULL;
        }

        bool valid = true;        
        sqlite3_bind_int( select, 1, z );
        sqlite3_bind_int( select, 2, x );
        sqlite3_bind_int( select, 3, y );


        osg::Image* result = NULL;
        rc = sqlite3_step( select );
        if ( rc == SQLITE_ROW)
        {                     
            // the pointer returned from _blob gets freed internally by sqlite, supposedly
            const char* data = (const char*)sqlite3_column_blob( select, 0 );
            int imageBufLen = sqlite3_column_bytes( select, 0 );

            // deserialize the image from the buffer:
            std::string imageString( data, imageBufLen );
            std::stringstream imageBufStream( imageString );
            osgDB::ReaderWriter::ReadResult rr = _rw->readImage( imageBufStream );
            if (rr.validImage())
            {
                result = rr.takeImage();                
            }            
        }
        else
        {
            OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
            valid = false;
        }

        sqlite3_finalize( select );
        return result;

    }

    bool getMetaData( const std::string& key, std::string& value )
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
            OE_NOTICE << "Min=" << _minLevel << " Max=" << _maxLevel << std::endl;
        }
        else
        {
            OE_DEBUG << LC << "SQL QUERY failed for " << query << ": " << std::endl;
        }

        sqlite3_finalize( select );        
    }

    // override
    virtual std::string getExtension() const 
    {
        return _tileFormat;
    }

private:
    const MBTilesOptions _options;    
    sqlite3* _database;
    unsigned int _minLevel;
    unsigned int _maxLevel;
    osg::ref_ptr< osg::Image> _emptyImage;

    osg::ref_ptr<osgDB::ReaderWriter> _rw;
    osg::ref_ptr<osgDB::Options> _dbOptions;
    std::string _tileFormat;

};


class MBTilesTileSourceFactory : public TileSourceDriver
{
public:
    MBTilesTileSourceFactory()
    {
        supportsExtension( "osgearth_mbtiles", "MBTiles Driver" );
    }

    virtual const char* className()
    {
        return "MBTiles ReaderWriter";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new MBTilesSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_mbtiles, MBTilesTileSourceFactory)


