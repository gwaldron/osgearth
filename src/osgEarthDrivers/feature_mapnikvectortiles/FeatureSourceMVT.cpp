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

#define LC "[MVT FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

class MVTFeatureSource : public FeatureSource
{
public:
    MVTFeatureSource(const MVTFeatureOptions& options ) :
      FeatureSource( options ),
      _options     ( options )
    {                
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

        int rc = sqlite3_open_v2( fullFilename.c_str(), &_database, flags, 0L );
        if ( rc != 0 )
        {          
            OE_WARN << "Failed to open database " << 
            return Status::Error( Stringify()
                << "Database \"" << fullFilename << "\": " << sqlite3_errmsg(_database) );
        }
        /*(
        _dbOptions = dbOptions ? osg::clone(dbOptions) : 0L;
        if ( _dbOptions.valid() )
        {
            // Set up a Custom caching bin for this source:
            Cache* cache = Cache::get( _dbOptions.get() );
            if ( cache )
            {
                Config optionsConf = _options.getConfig();

                std::string binId = Stringify() << std::hex << hashString(optionsConf.toJSON()) << "_tfs";
                _cacheBin = cache->addBin( binId );
                if ( _cacheBin.valid() )
                {                
                    // write a metadata record just for reference purposes.. we don't actually use it
                    Config metadata = _cacheBin->readMetadata();
                    if ( metadata.empty() )
                    {
                        _cacheBin->writeMetadata( optionsConf );
                    }

                    if ( _cacheBin.valid() )
                    {
                        _cacheBin->apply( _dbOptions.get() );
                    }
                }
                else
                {
                    OE_INFO << LC << "Failed to open cache bin \"" << binId << "\"\n";
                }
            }
        }     
        _layerValid = TFSReaderWriter::read(_options.url().get(), _dbOptions.get(), _layer);
        if (_layerValid)
        {
            OE_INFO << LC <<  "Read layer TFS " << _layer.getTitle() << " " << _layer.getAbstract() << " " << _layer.getFirstLevel() << " " << _layer.getMaxLevel() << " " << _layer.getExtent().toString() << std::endl;
        }
        */
    }


    /** Called once at startup to create the profile for this feature set. Successful profile
        creation implies that the datasource opened succesfully. */
    const FeatureProfile* createFeatureProfile()
    {
        /*
        FeatureProfile* result = NULL;
        if (_layerValid)
        {
            result = new FeatureProfile(_layer.getExtent());
            result->setTiled( true );
            result->setFirstLevel( _layer.getFirstLevel());
            result->setMaxLevel( _layer.getMaxLevel());
            result->setProfile( osgEarth::Profile::create(_layer.getSRS(), _layer.getExtent().xMin(), _layer.getExtent().yMin(), _layer.getExtent().xMax(), _layer.getExtent().yMax(), 1, 1) );
            if ( _options.geoInterp().isSet() )
                result->geoInterp() = _options.geoInterp().get();
        }
        return result;        
        */
        return 0;
    }

    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
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

