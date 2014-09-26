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
#include <limits.h>

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/MemCache>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#define LC "[TileSource] "

using namespace osgEarth;

//#undef OE_DEBUG
//#define OE_DEBUG OE_INFO

//------------------------------------------------------------------------

TileBlacklist::TileBlacklist()
{
    //NOP
}

void
TileBlacklist::add(const TileKey& key)
{
    Threading::ScopedWriteLock lock(_mutex);
    _tiles.insert(key);
    OE_DEBUG << "Added " << key.str() << " to blacklist" << std::endl;
}

void
TileBlacklist::remove(const TileKey& key)
{
    Threading::ScopedWriteLock lock(_mutex);
    _tiles.erase(key);
    OE_DEBUG << "Removed " << key.str() << " from blacklist" << std::endl;
}

void
TileBlacklist::clear()
{
    Threading::ScopedWriteLock lock(_mutex);
    _tiles.clear();
    OE_DEBUG << "Cleared blacklist" << std::endl;
}

bool
TileBlacklist::contains(const TileKey& key) const
{
    Threading::ScopedReadLock lock(_mutex);
    return _tiles.find(key) != _tiles.end();
}

unsigned int
TileBlacklist::size() const
{
    Threading::ScopedReadLock lock(_mutex);
    return _tiles.size();
}

TileBlacklist*
TileBlacklist::read(std::istream &in)
{
    osg::ref_ptr< TileBlacklist > result = new TileBlacklist();

    while (!in.eof())
    {
        std::string line;
        std::getline(in, line);
        if (!line.empty())
        {
            int z, x, y;
            if (sscanf(line.c_str(), "%d %d %d", &z, &x, &y) == 3)
            {
                result->add(TileKey(z, x, y, 0L));
            }

        }
    }

    return result.release();
}

TileBlacklist*
TileBlacklist::read(const std::string &filename)
{
    if (osgDB::fileExists(filename) && (osgDB::fileType(filename) == osgDB::REGULAR_FILE))
    {
        std::ifstream in( filename.c_str() );
        return read( in );
    }
    return NULL;
}

void
TileBlacklist::write(const std::string &filename) const
{ 
    std::string path = osgDB::getFilePath(filename);
    if (!path.empty() && !osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        OE_NOTICE << "Couldn't create path " << path << std::endl;
        return;
    }
    std::ofstream out(filename.c_str());
    write(out);
}

void
TileBlacklist::write(std::ostream &output) const
{
    Threading::ScopedReadLock lock(const_cast<TileBlacklist*>(this)->_mutex);
    for (BlacklistedTiles::const_iterator itr = _tiles.begin(); itr != _tiles.end(); ++itr)
    {
        output << itr->getLOD() << " " << itr->getTileX() << " " << itr->getTileY() << std::endl;
    }
}


//------------------------------------------------------------------------


TileSourceOptions::TileSourceOptions( const ConfigOptions& options ) :
DriverConfigOptions   ( options ),
_tileSize             ( 256 ),
_noDataValue          ( (float)SHRT_MIN ),
_minValidValue        ( -32000.0f ),
_maxValidValue        (  32000.0f ),
_L2CacheSize          ( 16 ),
_bilinearReprojection ( true )
{ 
    fromConfig( _conf );
}


Config 
TileSourceOptions::getConfig() const
{
    Config conf = DriverConfigOptions::getConfig();
    conf.updateIfSet( "tile_size", _tileSize );
    conf.updateIfSet( "nodata_value", _noDataValue );
    conf.updateIfSet( "min_valid_value", _minValidValue );
    conf.updateIfSet( "nodata_min", _minValidValue ); // backcompat
    conf.updateIfSet( "max_valid_value", _maxValidValue );
    conf.updateIfSet( "nodata_max", _maxValidValue ); // backcompat
    conf.updateIfSet( "blacklist_filename", _blacklistFilename);
    conf.updateIfSet( "l2_cache_size", _L2CacheSize );
    conf.updateIfSet( "bilinear_reprojection", _bilinearReprojection );
    conf.updateIfSet( "max_data_level", _maxDataLevel );
    conf.updateObjIfSet( "profile", _profileOptions );
    return conf;
}


void
TileSourceOptions::mergeConfig( const Config& conf )
{
    DriverConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}


void
TileSourceOptions::fromConfig( const Config& conf )
{
    conf.getIfSet( "tile_size", _tileSize );
    conf.getIfSet( "nodata_value", _noDataValue );
    conf.getIfSet( "nodata_min", _minValidValue );
    conf.getIfSet( "nodata_max", _maxValidValue );
    conf.getIfSet( "blacklist_filename", _blacklistFilename);
    conf.getIfSet( "l2_cache_size", _L2CacheSize );
    conf.getIfSet( "bilinear_reprojection", _bilinearReprojection );
    conf.getIfSet( "max_data_level", _maxDataLevel );
    conf.getObjIfSet( "profile", _profileOptions );
}


//------------------------------------------------------------------------

// statics
TileSource::Status TileSource::STATUS_OK = TileSource::Status();

const char* TileSource::INTERFACE_NAME = "osgEarth::TileSource";

const TileSource::Mode TileSource::MODE_READ   = 0x01;
const TileSource::Mode TileSource::MODE_WRITE  = 0x02;
const TileSource::Mode TileSource::MODE_CREATE = 0x04;


TileSource::TileSource(const TileSourceOptions& options) :
_options( options ),
_status ( Status::Error("Not initialized") ),
_mode   ( 0 )
{
    this->setThreadSafeRefUnref( true );

    int l2CacheSize = 0;
    char const* l2env = ::getenv( "OSGEARTH_L2_CACHE_SIZE" );
    if ( l2env )
    {
        l2CacheSize = as<int>( std::string(l2env), 0 );
    }
    else if ( *options.L2CacheSize() > 0 )
    {
        _memCache = new MemCache( *options.L2CacheSize() );
    }

    if (_options.blacklistFilename().isSet())
    {
        _blacklistFilename = _options.blacklistFilename().value();
    }

    
    if (!_blacklistFilename.empty() && osgDB::fileExists(_blacklistFilename))
    {
        _blacklist = TileBlacklist::read(_blacklistFilename);
        if (_blacklist.valid())
        {
            OE_INFO << "Read blacklist from file" << _blacklistFilename << std::endl;
        }
    }

    if (!_blacklist.valid())
    {
        //Initialize the blacklist if we couldn't read it.
        _blacklist = new TileBlacklist();
    }
}

TileSource::~TileSource()
{
    if (_blacklist.valid() && !_blacklistFilename.empty())
    {
        _blacklist->write(_blacklistFilename);
    }
}

void
TileSource::setStatus( TileSource::Status status )
{
    _status = status;
}

TileSource::Status
TileSource::initialize(const osgDB::Options* options)
{
    // default implementation. Subclasses should override this.
    return STATUS_OK;
}

const TileSource::Status&
TileSource::open(const Mode&           openMode,
                 const osgDB::Options* options)
{
    _mode = openMode;

    // Initialize the underlying data store
    Status status = initialize(options);

    // Check the return status. The TileSource MUST have a valid
    // Profile after initialization.
    if ( status == STATUS_OK )
    {
        if ( getProfile() != 0L )
        {
            _status = status;
        }
        else 
        {
            _status = Status::Error("No profile available");
        }
    }
    else
    {
        _status = status;
    }

    if ( _status.isError() )
    {
        OE_WARN << LC << "Open failed: " << _status.message() << std::endl;
    }

    return _status;
}

int
TileSource::getPixelsPerTile() const
{
    return _options.tileSize().value();
}

osg::Image*
TileSource::createImage(const TileKey&        key,
                        ImageOperation*       prepOp, 
                        ProgressCallback*     progress )
{
    if ( _status != STATUS_OK )
        return 0L;

    // Try to get it from the memcache fist
    if (_memCache.valid())
    {
        ReadResult r = _memCache->getOrCreateDefaultBin()->readImage( key.str() );
        if ( r.succeeded() )
            return r.releaseImage();
    }

    osg::ref_ptr<osg::Image> newImage = createImage(key, progress);

    if ( prepOp )
        (*prepOp)( newImage );

    if ( newImage.valid() && _memCache.valid() )
    {
        // cache it to the memory cache.
        _memCache->getOrCreateDefaultBin()->write( key.str(), newImage.get() );
    }

    return newImage.release();
}

osg::HeightField*
TileSource::createHeightField(const TileKey&        key,
                              HeightFieldOperation* prepOp, 
                              ProgressCallback*     progress )
{
    if ( _status != STATUS_OK )
        return 0L;

    // Try to get it from the memcache first:
    if (_memCache.valid())
    {
        ReadResult r = _memCache->getOrCreateDefaultBin()->readObject( key.str() );
        if ( r.succeeded() )
            return r.release<osg::HeightField>();
    }

    osg::ref_ptr<osg::HeightField> newHF = createHeightField( key, progress );

    if ( prepOp )
        (*prepOp)( newHF );

    if ( newHF.valid() && _memCache.valid() )
    {
        _memCache->getOrCreateDefaultBin()->write( key.str(), newHF.get() );
    }

    //TODO: why not just newHF.release()? -gw
    return newHF.valid() ? new osg::HeightField( *newHF.get() ) : 0L;
}

osg::Image*
TileSource::createImage(const TileKey&    key,
                        ProgressCallback* progress)
{
    return 0L;
}

osg::HeightField*
TileSource::createHeightField(const TileKey&        key,
                              ProgressCallback*     progress)
{
    if ( _status != STATUS_OK )
        return 0L;

    osg::ref_ptr<osg::Image> image = createImage(key, progress);
    osg::HeightField* hf = 0;
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        hf = conv.convert( image.get() );
    }      
    return hf;
}

bool
TileSource::storeHeightField(const TileKey&     key,
                             osg::HeightField*  hf,
                              ProgressCallback* progress)
{
    if ( _status != STATUS_OK || hf == 0L )
        return 0L;

    ImageToHeightFieldConverter conv;
    osg::ref_ptr<osg::Image> image = conv.convert(hf, 32);
    if (image.valid())
    {
        return storeImage(key, image.get(), progress);
    }
    return false;
}

bool
TileSource::isOK() const 
{
    return _status == STATUS_OK;
}

void
TileSource::setProfile( const Profile* profile )
{
    _profile = profile;
}

const Profile*
TileSource::getProfile() const
{
    return _profile.get();
}

bool
TileSource::hasDataAtLOD( unsigned lod ) const
{
    // the sematics here are really "MIGHT have data at LOD".

    // Explicit max data level?
    if ( _options.maxDataLevel().isSet() && lod > _options.maxDataLevel().value() )
        return false;

    // If no data extents are provided, just return true
    if ( _dataExtents.size() == 0 )
        return true;

    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if ((!itr->minLevel().isSet() || itr->minLevel() <= lod) &&
            (!itr->maxLevel().isSet() || itr->maxLevel() >= lod))
        {
            return true;
        }
    }

    return false;
}


bool
TileSource::hasDataInExtent( const GeoExtent& extent ) const
{
    // if the extent is invalid, no intersection.
    if ( !extent.isValid() )
        return false;

    // If no data extents are provided, just return true
    if ( _dataExtents.size() == 0 )
        return true;

    bool intersects = false;

    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if ( extent.intersects( *itr ) )
        {
            intersects = true;
            break;
        }
    }
    return intersects;
}


bool
TileSource::hasData(const osgEarth::TileKey& key) const
{
    //sematics: might have data.

    //If no data extents are provided, just return true
    if (_dataExtents.size() == 0) 
        return true;

    unsigned int lod = key.getLevelOfDetail();

    // Remap the lod to an appropriate lod if it's not in the same SRS        
    if (!key.getProfile()->isHorizEquivalentTo( getProfile() ) )
    {        
        lod = getProfile()->getEquivalentLOD( key.getProfile(), key.getLevelOfDetail() );        
    }

    // Check the explicit max data override:
    if (_options.maxDataLevel().isSet() && lod > _options.maxDataLevel().value())
        return false;


    bool intersectsData = false;
    const osgEarth::GeoExtent& keyExtent = key.getExtent();
    
    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if ((keyExtent.intersects( *itr )) && 
            (!itr->minLevel().isSet() || itr->minLevel() <= lod ) &&
            (!itr->maxLevel().isSet() || itr->maxLevel() >= lod ))
        {
            intersectsData = true;
            break;
        }
    }

    return intersectsData;
}

bool
TileSource::hasDataForFallback(const osgEarth::TileKey& key) const
{
    //sematics: might have data.

    //If no data extents are provided, just return true
    if (_dataExtents.size() == 0) 
        return true;

    const osgEarth::GeoExtent& keyExtent = key.getExtent();
    bool intersectsData = false;

    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if ((keyExtent.intersects( *itr )) && 
            (!itr->minLevel().isSet() || itr->minLevel() <= key.getLOD()))
        {
            intersectsData = true;
            break;
        }
    }

    return intersectsData;
}

TileBlacklist*
TileSource::getBlacklist()
{
    return _blacklist.get();
}

const TileBlacklist*
TileSource::getBlacklist() const
{
    return _blacklist.get();
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[TileSourceFactory] "
#define TILESOURCE_OPTIONS_TAG   "__osgEarth::TileSourceOptions"
#define TILESOURCE_INTERFACE_TAG "__osgEarth::Interface"

TileSource*
TileSourceFactory::create(const TileSourceOptions& options)
{
    TileSource* result = 0L;

    std::string driver = options.getDriver();
    if ( driver.empty() )
    {
        OE_WARN << LC << "ILLEGAL- no driver set for tile source" << std::endl;
        return 0L;
    }

    osg::ref_ptr<osgDB::Options> dbopt = Registry::instance()->cloneOrCreateOptions();
    dbopt->setPluginData      ( TILESOURCE_OPTIONS_TAG,   (void*)&options );
    dbopt->setPluginStringData( TILESOURCE_INTERFACE_TAG, TileSource::INTERFACE_NAME );

    std::string driverExt = std::string( ".osgearth_" ) + driver;
    result = dynamic_cast<TileSource*>( osgDB::readObjectFile( driverExt, dbopt.get() ) );
    if ( !result )
    {
        OE_WARN << LC << "Failed to load TileSource driver \"" << driver << "\"" << std::endl;
    }

    // apply an Override Profile if provided.
    if ( result && options.profile().isSet() )
    {
        const Profile* profile = Profile::create(*options.profile());
        if ( profile )
        {
            result->setProfile( profile );
        }
    }

    return result;
}

#if 0
ReadWriteTileSource*
TileSourceFactory::openReadWrite(const TileSourceOptions& options)
{
    ReadWriteTileSource* result = 0L;

    std::string driver = options.getDriver();
    if ( driver.empty() )
    {
        OE_WARN << LC << "ILLEGAL- no driver set for tile source" << std::endl;
        return 0L;
    }

    osg::ref_ptr<osgDB::Options> dbopt = Registry::instance()->cloneOrCreateOptions();
    dbopt->setPluginData      ( TILESOURCEOPTIONS_TAG,   (void*)&options );
    dbopt->setPluginStringData( TILESOURCEINTERFACE_TAG, ReadWriteTileSource::INTERFACE_NAME );

    std::string driverExt = std::string( ".osgearth_" ) + driver;
    result = dynamic_cast<ReadWriteTileSource*>( osgDB::readObjectFile( driverExt, dbopt.get() ) );
    if ( !result )
    {
        OE_WARN << LC << "Failed to load ReadWriteTileSource driver \"" << driver << "\"" << std::endl;
    }

    // apply an Override Profile if provided.
    if ( result && options.profile().isSet() )
    {
        const Profile* profile = Profile::create(*options.profile());
        if ( profile )
        {
            result->setProfile( profile );
        }
    }

    return result;
}
#endif

//------------------------------------------------------------------------

const TileSourceOptions&
TileSourceDriver::getTileSourceOptions(const osgDB::Options* dbopt ) const
{
    return *static_cast<const TileSourceOptions*>( dbopt->getPluginData( TILESOURCE_OPTIONS_TAG ) );
}

const std::string
TileSourceDriver::getInterfaceName(const osgDB::Options* dbopt) const
{
    return dbopt->getPluginStringData(TILESOURCE_INTERFACE_TAG);
}
