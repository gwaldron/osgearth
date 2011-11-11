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
#include <osgEarth/TerrainLayer>
#include <osgEarth/TileSource>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/URI>
#include <osgDB/WriteFile>
#include <osg/Version>
#include <OpenThreads/ScopedLock>
#include <memory.h>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TerrainLayer] "

//------------------------------------------------------------------------

TerrainLayerOptions::TerrainLayerOptions( const ConfigOptions& options ) :
ConfigOptions       ( options ),
_minLevel           ( 0 ),
_maxLevel           ( 99 ),
_cachePolicy        ( CachePolicy::USAGE_DEFAULT ),
_loadingWeight      ( 1.0f ),
_exactCropping      ( false ),
_enabled            ( true ),
_reprojectedTileSize( 256 )

{
    setDefaults();
    fromConfig( _conf ); 
}

TerrainLayerOptions::TerrainLayerOptions( const std::string& name, const TileSourceOptions& driverOptions ) :
ConfigOptions()
{
    setDefaults();
    fromConfig( _conf );
    _name = name;
    _driver = driverOptions;
}

void
TerrainLayerOptions::setDefaults()
{
    _enabled.init( true );
    _exactCropping.init( false );
    _reprojectedTileSize.init( 256 );
    _cachePolicy.init( CachePolicy() );
    _loadingWeight.init( 1.0f );
    _minLevel.init( 0 );
    _maxLevel.init( 99 );
}

Config
TerrainLayerOptions::getConfig( bool isolate ) const
{
    Config conf = isolate ? ConfigOptions::newConfig() : ConfigOptions::getConfig();

    conf.set("name", _name);
    conf.updateIfSet( "min_level", _minLevel );
    conf.updateIfSet( "max_level", _maxLevel );
    conf.updateIfSet( "min_level_resolution", _minLevelResolution );
    conf.updateIfSet( "max_level_resolution", _maxLevelResolution );
    conf.updateIfSet( "loading_weight", _loadingWeight );
    conf.updateIfSet( "enabled", _enabled );
    conf.updateIfSet( "edge_buffer_ratio", _edgeBufferRatio);
    conf.updateObjIfSet( "profile", _profile );
    conf.updateIfSet( "max_data_level", _maxDataLevel);
    conf.updateIfSet( "reprojected_tilesize", _reprojectedTileSize);

    conf.updateIfSet   ( "cacheid",      _cacheId );
    conf.updateIfSet   ( "cache_format", _cacheFormat );
    conf.updateObjIfSet( "cache_policy", _cachePolicy );

    // Merge the TileSource options
    if ( !isolate && driver().isSet() )
        conf.merge( driver()->getConfig() );

    return conf;
}

void
TerrainLayerOptions::fromConfig( const Config& conf )
{
    _name = conf.value("name");
    conf.getIfSet( "min_level", _minLevel );
    conf.getIfSet( "max_level", _maxLevel );        
    conf.getIfSet( "min_level_resolution", _minLevelResolution );
    conf.getIfSet( "max_level_resolution", _maxLevelResolution );
    conf.getIfSet( "loading_weight", _loadingWeight );
    conf.getIfSet( "enabled", _enabled );
    conf.getIfSet( "edge_buffer_ratio", _edgeBufferRatio);
    conf.getObjIfSet( "profile", _profile );
    conf.getIfSet( "max_data_level", _maxDataLevel);
    conf.getIfSet( "reprojected_tilesize", _reprojectedTileSize);

    conf.getIfSet   ( "cacheid",      _cacheId );
    conf.getIfSet   ( "cache_format", _cacheFormat );
    conf.getObjIfSet( "cache_policy", _cachePolicy );

    // legacy support:
    if ( conf.value<bool>( "cache_only", false ) == true )
        _cachePolicy->usage() = CachePolicy::USAGE_CACHE_ONLY;
    if ( conf.value<bool>( "cache_enabled", true ) == false )
        _cachePolicy->usage() = CachePolicy::USAGE_NO_CACHE;

    if ( conf.hasValue("driver") )
        driver() = TileSourceOptions(conf);
}

void
TerrainLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

TerrainLayer::TerrainLayer(const TerrainLayerOptions& initOptions,
                           TerrainLayerOptions*       runtimeOptions ) :
_initOptions   ( initOptions ),
_runtimeOptions( runtimeOptions )
{
    init();
}

TerrainLayer::TerrainLayer(const TerrainLayerOptions& initOptions,
                           TerrainLayerOptions*       runtimeOptions,
                           TileSource*                tileSource ) :
_initOptions   ( initOptions ),
_runtimeOptions( runtimeOptions ),
_tileSource    ( tileSource )
{
    init();
}

TerrainLayer::~TerrainLayer()
{
    if ( _cache.valid() )
    {
        Threading::ScopedWriteLock exclusive( _cacheBinsMutex );
        for( CacheBinInfoMap::iterator i = _cacheBins.begin(); i != _cacheBins.end(); ++i )
        {
            CacheBinInfo& info = i->second;
            if ( info._bin.valid() )
            {
                _cache->removeBin( info._bin.get() );
            }
        }
    }
}

void
TerrainLayer::init()
{
    _tileSourceInitialized = false;
    _tileSize              = 256;
}

void
TerrainLayer::overrideCachePolicy( const CachePolicy& policy )
{
    _runtimeOptions->cachePolicy() = policy;
}

void
TerrainLayer::setCache( Cache* cache )
{
    if (_cache.get() != cache && 
        _runtimeOptions->cachePolicy()->usage() != CachePolicy::USAGE_NO_CACHE )
    {
        _cache = cache;

        // Initialize a cache bin for this layer.
        if ( _cache.valid() )
        {
            // create the unique cache ID for the cache bin.
            std::string cacheId;

            if ( _runtimeOptions->cacheId().isSet() && !_runtimeOptions->cacheId()->empty() )
            {
                // user expliticy set a cacheId in the terrain layer options.
                cacheId = *_runtimeOptions->cacheId();
            }
            else
            {
                // system will generate a cacheId.
                // technically, this is not quite right, we need to remove everything that's
                // an image layer property and just use the tilesource properties.
                Config layerConf = _runtimeOptions->getConfig( true );
                Config driverConf = _runtimeOptions->driver()->getConfig();
                Config hashConf = driverConf - layerConf;

                OE_DEBUG << LC << "Hash JSON for layer " << getName() << " is: " << hashConf.toJSON(false) << std::endl;

                // remove cache-control properties before hashing.
                hashConf.remove( "cache_only" );
                hashConf.remove( "cache_enabled" );
                hashConf.remove( "cache_policy" );
                hashConf.remove( "cacheid" );

                cacheId = Stringify() << std::hex << osgEarth::hashString(hashConf.toJSON());
            }

            _runtimeOptions->cacheId() = cacheId;
        }
    }

    if ( !_cache.valid() )
    {
        _cache = 0L; 
        _runtimeOptions->cachePolicy()->usage() = CachePolicy::USAGE_NO_CACHE;
    }
}

void
TerrainLayer::setTargetProfileHint( const Profile* profile )
{
    _targetProfileHint = profile;
}

TileSource* 
TerrainLayer::getTileSource() const
{
    if ((_tileSource.valid() && !_tileSourceInitialized) ||
        (!_tileSource.valid() && !isCacheOnly()))
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock(const_cast<TerrainLayer*>(this)->_initTileSourceMutex );
        // double-check pattern
        if ((_tileSource.valid() && !_tileSourceInitialized) ||
            (!_tileSource.valid() && !isCacheOnly()))
        {
            const_cast<TerrainLayer*>(this)->initTileSource();
        }
    }

    return _tileSource.get();
}

const Profile*
TerrainLayer::getProfile() const
{
    // NB: in cache-only mode, there IS NO layer profile.
    if ( !_profile.valid() && !isCacheOnly() )
    {
        if ( !_tileSourceInitialized )
        {
            // Call getTileSource to make sure the TileSource is initialized
            getTileSource();
        }

        if ( _tileSource.valid() && !_profile.valid() )
        {
            const_cast<TerrainLayer*>(this)->_profile = _tileSource->getProfile();
        }
    }
    
    return _profile.get();
}

unsigned int
TerrainLayer::getMaxDataLevel() const
{
    //Try the setting first

    if ( _runtimeOptions->maxDataLevel().isSet() )
    {
        return _runtimeOptions->maxDataLevel().get();
    }

    //Try the TileSource
	TileSource* ts = getTileSource();
	if ( ts )
	{
		return ts->getMaxDataLevel();
	}

    //Just default
	return 20;
}

unsigned
TerrainLayer::getTileSize() const
{
    TileSource* ts = getTileSource();
    return ts ? ts->getPixelsPerTile() : _tileSize;
}

bool
TerrainLayer::isDynamic() const
{
    TileSource* ts = getTileSource();
    return ts ? ts->isDynamic() : false;
}

CacheBin*
TerrainLayer::getCacheBin( const Profile* profile )
{
    if (_runtimeOptions->cachePolicy().isSet() &&
        _runtimeOptions->cachePolicy()->usage() == CachePolicy::USAGE_NO_CACHE )
    {
        return 0L;
    }

    // the cache bin ID is the cache IF concatenated with the profile signature.
    std::string binId = *_runtimeOptions->cacheId() + "_" + profile->getSignature();

    {
        Threading::ScopedReadLock shared(_cacheBinsMutex);
        CacheBinInfoMap::iterator i = _cacheBins.find( binId );
        if ( i != _cacheBins.end() )
            return i->second._bin.get();
    }

    {
        Threading::ScopedWriteLock exclusive(_cacheBinsMutex);

        // double-check:
        CacheBinInfoMap::iterator i = _cacheBins.find( binId );
        if ( i != _cacheBins.end() )
            return i->second._bin.get();

        // add the new bin:
        osg::ref_ptr<CacheBin> newBin = _cache->addBin( binId );

        // and configure:
        if ( newBin.valid() )
        {
            // attempt to read the cache metadata:
            CacheBinMetadata meta( newBin->readMetadata() );

            if ( !meta._empty ) // cache exists
            {
                // verify that the cache if compatible with the tile source:
                if ( getTileSource() && getProfile() )
                {
                    //todo: check the profile too
                    if ( *meta._sourceDriver != getTileSource()->getOptions().getDriver() )
                    {
                        OE_WARN << LC << "Cache has an incompatible driver or profile... disabling"
                            << std::endl;
                        _runtimeOptions->cachePolicy()->usage() = CachePolicy::USAGE_NO_CACHE;
                        return 0L;
                    }
                }   

                else if ( isCacheOnly() && !_profile.valid() )
                {
                    // in cacheonly mode, create a profile from the first cache bin accessed
                    // (they SHOULD all be the same...)
                    _profile = Profile::create( *meta._sourceProfile );

                    // copy the max data level from the cache
                    _runtimeOptions->maxDataLevel() = *meta._maxDataLevel;
                }
            }

            else
            {
                // cache does not exist, so try to create it. A valid TileSource is necessary
                // for this.
                if ( getTileSource() && getProfile() )
                {
                    // no existing metadata; create some.
                    meta._cacheBinId    = binId;
                    meta._sourceName    = this->getName();
                    meta._maxDataLevel  = getMaxDataLevel();
                    meta._sourceDriver  = getTileSource()->getOptions().getDriver();
                    meta._sourceProfile = getProfile()->toProfileOptions();
                    meta._cacheProfile  = profile->toProfileOptions();

                    // store it in the cache bin.
                    newBin->writeMetadata( meta.getConfig() );
                }
                else if ( isCacheOnly() )
                {
                    OE_WARN << LC << "Failed to create a cache bin for layer \"" << getName()
                        << "\" because cache_only mode is enabled and no existing cache could be found."
                        << std::endl;
                    return 0L;
                }
                else
                {
                    OE_WARN << LC << "Failed to create a cache bin for layer \"" << getName()
                        << "\" because there is no valid tile source."
                        << std::endl;
                    return 0L;
                }
            }


            // store the bin.
            CacheBinInfo& newInfo = _cacheBins[binId];
            newInfo._metadata = meta;
            newInfo._bin      = newBin.get();
        }
        else
        {
            // bin creation failed, so disable caching for this layer.
            _runtimeOptions->cachePolicy()->usage() = CachePolicy::USAGE_NO_CACHE;
            OE_WARN << LC << "Failed to create a cacheing bin for layer \"" << getName() 
                << "\"; cache disabled." 
                << std::endl;
        }

        return newBin.get(); // not release()
    }
}

bool
TerrainLayer::getCacheBinMetadata( const Profile* profile, CacheBinMetadata& output )
{
    // the cache bin ID is the cache IF concatenated with the profile signature.
    std::string binId = *_runtimeOptions->cacheId() + "_" + profile->getSignature();
    CacheBin* bin = getCacheBin( profile );
    if ( bin )
    {
        Threading::ScopedReadLock shared(_cacheBinsMutex);
        CacheBinInfoMap::iterator i = _cacheBins.find( binId );
        if ( i != _cacheBins.end() )
        {
            output = i->second._metadata.value();
            return true;
        }
    }
    return false;
}

void
TerrainLayer::initTileSource()
{	
    OE_DEBUG << LC << "Initializing tile source ..." << std::endl;

    // instantiate it from driver options if it has not already been created:
    if ( !_tileSource.valid() )
    {
        if ( _runtimeOptions->driver().isSet() )
        {
            _tileSource = TileSourceFactory::create( *_runtimeOptions->driver() );
        }
    }

    // next check for an override-profile. The profile usually comes from the
    // TileSource itself, but you have the option of overriding:
	osg::ref_ptr<const Profile> overrideProfile;
	if ( _runtimeOptions->profile().isSet() )
	{
		overrideProfile = Profile::create( *_runtimeOptions->profile() );

        if ( overrideProfile.valid() )
        {
            OE_INFO << LC << "Layer " << getName() << " overrides profile to "
                << overrideProfile->toString() << std::endl;
        }
	}

    // Initialize the profile with the context information:
	if ( _tileSource.valid() )
	{
        // set up the URI options.
        if ( !_dbOptions.valid() )
        {
            _dbOptions = new osgDB::Options();       
            if ( _cache.valid() ) _cache->store( _dbOptions.get() );
            URIContext( _runtimeOptions->referrer() ).store( _dbOptions.get() );
        }

        // intialize the tile source
		_tileSource->initialize( _dbOptions.get(), overrideProfile.get() );

		if ( _tileSource->isOK() )
		{
			_tileSize = _tileSource->getPixelsPerTile();
		}
		else
		{
	        OE_WARN << "Could not initialize TileSource for layer " << getName() << std::endl;
            _tileSource = NULL;
		}
	}

    // Set the profile from the TileSource if possible:
    if ( _tileSource.valid() )
    {
        _profile = _tileSource->getProfile();
    }
    
    // Otherwise, force cache-only mode (since there is no tilesource). The layer will try to 
    // establish a profile from the metadata in the cache instead.
    else if (_cache.valid())
    {
        OE_NOTICE << LC << "Could not initialize TileSource " << _name << ", but a cache exists. Setting layer to cache-only mode." << std::endl;
        _runtimeOptions->cachePolicy()->usage() = CachePolicy::USAGE_CACHE_ONLY;
    }

    _tileSourceInitialized = true;
}

bool
TerrainLayer::isKeyValid(const TileKey& key) const
{
	if (!key.valid()) return false;
	
    // Check to see if explicit levels of detail are set
    if ( _runtimeOptions->minLevel().isSet() && (int)key.getLevelOfDetail() < _runtimeOptions->minLevel().value() )
        return false;
	if ( _runtimeOptions->maxLevel().isSet() && (int)key.getLevelOfDetail() > _runtimeOptions->maxLevel().value() ) 
        return false;
    
    // Check to see if levels of detail based on resolution are set
    if ( getProfile() )
    {
        if ( _runtimeOptions->minLevelResolution().isSet() )
        {        
            unsigned int minLevel = getProfile()->getLevelOfDetailForHorizResolution(
                _runtimeOptions->minLevelResolution().value(), 
                getTileSize() );
            OE_DEBUG << "Computed min level of " << minLevel << std::endl;
            if (key.getLevelOfDetail() < minLevel) 
                return false;
        }

        if (_runtimeOptions->maxLevelResolution().isSet())
        {        
            unsigned int maxLevel = getProfile()->getLevelOfDetailForHorizResolution(
                _runtimeOptions->maxLevelResolution().value(), getTileSize() );
            OE_DEBUG << "Computed max level of " << maxLevel << std::endl;
            if (key.getLevelOfDetail() > maxLevel) return false;
        }
    }

	return true;
}

bool
TerrainLayer::isCached(const TileKey& key) const
{
    CacheBin* bin = const_cast<TerrainLayer*>(this)->getCacheBin( key.getProfile() );
    return bin ? bin->isCached(key.str()) : false;
}

void
TerrainLayer::setEnabled( bool value )
{
    _runtimeOptions->enabled() = value;
    fireCallback( &TerrainLayerCallback::onEnabledChanged );
}

void
TerrainLayer::setDBOptions( const osgDB::Options* dbOptions )
{
    _dbOptions = osg::clone( dbOptions );
}
