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
#include <osgDB/WriteFile>
#include <osg/Version>
#include <OpenThreads/ScopedLock>
#include <memory.h>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TerrainLayer] "

//------------------------------------------------------------------------

TerrainLayerOptions::TerrainLayerOptions( const ConfigOptions& options ) :
ConfigOptions( options ),
_minLevel( 0 ),
_maxLevel( 99 ),
_cacheEnabled( true ),
_cacheOnly( false ),
_loadingWeight( 1.0f ),
_exactCropping( false ),
_enabled( true ),
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
    _cacheEnabled.init( true );
    _cacheOnly.init( false );
    _loadingWeight.init( 1.0f );
    _minLevel.init( 0 );
    _maxLevel.init( 99 );
}

Config
TerrainLayerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();

    conf.attr("name") = _name;
    conf.updateIfSet( "cacheid", _cacheId );
    conf.updateIfSet( "min_level", _minLevel );
    conf.updateIfSet( "max_level", _maxLevel );
    conf.updateIfSet( "min_level_resolution", _minLevelResolution );
    conf.updateIfSet( "max_level_resolution", _maxLevelResolution );
    conf.updateIfSet( "cache_enabled", _cacheEnabled );
    conf.updateIfSet( "cache_only", _cacheOnly );
    conf.updateIfSet( "cache_format", _cacheFormat );
    conf.updateIfSet( "loading_weight", _loadingWeight );
    conf.updateIfSet( "enabled", _enabled );
    conf.updateIfSet( "edge_buffer_ratio", _edgeBufferRatio);
    conf.updateObjIfSet( "profile", _profile );
    conf.updateIfSet( "max_data_level", _maxDataLevel);
    conf.updateIfSet( "reprojected_tilesize", _reprojectedTileSize);

    //Merge the TileSource options
    if (driver().isSet()) conf.merge( driver()->getConfig() );

    return conf;
}

void
TerrainLayerOptions::fromConfig( const Config& conf )
{
    _name = conf.value("name");
    conf.getIfSet( "cacheid", _cacheId );
    conf.getIfSet( "min_level", _minLevel );
    conf.getIfSet( "max_level", _maxLevel );        
    conf.getIfSet( "min_level_resolution", _minLevelResolution );
    conf.getIfSet( "max_level_resolution", _maxLevelResolution );
    conf.getIfSet( "cache_enabled", _cacheEnabled );
    conf.getIfSet( "cache_only", _cacheOnly );
    conf.getIfSet( "cache_format", _cacheFormat );
    conf.getIfSet( "loading_weight", _loadingWeight );
    conf.getIfSet( "enabled", _enabled );
    conf.getIfSet( "edge_buffer_ratio", _edgeBufferRatio);
    conf.getObjIfSet( "profile", _profile );
    conf.getIfSet( "max_data_level", _maxDataLevel);
    conf.getIfSet( "reprojected_tilesize", _reprojectedTileSize);


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

TerrainLayer::TerrainLayer( TerrainLayerOptions* options ) :
_runtimeOptions( options )
{
    init();
}

TerrainLayer::TerrainLayer( TerrainLayerOptions* options, TileSource* tileSource ) :
_runtimeOptions( options ),
_tileSource    ( tileSource )
{
    init();
}

void
TerrainLayer::init()
{
    _tileSourceInitialized = false;
    _tileSize              = 256;
}

void
TerrainLayer::setCache(Cache* cache)
{
    if (_cache.get() != cache)
    {
        _cache = cache;        

        // Read properties from the cache if not already set
        if ( _cache.valid() && _runtimeOptions->cacheEnabled() == true )
        {
            // create the unique cache ID for the tile configuration.
            std::string cacheId;

            if ( _runtimeOptions->cacheId().isSet() && !_runtimeOptions->cacheId()->empty() )
            {
                // user expliticy set a cacheId in the terrain layer options.
                cacheId = *_runtimeOptions->cacheId();
            }
            else
            {
                // system will generate a cacheId.
                Config hashConf = _runtimeOptions->driver()->getConfig();

                // remove cache-control properties before hashing.
                hashConf.remove( "cache_only" );
                hashConf.remove( "cache_enabled" );

                std::stringstream buf;
                //OE_NOTICE << hashConf.toHashString() << std::endl;
                buf << std::fixed << std::setfill('0') << std::hex
                    << osgEarth::hashString( hashConf.toHashString() );
                cacheId = buf.str();
            }

            // try to load the properties from the cache; if that is unsuccesful,
            // create new properties.
            osg::ref_ptr<const Profile> profile;
            _cache->loadProperties( cacheId, _cacheSpec, profile, _tileSize );

            // Set the profile if it hasn't already been set
            if (!_profile.valid() && profile.valid())
            {
                _profile = profile.get();
            }

            // Set the cache format if it hasn't been explicitly set
            if ( !_runtimeOptions->cacheFormat().isSet() )
            {
                _runtimeOptions->cacheFormat() = _cacheSpec.format();
            }

            _cacheSpec = CacheSpec( cacheId, *_runtimeOptions->cacheFormat(), getName() );
        }
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
        (!_tileSource.valid() && _runtimeOptions->cacheOnly() == false) )
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock(const_cast<TerrainLayer*>(this)->_initTileSourceMutex );
        // double-check pattern
        if ((_tileSource.valid() && !_tileSourceInitialized) ||
            (!_tileSource.valid() && _runtimeOptions->cacheOnly() == false))
        {
            const_cast<TerrainLayer*>(this)->initTileSource();
        }
    }

    return _tileSource.get();
}

const Profile*
TerrainLayer::getProfile() const
{
    if ( !_profile.valid() )
    {
        if ( _runtimeOptions->cacheOnly() == false && !_tileSourceInitialized )
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

//TODO: move this to ImageLayer/ElevationLayer
std::string
TerrainLayer::suggestCacheFormat() const
{
    std::string ext = _tileSource.valid() ? _tileSource->getExtension() : "";
    return !ext.empty() ? ext : "png";
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
	}

    // Initialize the profile with the context information:
	if ( _tileSource.valid() )
	{
		_tileSource->initialize( _referenceURI, overrideProfile.get() );

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
    
    // Set the cache format to the native format of the TileSource if it isn't already set.
    if ( _runtimeOptions->cacheFormat()->empty() )
    {
        _runtimeOptions->cacheFormat() = suggestCacheFormat();
        _cacheSpec = CacheSpec( _cacheSpec.cacheId(), *_runtimeOptions->cacheFormat(), _cacheSpec.name() );
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
        OE_NOTICE << "Could not initialize TileSource " << _name << " but cache is valid.  Setting layer to cache_only." << std::endl;
        _runtimeOptions->cacheOnly() = true;
    }

	// check the environment to see if cache only should be enabled
    if ( _runtimeOptions->cacheOnly() == false && ::getenv("OSGEARTH_CACHE_ONLY") != 0 )
	{
        _runtimeOptions->cacheOnly() = true;
        OE_INFO << "CACHE-ONLY mode enabled!!" << std::endl;
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
    if ( _runtimeOptions->minLevelResolution().isSet() )
    {        
        unsigned int minLevel = getProfile()->getLevelOfDetailForHorizResolution(
            _runtimeOptions->minLevelResolution().value(), getTileSize() );
        OE_DEBUG << "Computed min level of " << minLevel << std::endl;
        if (key.getLevelOfDetail() < minLevel) return false;
    }

    if (_runtimeOptions->maxLevelResolution().isSet())
    {        
        unsigned int maxLevel = getProfile()->getLevelOfDetailForHorizResolution(
            _runtimeOptions->maxLevelResolution().value(), getTileSize() );
        OE_DEBUG << "Computed max level of " << maxLevel << std::endl;
        if (key.getLevelOfDetail() > maxLevel) return false;
    }

	return true;
}

void
TerrainLayer::setEnabled( bool value )
{
    _runtimeOptions->enabled() = value;
    fireCallback( &TerrainLayerCallback::onEnabledChanged );
}

void
TerrainLayer::setReferenceURI( const std::string& uri )
{
    _referenceURI = uri;
}

void
TerrainLayer::setCacheOnly( bool value )
{
    _runtimeOptions->cacheOnly() = value;
}
