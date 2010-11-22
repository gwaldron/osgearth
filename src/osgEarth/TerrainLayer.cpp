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
_enabled( true ),
_exactCropping( false ),
_reprojectedTileSize( 256 ),
_cacheEnabled( true ),
_cacheOnly( false ),
_loadingWeight( 1.0f ),
_minLevel( 0 ),
_maxLevel( 99 )
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
    conf.updateIfSet( "cache_enabled", _cacheEnabled );
    conf.updateIfSet( "cache_only", _cacheOnly );
    conf.updateIfSet( "cache_format", _cacheFormat );
    conf.updateIfSet( "loading_weight", _loadingWeight );
    conf.updateIfSet( "enabled", _enabled );
    conf.updateIfSet( "edge_buffer_ratio", _edgeBufferRatio);
    conf.updateObjIfSet( "profile", _profile );

    return conf;
}

void
TerrainLayerOptions::fromConfig( const Config& conf )
{
    _name = conf.value("name");
    conf.getIfSet( "cacheid", _cacheId );
    conf.getIfSet( "min_level", _minLevel );
    conf.getIfSet( "max_level", _maxLevel );
    conf.getIfSet( "cache_enabled", _cacheEnabled );
    conf.getIfSet( "cache_only", _cacheOnly );
    conf.getIfSet( "cache_format", _cacheFormat );
    conf.getIfSet( "loading_weight", _loadingWeight );
    conf.getIfSet( "enabled", _enabled );
    conf.getIfSet( "edge_buffer_ratio", _edgeBufferRatio);
    conf.getObjIfSet( "profile", _profile );
}

void
TerrainLayerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//------------------------------------------------------------------------

TerrainLayer::TerrainLayer()
{
    init();
}

TerrainLayer::TerrainLayer( TileSource* tileSource ) :
_tileSource( tileSource )
{
    init();
}

void
TerrainLayer::init()
{
    // Warning: don't access getTerrainLayerOptions() here, since it's a virtual function.

    _tileSize          = 256;
    _actualCacheFormat = "";
    _actualCacheOnly   = false;
    _actualEnabled     = true;

	// Parse any environment variables:
    if ( ::getenv("OSGEARTH_CACHE_ONLY") != 0 )
	{
		_cacheOnlyEnv = true;
		_actualCacheOnly = true;
        OE_INFO << "CACHE-ONLY mode enabled!!" << std::endl;
	}
}

void
TerrainLayer::setCache(Cache* cache)
{
    if (_cache.get() != cache)
    {
        _cache = cache;        

        const TerrainLayerOptions& opt = getTerrainLayerOptions();

        // Read properties from the cache if not already set
        if (_cache.valid() && opt.cacheEnabled() == true )
        {
            // create the unique cache ID for the tile configuration.
            std::string cacheId;

            if ( opt.cacheId().isSet() && !opt.cacheId()->empty() )
            {
                // user expliticy set a cacheId in the terrain layer options.
                cacheId = *opt.cacheId();
            }
            else
            {
                // system will generate a cacheId.
                std::stringstream buf;
                buf << std::fixed << std::setfill('0') << std::hex
                    << osgEarth::hashString( opt.driver()->getConfig().toHashString() );
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
            _actualCacheFormat = opt.cacheFormat().value();
            if ( _actualCacheFormat.empty() )
                _actualCacheFormat = _cacheSpec.format();

            // check for a cache-only override.
            if ( _overrideCacheOnly.isSetTo( true ) )
                _actualCacheOnly = true;

            _cacheSpec = CacheSpec( cacheId, _actualCacheFormat, getName() );
        }
    }
}

TileSource* 
TerrainLayer::getTileSource() const
{
    const TerrainLayerOptions& opt = getTerrainLayerOptions();

	//Only load the TileSource if it hasn't been loaded previously and we aren't running strictly off the cache.
	if ( !_tileSource.valid() && opt.cacheOnly() == false )
	{
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock(const_cast<TerrainLayer*>(this)->_initTileSourceMutex );
        //Double check pattern
        if (!_tileSource.valid() && opt.cacheOnly() == false )
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
        const TerrainLayerOptions& opt = getTerrainLayerOptions();

        if ( opt.cacheOnly() == false && !_tileSource.valid() )
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
	TileSource* ts = getTileSource();
	if (ts)
	{
		return ts->getMaxDataLevel();
	}
	return 20;
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

    const TerrainLayerOptions& opt = getTerrainLayerOptions();

    osg::ref_ptr<TileSource> tileSource;

    if ( opt.driver().isSet() )
    {
        tileSource = TileSourceFactory::create( opt.driver().value() );
    }

	//Get the override profile if it is set.
	osg::ref_ptr<const Profile> overrideProfile;
	if ( opt.profile().isSet() )
	{
		overrideProfile = Profile::create( opt.profile().value() );
	}

	if ( tileSource.valid() )
	{
		// Initialize the TileSource
		tileSource->initialize( _referenceURI, overrideProfile.get() );

		if ( tileSource->isOK() )
		{
			_tileSize = tileSource->getPixelsPerTile();
		}
		else
		{
	        OE_WARN << "Could not initialize TileSource for layer " << getName() << std::endl;
            tileSource = NULL;
		}
	}
	_tileSource = tileSource;
    
    //Set the cache format to the native format of the TileSource if it isn't already set.
    _actualCacheFormat = opt.cacheFormat().value();
    if ( _actualCacheFormat.empty() )
        _actualCacheFormat = suggestCacheFormat();

    if ( _tileSource.valid() )
    {
        // Set the profile from the TileSource
        _profile = _tileSource->getProfile();
    }
    else if (_cache.valid())
    {
        OE_NOTICE << "Could not initialize TileSource " << _name << " but cache is valid.  Setting layer to cache_only." << std::endl;
        _actualCacheOnly = true;
    }

    // check for a cache-only override.
    if ( _overrideCacheOnly.isSetTo( true ) )
        _actualCacheOnly = true;
}

bool
TerrainLayer::isKeyValid(const TileKey& key) const
{
	if (!key.valid()) return false;
    const TerrainLayerOptions& opt = getTerrainLayerOptions();
	if ( opt.minLevel().isSet() && (int)key.getLevelOfDetail() < opt.minLevel().value() ) return false;
	if ( opt.maxLevel().isSet() && (int)key.getLevelOfDetail() > opt.maxLevel().value() ) return false;
	return true;
}

void
TerrainLayer::setEnabled( bool value )
{
    _actualEnabled = value;
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
    _overrideCacheOnly = value;
}
