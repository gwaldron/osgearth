/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/TileSourceFactory>
#include <osgEarth/Caching>
#include <osgEarth/DirectReadTileSource>
#include <osgEarth/Registry>

using namespace osgEarth;

TileSource* 
TileSourceFactory::createMapTileSource(const SourceConfig& sourceConfig,
                                       const MapConfig& mapConfig )
{
    /*
    *The map tile source chain will look like
    *MemCachedTileSource -> DiskCachedTileSource -> TileSource
    *
    *If no cache is configured, the chain will look like
    *MemCachedTileSource -> TileSource
    *
    *If the cache only option is specified, the chain will look like
    *MemCachedTileSource -> DiskCachedTileSource
    */

    const osgDB::ReaderWriter::Options* global_options = mapConfig.getGlobalOptions();

    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ?
        new osgDB::ReaderWriter::Options( *global_options ) : 
        new osgDB::ReaderWriter::Options();

    //Setup the plugin options for the source
    for( SourceProperties::const_iterator p = sourceConfig.getProperties().begin(); p != sourceConfig.getProperties().end(); p++ )
    {
        local_options->setPluginData( p->first, (void*)p->second.c_str() );
    }

    bool foundValidSource = false;
    osg::ref_ptr<TileSource> tile_source;

    bool cacheOnlyEnv = false;
    //If the OSGEARTH_CACHE_ONLY environment variable is set, override whateve is in the map config
    if (getenv("OSGEARTH_CACHE_ONLY") != 0)
    {
        osg::notify(osg::NOTICE) << "[osgEarth::MapConfig] Setting osgEarth to cache only mode due to OSGEARTH_CACHE_ONLY environment variable " << std::endl;
        cacheOnlyEnv = true;
    }

     //Only load the source if we are not running offline
    if ( !mapConfig.getCacheOnly() && !cacheOnlyEnv )
    {
        //Add the source to the list.  The "." prefix causes OSG to select the correct plugin.
        //For instance, the WMS plugin can be loaded by using ".osgearth_wms" as the filename
        tile_source = dynamic_cast<TileSource*>(
            osgDB::readObjectFile(".osgearth_" + sourceConfig.getDriver(), local_options.get()));

        if (!tile_source.valid())
        {
          osg::notify(osg::NOTICE) << "Warning:  Could not load TileSource from "  << sourceConfig.getDriver() << std::endl;
        }
    }

    if (tile_source.valid())
    {           
        //Initialize the source and set its name
        tile_source->setName( sourceConfig.getName() );
        osg::notify(osg::INFO) << "Loaded " << sourceConfig.getDriver() << " TileSource" << std::endl;

		//If the TileSource is set to reproject before caching occurs, wrap the TileSource in a DirectReadTileSource.
		//This allows us to pull down imagery from pretiled datasources such as TMS or WMS-C that may not be in
		//the correct SRS, reproject them to the cache, and have a nice fast map once caching occurs.
		if (sourceConfig.getReprojectBeforeCaching())
		{			
			//Try to read in a preferred tile_size.
			int tile_size = as<int>(sourceConfig["tile_size"], 256);
			osg::notify(osg::INFO) << "Wrapping " << sourceConfig.getName() << " in DirectReadTileSource with a tile size of " << tile_size << std::endl;
			tile_source = new DirectReadTileSource( tile_source, tile_size );
		}
    }

    //Configure the cache if necessary
    CacheConfig cacheConfig = sourceConfig.getCacheConfig();

    //Inherit from the MapConfig if it is defined
    if (mapConfig.getCacheConfig().defined())
    {
        cacheConfig.inheritFrom(mapConfig.getCacheConfig());
    }

    //Inherit the map CacheConfig with the override from the registry
    if ( Registry::instance()->getCacheConfigOverride().defined() )
    {
        cacheConfig.inheritFrom( Registry::instance()->getCacheConfigOverride() );
        osg::notify(osg::NOTICE) << "[osgEarth::MapConfig] Overriding Map Cache" << std::endl;
    }

    osg::ref_ptr<TileSource> topSource = tile_source.get();

    //If the cache config is valid, wrap the TileSource with a caching TileSource.
    if ( cacheConfig.defined() ) // && tile_source->supportsPersistentCaching() )
    {
        osg::ref_ptr<CachedTileSource> cache = CachedTileSourceFactory::create(
            tile_source.get(),
            cacheConfig.getType(),
            cacheConfig.getProperties(),
            local_options.get() );

        if (cache.valid())
        {
            cache->setName(sourceConfig.getName());
            cache->setMapConfigFilename( mapConfig.getFilename() );
            topSource = cache.get();
        }
    }

    MemCachedTileSource* memCachedSource = new MemCachedTileSource(topSource.get(), local_options.get());
    memCachedSource->setName( topSource->getName() );

    // Finally, install an override profile if the caller requested one. This will override the profile
    // that the TileSource reports.
    if ( sourceConfig.getProfileConfig().defined() )
    {
        const ProfileConfig& pconf = sourceConfig.getProfileConfig();

        osg::ref_ptr<const Profile> override_profile;

        if ( !pconf.getNamedProfile().empty() )
        {
            override_profile = osgEarth::Registry::instance()->getNamedProfile( pconf.getNamedProfile() );
        }

        if ( !override_profile.valid() && !pconf.getSRS().empty() )
        {
            double xmin, ymin, xmax, ymax;
            pconf.getExtents( xmin, ymin, xmax, ymax );
            override_profile = Profile::create( pconf.getSRS(), xmin, ymin, xmax, ymax );
        }

        if ( override_profile.valid() )
        {
            memCachedSource->setOverrideProfile( override_profile.get() );
        }
    }

    return memCachedSource;
}

TileSource*
TileSourceFactory::createDirectReadTileSource(const SourceConfig& sourceConfig,
                                              const Profile* profile,
                                              unsigned int tileSize,
                                              const osgDB::ReaderWriter::Options* global_options)
{
    //Create the map source
    MapConfig mapConfig;
    mapConfig.setGlobalOptions( global_options );
    TileSource* source = createMapTileSource( sourceConfig, mapConfig );

    //Wrap it in a DirectTileSource that will convert to the map's profile
    TileSource* tileSource = new DirectReadTileSource( source, tileSize, global_options );
    tileSource->initProfile( profile, std::string() );
    tileSource->setName( sourceConfig.getName() );
    return tileSource;
}