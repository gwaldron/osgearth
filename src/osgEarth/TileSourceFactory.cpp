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
TileSourceFactory::createMapTileSource( const MapConfig* mapConfig,
                                        const SourceConfig* source,
                                        const osgDB::ReaderWriter::Options* global_options)
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

    osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ?
        new osgDB::ReaderWriter::Options( *global_options ) : 
        new osgDB::ReaderWriter::Options();

    //Setup the plugin options for the source
    for( SourceProperties::const_iterator p = source->getProperties().begin(); p != source->getProperties().end(); p++ )
    {
        local_options->setPluginData( p->first, (void*)p->second.c_str() );
    }

    bool foundValidSource = false;
    osg::ref_ptr<TileSource> tile_source;

    //Only load the source if we are not running offline
    if ( !mapConfig || !mapConfig->getCacheOnly() )
    {
        //Add the source to the list.  The "." prefix causes OSG to select the correct plugin.
        //For instance, the WMS plugin can be loaded by using ".osgearth_wms" as the filename
        tile_source = dynamic_cast<TileSource*>(osgDB::readObjectFile(".osgearth_" + source->getDriver(), local_options.get()));
        if (!tile_source.valid())
        {
          osg::notify(osg::NOTICE) << "Warning:  Could not load TileSource from "  << source->getDriver() << std::endl;
        }
    }

    if (tile_source.valid())
    {           
        //Initialize the source and set its name
        //tile_source->init(local_options.get());
        tile_source->setName( source->getName() );
        osg::notify(osg::INFO) << "Loaded " << source->getDriver() << " TileSource" << std::endl;
    }

    //Configure the cache if necessary
    osg::ref_ptr<const CacheConfig> cacheConfig = source->getCacheConfig();

    osg::ref_ptr<TileSource> topSource = tile_source.get();

    //If the cache config is valid, wrap the TileSource with a caching TileSource.
    if (cacheConfig.valid())
    {
        osg::ref_ptr<CachedTileSource> cache = CachedTileSourceFactory::create(
            tile_source.get(),
            cacheConfig->getType(),
            cacheConfig->getProperties(),
            local_options.get() );

        if (cache.valid())
        {
            cache->setName(source->getName());
            if ( mapConfig )
                cache->setMapConfigFilename( mapConfig->getFilename() );
            topSource = cache.get();
        }
    }

    MemCachedTileSource* memCachedSource = new MemCachedTileSource(topSource.get(), local_options.get());
    memCachedSource->setName( topSource->getName() );

    // Finally, install an override profile if the caller requested one. This will override the profile
    // that the TileSource reports.
    if ( source->getProfileConfig() )
    {
        osg::ref_ptr<const Profile> override_profile =
            osgEarth::Registry::instance()->getNamedProfile( source->getProfileConfig()->getSRS() );

        if ( !override_profile.valid() )
        {
            double xmin, ymin, xmax, ymax;
            source->getProfileConfig()->getExtents( xmin, ymin, xmax, ymax );
            override_profile = Profile::create( source->getProfileConfig()->getSRS(), xmin, ymin, xmax, ymax );
        }

        if ( override_profile.valid() )
        {
            memCachedSource->setOverrideProfile( override_profile.get() );
        }
    }

    return memCachedSource;
}

TileSource*
TileSourceFactory::createDirectReadTileSource(const SourceConfig* source,
                                              const Profile* profile,
                                              unsigned int tileSize,
                                              const osgDB::ReaderWriter::Options* global_options)
{
    //Create the map source
    TileSource* mapSource = createMapTileSource( NULL, source, global_options );

    //Wrap it in a DirectTileSource that will convert to the map's profile
    TileSource* tileSource = new DirectReadTileSource( mapSource, tileSize, global_options );
    tileSource->initProfile( profile, std::string() );
    tileSource->setName( source->getName() );
    return tileSource;
}