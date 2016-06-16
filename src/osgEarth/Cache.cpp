/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <osgEarth/Cache>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>

using namespace osgEarth;
using namespace osgEarth::Threading;

#define LC "[Cache] "

//------------------------------------------------------------------------

CacheOptions::~CacheOptions()
{
}

void
CacheSettings::integrateCachePolicy(const optional<CachePolicy>& policy)
{
    // integrate the fields that are passed in first:
    if ( policy.isSet() )
        cachePolicy()->mergeAndOverride( policy );

    // then resolve with global overrides from the registry.
    Registry::instance()->resolveCachePolicy( cachePolicy() );
}

void
CacheSettings::store(osgDB::Options* readOptions)
{
    if (readOptions)
    {
        readOptions->setPluginData("osgEarth::CacheSettings", this);
    }
}
 
CacheSettings*
CacheSettings::get(const osgDB::Options* readOptions)
{
    CacheSettings* obj = 0L;
    if (readOptions)
    {
        obj = static_cast<CacheSettings*>(const_cast<void*>((readOptions->getPluginData("osgEarth::CacheSettings"))));
    }
    return obj;
}

//------------------------------------------------------------------------

CacheManager::CacheManager()
{
    // Check the regsitry for a default cache.
    Cache* cache = Registry::instance()->getDefaultCache();
    if (cache)
        setCache(cache);

    // .. and a default policy.
    if (Registry::instance()->defaultCachePolicy().isSet())
        defaultCachePolicy() = Registry::instance()->defaultCachePolicy();
}
        
void
CacheManager::store(osgDB::Options* readOptions)
{
    if (readOptions)
    {
        readOptions->setPluginData("osgEarth::CacheManager", this);
    }
}
 
CacheManager*
CacheManager::get(const osgDB::Options* readOptions)
{
    CacheManager* obj = 0L;
    if (readOptions)
    {
        obj = static_cast<CacheManager*>(const_cast<void*>((readOptions->getPluginData("osgEarth::CacheManager"))));
    }
    return obj;
}
        
osg::ref_ptr<CacheSettings>
CacheManager::getOrCreateSettings(const CacheSettings::ID& id)
{
    Threading::ScopedMutexLock lock(_mutex);
    
    int size = _lut.size();
    osg::ref_ptr<CacheSettings>& settings = _lut[id];
    if (_lut.size() > size)
    {
        // new entry - propagate some properties
        settings = new CacheSettings();
        settings->setCache(_cache.get());
        settings->defaultCachePolicy() = _policy;
        settings->cachePolicy() = _policy;
    }
    return settings;
}

void 
CacheManager::close(const CacheSettings::ID& id)
{
    Threading::ScopedMutexLock lock(_mutex);
    _lut.erase(id);
}

void
CacheManager::setCache(Cache* cache)
{
    if (_cache.get() != cache)
    {
        _cache = cache;

        Threading::ScopedMutexLock lock(_mutex);
        for (LUT::iterator i = _lut.begin(); i != _lut.end(); ++i)
        {
            i->second->setCache(cache);
        }
    }
}

//------------------------------------------------------------------------

Cache::Cache( const CacheOptions& options ) :
_ok     ( true ),
_options( options )
{
    //nop
}

Cache::~Cache()
{
}

Cache::Cache( const Cache& rhs, const osg::CopyOp& op ) :
osg::Object( rhs, op )
{
    _ok = rhs._ok;
}

CacheBin*
Cache::getBin( const std::string& binID )
{
    osg::ref_ptr<CacheBin> _bin;
    _bin = _bins.get( binID );
    return _bin.get();
}

void
Cache::removeBin( CacheBin* bin )
{
    _bins.remove( bin );
}

Cache*
Cache::get(const osgDB::Options* readOptions)
{
    Cache* cache = 0L;
    if (readOptions)
    {
        const CacheManager* cacheManager = CacheManager::get(readOptions);
        cache = cacheManager ? cacheManager->getCache() : 0L;
    }
    return cache;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[CacheFactory] "
#define CACHE_OPTIONS_TAG "__osgEarth::CacheOptions"

Cache*
CacheFactory::create( const CacheOptions& options )
{
    osg::ref_ptr<Cache> result =0L;
    OE_DEBUG << LC << "Initializing cache of type \"" << options.getDriver() << "\"" << std::endl;

    if ( options.getDriver().empty() )
    {
        OE_WARN << LC << "ILLEGAL: no driver set in cache options" << std::endl;
    }
    else if ( options.getDriver() == "tms" )
    {
        OE_WARN << LC << "Sorry, but TMS caching is no longer supported; try \"filesystem\" instead" << std::endl;
    }
    else // try to load from a plugin
    {
        osg::ref_ptr<osgDB::Options> rwopt = Registry::cloneOrCreateOptions();
        rwopt->setPluginData( CACHE_OPTIONS_TAG, (void*)&options );

        std::string driverExt = std::string(".osgearth_cache_") + options.getDriver();
        osgDB::ReaderWriter::ReadResult rr = osgDB::readObjectFile( driverExt, rwopt.get() );
        result = dynamic_cast<Cache*>( rr.getObject() );
        if ( !result.valid() )
        {
            OE_WARN << LC << "Failed to load cache plugin for type \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    return result.release();
}

//------------------------------------------------------------------------

const CacheOptions&
CacheDriver::getCacheOptions( const osgDB::ReaderWriter::Options* rwopt ) const 
{
    return *static_cast<const CacheOptions*>( rwopt->getPluginData( CACHE_OPTIONS_TAG ) );
}

CacheDriver::~CacheDriver()
{
}

//------------------------------------------------------------------------
