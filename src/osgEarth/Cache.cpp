/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/Utils>
#include "sha1.hpp"
#include <osgEarth/Notify>

#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Threading;

#define LC "[Cache] "

//------------------------------------------------------------------------

CacheOptions::~CacheOptions()
{
}

Config
CacheOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.set("enable_node_caching", enableNodeCaching());
    return conf;
}

void
CacheOptions::fromConfig(const Config& conf)
{
    enableNodeCaching().setDefault(false);
    conf.get("enable_node_caching", enableNodeCaching());
}

//------------------------------------------------------------------------

#define CACHESETTINGS_UDC_NAME "osgEarth.CacheSettings"

CacheSettings::CacheSettings()
{
    setName(CACHESETTINGS_UDC_NAME);
}

CacheSettings::CacheSettings(const CacheSettings& rhs, const osg::CopyOp& copy) :
osg::Object(rhs, copy),
_cache(rhs._cache.get()),
_policy(rhs._policy),
_activeBin(rhs._activeBin.get())
{
    //nop
}
        
bool
CacheSettings::isCacheEnabled() const
{
    return _cache.valid() && _policy->isCacheEnabled();
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
CacheSettings::integrateCachePolicy(const CachePolicy& policy)
{
    optional<CachePolicy> temp;
    temp = policy;
    integrateCachePolicy(temp);
}

void
CacheSettings::store(osgDB::Options* readOptions)
{
    if (readOptions)
    {
        ObjectStorage::set(readOptions, this);
    }
}
 
CacheSettings*
CacheSettings::get(const osgDB::Options* readOptions)
{
    osg::ref_ptr<CacheSettings> settings;
    ObjectStorage::get(readOptions, settings);
    return settings.get();
}

std::string
CacheSettings::toString() const
{
    if (_cache.valid() && _policy->isCacheEnabled())
    {
        return Stringify()
            << "[cache=" << (_cache.valid() ? _cache->className() : "none")
            << "; policy=" << _policy->usageString()
            << "; bin=" << (_activeBin.get() ? "yes" : "no")
            << "]";
    }
    else
    {
        return "[no cache]";
    }
}

//------------------------------------------------------------------------

Cache::Cache( const CacheOptions& options ) :
_options( options ),
_bins("OE.Cache.bins")
{
    //nop
}

Cache::~Cache()
{
}

Cache::Cache( const Cache& rhs, const osg::CopyOp& op ) :
osg::Object( rhs, op ),
_bins("OE.Cache.bins")
{
    _status = rhs._status;
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
    _bins.remove( bin->getID() );
}

namespace
{
    int hash8(const std::string& str)
    {
        int hash = 0;
        for(unsigned i=0; i<str.length(); ++i)
            hash += (int)str[i];
        return hash;
    }
}

std::string
Cache::makeCacheKey(const std::string& key, const std::string& prefix)
{
    char hex[SHA1_HEX_SIZE];
    sha1(key.c_str()).finalize().print_hex(hex);
    std::string val(hex);
    std::stringstream out;
    if (!prefix.empty())
    {
        out << prefix << "/";
    }
    // Use the first 2 characters as a directory name and the remainder as the filename
    // This is the same scheme that git uses
    out << val.substr(0, 2) << "/" << val.substr(2, 38);
    return out.str();
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[CacheFactory] "
#define CACHE_OPTIONS_TAG "__osgEarth::CacheOptions"

Cache*
CacheFactory::create( const CacheOptions& options )
{
    osg::ref_ptr<Cache> result;
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
        osg::ref_ptr<osg::Object> object = osgDB::readRefObjectFile( driverExt, rwopt.get() );
        result = dynamic_cast<Cache*>( object.release() );
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
    static CacheOptions s_default;
    const void* data = rwopt->getPluginData(CACHE_OPTIONS_TAG);
    return data ? *static_cast<const CacheOptions*>(data) : s_default;
}

CacheDriver::~CacheDriver()
{
}

//------------------------------------------------------------------------
