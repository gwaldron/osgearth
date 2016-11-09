/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

#include <osg/UserDataContainer>
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
CacheSettings::store(osgDB::Options* readOptions)
{
    if (readOptions)
    {
        osg::UserDataContainer* udc = readOptions->getOrCreateUserDataContainer();
        unsigned index = udc->getUserObjectIndex(CACHESETTINGS_UDC_NAME);
        udc->removeUserObject(index);
        udc->addUserObject(this);
    }
}
 
CacheSettings*
CacheSettings::get(const osgDB::Options* readOptions)
{
    CacheSettings* obj = 0L;
    if (readOptions)
    {
        const osg::UserDataContainer* udc = readOptions->getUserDataContainer();
        if (udc) {
            osg::Object* temp = const_cast<osg::Object*>(udc->getUserObject(CACHESETTINGS_UDC_NAME));
            obj = dynamic_cast<CacheSettings*>(temp);
        }
    }
    return obj;
}

std::string
CacheSettings::toString() const
{
    return Stringify()
        << "cache=" << (_cache.valid() ? _cache->className() : "none")
        << "; policy=" << _policy->usageString()
        << "; bin=" << (_activeBin.get() ? "yes" : "no");
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
    static CacheOptions s_default;
    const void* data = rwopt->getPluginData(CACHE_OPTIONS_TAG);
    return data ? *static_cast<const CacheOptions*>(data) : s_default;
}

CacheDriver::~CacheDriver()
{
}

//------------------------------------------------------------------------
