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
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Cube>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/TaskService>
#include <osgEarth/IOTypes>
#include <osgEarth/ColorFilter>
#include <osgEarth/StateSetCache>
#include <osgEarth/HTTPClient>
#include <osgEarth/StringUtils>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ObjectIndex>

#include <osgEarth/Units>
#include <osg/Notify>
#include <osg/Version>
#include <osgDB/Registry>
#include <osgDB/Options>
#include <osgText/Font>

#include <gdal_priv.h>
#include <ogr_api.h>
#include <stdlib.h>
#include <locale>

using namespace osgEarth;
using namespace OpenThreads;

#define STR_GLOBAL_GEODETIC    "global-geodetic"
#define STR_GLOBAL_MERCATOR    "global-mercator"
#define STR_SPHERICAL_MERCATOR "spherical-mercator"
#define STR_CUBE               "cube"
#define STR_LOCAL              "local"

#define LC "[Registry] "


Registry::Registry() :
osg::Referenced     ( true ),
_gdal_registered    ( false ),
_numGdalMutexGets   ( 0 ),
_uidGen             ( 0 ),
_caps               ( 0L ),
_defaultFont        ( 0L ),
_terrainEngineDriver( "mp" ),
_cacheDriver        ( "filesystem" ),
_overrideCachePolicyInitialized( false )
{
    // set up GDAL and OGR.
    OGRRegisterAll();
    GDALAllRegister();
    
    // support Chinese character in the file name and attributes in ESRI's shapefile
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");
    CPLSetConfigOption("SHAPE_ENCODING","");

    // global initialization for CURL (not thread safe)
    HTTPClient::globalInit();

    // generates the basic shader code for the terrain engine and model layers.
    _shaderLib = new ShaderFactory();

    // shader generator used internally by osgEarth. Can be replaced.
    _shaderGen = new ShaderGenerator();

    // thread pool for general use
    _taskServiceManager = new TaskServiceManager();

    // optimizes sharing of state attributes and state sets for
    // performance boost
    _stateSetCache = new StateSetCache();

    // Default unref-after apply policy:
    _unRefImageDataAfterApply = true;

    // Default object index for tracking scene object by UID.
    _objectIndex = new ObjectIndex();

    // activate KMZ support
    osgDB::Registry::instance()->addArchiveExtension  ( "kmz" );
    //osgDB::Registry::instance()->addFileExtensionAlias( "kmz", "kml" );

    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/vnd.google-earth.kml+xml", "kml" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/vnd.google-earth.kmz",     "kmz" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/plain",                           "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/xml",                             "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/json",                     "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/json",                            "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/x-json",                          "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "image/jpg",                            "jpg" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "image/dds",                            "dds" );
    
    // pre-load OSG's ZIP plugin so that we can use it in URIs
    std::string zipLib = osgDB::Registry::instance()->createLibraryNameForExtension( "zip" );
    if ( !zipLib.empty() )
        osgDB::Registry::instance()->loadLibrary( zipLib );    

    // set up our default r/w options to NOT cache archives!
    _defaultOptions = new osgDB::Options();
    _defaultOptions->setObjectCacheHint( osgDB::Options::CACHE_NONE );

    const char* teStr = ::getenv(OSGEARTH_ENV_TERRAIN_ENGINE_DRIVER);
    if ( teStr )
    {
        _terrainEngineDriver = std::string(teStr);
    }

    // load a default font
    const char* envFont = ::getenv("OSGEARTH_DEFAULT_FONT");
    if ( envFont )
    {
        _defaultFont = osgText::readFontFile( std::string(envFont) );
    }
    if ( !_defaultFont.valid() )
    {
#ifdef WIN32
        _defaultFont = osgText::readFontFile("arial.ttf");
#endif
    }
    if ( _defaultFont.valid() )
    {
        // mitigates mipmapping issues that cause rendering artifacts
        // for some fonts/placement
        _defaultFont->setGlyphImageMargin( 2 );
    }

    // register the system stock Units.
    Units::registerAll( this );
}

Registry::~Registry()
{
    //nop
}

Registry* 
Registry::instance(bool erase)
{
    static osg::ref_ptr<Registry> s_registry = new Registry;

    if (erase) 
    {   
        s_registry->destruct();
        s_registry = 0;
    }

    return s_registry.get(); // will return NULL on erase
}

void 
Registry::destruct()
{
    //nop
}


OpenThreads::ReentrantMutex&
Registry::getGDALMutex()
{
    //_numGdalMutexGets++;
    //OE_NOTICE << "GDAL = " << _numGdalMutexGets << std::endl;
    return _gdal_mutex;
}


const Profile*
Registry::getGlobalGeodeticProfile() const
{
    if ( !_global_geodetic_profile.valid() )
    {
        GDAL_SCOPED_LOCK;

        if ( !_global_geodetic_profile.valid() ) // double-check pattern
        {
            const_cast<Registry*>(this)->_global_geodetic_profile = Profile::create(
                "epsg:4326",
                -180.0, -90.0, 180.0, 90.0,
                "",
                2, 1 );
        }
    }
    return _global_geodetic_profile.get();
}


const Profile*
Registry::getGlobalMercatorProfile() const
{
    return getSphericalMercatorProfile();
}


const Profile*
Registry::getSphericalMercatorProfile() const
{
    if ( !_spherical_mercator_profile.valid() )
    {
        GDAL_SCOPED_LOCK;

        if ( !_spherical_mercator_profile.valid() ) // double-check pattern
        {
            // automatically figure out proper mercator extents:
            const SpatialReference* srs = SpatialReference::create( "spherical-mercator" );

            //double e, dummy;
            //srs->getGeographicSRS()->transform2D( 180.0, 0.0, srs, e, dummy );
            //const_cast<Registry*>(this)->_global_mercator_profile = Profile::create(
            //    srs, -e, -e, e, e, 1, 1 );
            const_cast<Registry*>(this)->_spherical_mercator_profile = Profile::create(
                srs, MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY, 1, 1 );
        }
    }
    return _spherical_mercator_profile.get();
}

const Profile*
Registry::getCubeProfile() const
{
    if ( !_cube_profile.valid() )
    {
        GDAL_SCOPED_LOCK;

        if ( !_cube_profile.valid() ) // double-check pattern
        {
            const_cast<Registry*>(this)->_cube_profile = new UnifiedCubeProfile();
        }
    }
    return _cube_profile.get();
}

const Profile*
Registry::getNamedProfile( const std::string& name ) const
{
    if ( name == STR_GLOBAL_GEODETIC )
        return getGlobalGeodeticProfile();
    else if ( name == STR_GLOBAL_MERCATOR )
        return getGlobalMercatorProfile();
    else if ( name == STR_SPHERICAL_MERCATOR )
        return getSphericalMercatorProfile();
    else if ( name == STR_CUBE )
        return getCubeProfile();
    else
        return NULL;
}

void
Registry::setDefaultCachePolicy( const CachePolicy& value )
{
    _defaultCachePolicy = value;
}

void
Registry::setOverrideCachePolicy( const CachePolicy& value )
{
    _overrideCachePolicy = value;
}

bool
Registry::resolveCachePolicy(optional<CachePolicy>& cp) const
{
    optional<CachePolicy> new_cp;

    // start with the defaults
    if ( defaultCachePolicy().isSet() )
        new_cp = defaultCachePolicy();

    // merge in any set properties from the caller's CP, since they override
    // the defaults:
    if ( cp.isSet() )
        new_cp->mergeAndOverride( cp );

    // finally, merge in any set props from the OVERRIDE CP, which take
    // priority over everything else.
    if ( overrideCachePolicy().isSet() )
        new_cp->mergeAndOverride( overrideCachePolicy() );

    // return the new composited cache policy.
    cp = new_cp;
    return cp.isSet();
}

const std::string&
Registry::getDefaultCacheDriverName() const
{
    if (!_cacheDriver.isSet())
    {
        Threading::ScopedMutexLock lock(_regMutex);

        if (!_cacheDriver.isSet())
        {
            // see if the environment specifies a default caching driver.
            const char* value = ::getenv(OSGEARTH_ENV_CACHE_DRIVER);
            if ( value )
            {
                _cacheDriver = value;
                OE_DEBUG << LC << "Cache driver set from environment: " << value << std::endl;
            }        
        }
    }
    return _cacheDriver.get();
}

const optional<CachePolicy>&
Registry::defaultCachePolicy() const
{
    return _defaultCachePolicy;
}

const optional<CachePolicy>&
Registry::overrideCachePolicy() const
{
    if ( !_overrideCachePolicyInitialized )
    {
        Threading::ScopedMutexLock lock(_regMutex);

        if ( !_overrideCachePolicyInitialized )
        {
            // activate no-cache mode from the environment
            if ( ::getenv(OSGEARTH_ENV_NO_CACHE) )
            {
                _overrideCachePolicy = CachePolicy::NO_CACHE;
                OE_INFO << LC << "NO-CACHE MODE set from environment" << std::endl;
            }
            else
            {
                // activate cache-only mode from the environment
                if ( ::getenv(OSGEARTH_ENV_CACHE_ONLY) )
                {
                    _overrideCachePolicy->usage() = CachePolicy::USAGE_CACHE_ONLY;
                    OE_INFO << LC << "CACHE-ONLY MODE set from environment" << std::endl;
                }

                // cache max age?
                const char* cacheMaxAge = ::getenv(OSGEARTH_ENV_CACHE_MAX_AGE);
                if ( cacheMaxAge )
                {
                    TimeSpan maxAge = osgEarth::as<long>( std::string(cacheMaxAge), INT_MAX );
                    _overrideCachePolicy->maxAge() = maxAge;
                }
            }

            _overrideCachePolicyInitialized = true;
        }
    }
    return _overrideCachePolicy;
}

osgEarth::Cache*
Registry::getDefaultCache() const
{
    if (!_defaultCache.valid())
    {
        std::string driverName = getDefaultCacheDriverName();

        Threading::ScopedMutexLock lock(_regMutex);
        if (!_defaultCache.valid())
        {
            const char* noCache = ::getenv(OSGEARTH_ENV_NO_CACHE);
            if (noCache == 0L)
            {
                // see if there's a cache in the envvar; if so, create a cache.
                // Note: the value of the OSGEARTH_CACHE_PATH is not used here; rather
                // it's used in the driver(s) itself.
                const char* cachePath = ::getenv(OSGEARTH_ENV_CACHE_PATH);
                if (cachePath && !driverName.empty())
                {
                    CacheOptions cacheOptions;
                    cacheOptions.setDriver(driverName);
                    _defaultCache = CacheFactory::create(cacheOptions);
                }
            }
        }
    }
    return _defaultCache.get();
}

void
Registry::setDefaultCache(Cache* cache)
{
    _defaultCache = cache;
}

bool
Registry::isBlacklisted(const std::string& filename)
{
    Threading::ScopedReadLock sharedLock(_blacklistMutex);
    return (_blacklistedFilenames.count(filename)==1);
}

void
Registry::blacklist(const std::string& filename)
{
    {
        Threading::ScopedWriteLock exclusiveLock(_blacklistMutex);
        _blacklistedFilenames.insert( filename );
    }
    OE_DEBUG << "Blacklist size = " << _blacklistedFilenames.size() << std::endl;
}

void
Registry::clearBlacklist()
{
    Threading::ScopedWriteLock exclusiveLock(_blacklistMutex);
    _blacklistedFilenames.clear();
}

unsigned int
Registry::getNumBlacklistedFilenames()
{
    Threading::ScopedReadLock sharedLock(_blacklistMutex);
    return _blacklistedFilenames.size();
}

bool
Registry::hasCapabilities() const
{
    return _caps.valid();
}

const Capabilities&
Registry::getCapabilities() const
{
    if ( !_caps.valid() )
        const_cast<Registry*>(this)->initCapabilities();

    return *_caps;
}

void
Registry::setCapabilities( Capabilities* caps )
{
    _caps = caps;
}

void
Registry::initCapabilities()
{
    ScopedLock<Mutex> lock( _capsMutex ); // double-check pattern (see getCapabilities)
    if ( !_caps.valid() )
        _caps = new Capabilities();
}

const ShaderFactory*
Registry::getShaderFactory() const
{
    return _shaderLib.get();
}

void
Registry::setShaderFactory( ShaderFactory* lib )
{
    if ( lib != 0L && lib != _shaderLib.get() )
        _shaderLib = lib;
}

ShaderGeneratorProxy
Registry::getShaderGenerator() const
{
    return ShaderGeneratorProxy(_shaderGen.get());
}

void
Registry::setShaderGenerator(ShaderGenerator* shaderGen)
{
    if ( shaderGen != 0L && shaderGen != _shaderGen.get() )
        _shaderGen = shaderGen;
}
        
void
Registry::setURIReadCallback( URIReadCallback* callback ) 
{ 
    _uriReadCallback = callback;
}

URIReadCallback*
Registry::getURIReadCallback() const
{
    return _uriReadCallback.get(); 
}

void
Registry::setDefaultFont( osgText::Font* font )
{
    Threading::ScopedMutexLock exclusive(_regMutex);
    _defaultFont = font;
}

osgText::Font*
Registry::getDefaultFont()
{
    Threading::ScopedMutexLock shared(_regMutex);
    return _defaultFont.get();
}

UID
Registry::createUID()
{
    //todo: use OpenThreads::Atomic for this
    ScopedLock<Mutex> exclusive( _uidGenMutex );
    return (UID)( _uidGen++ );
}

const osgDB::Options*
Registry::getDefaultOptions() const 
{
    return _defaultOptions.get();
}

osgDB::Options*
Registry::cloneOrCreateOptions(const osgDB::Options* input)
{
    osgDB::Options* newOptions = 
        input ? static_cast<osgDB::Options*>(input->clone(osg::CopyOp::DEEP_COPY_USERDATA)) : 
        new osgDB::Options();

    // clear the CACHE_ARCHIVES flag because it is evil
    if ( ((int)newOptions->getObjectCacheHint() & osgDB::Options::CACHE_ARCHIVES) != 0 )
    {
        newOptions->setObjectCacheHint( (osgDB::Options::CacheHintOptions)
            ((int)newOptions->getObjectCacheHint() & ~osgDB::Options::CACHE_ARCHIVES) );
    }

    return newOptions;
}

void
Registry::registerUnits( const Units* units )
{
    Threading::ScopedWriteLock exclusive( _unitsVectorMutex );
    _unitsVector.push_back(units);
}

const Units*
Registry::getUnits(const std::string& name) const
{
    Threading::ScopedReadLock shared( _unitsVectorMutex );

    std::string lower = toLower(name);
    for( UnitsVector::const_iterator i = _unitsVector.begin(); i != _unitsVector.end(); ++i )
    {
        if (toLower((*i)->getName()) == lower ||
            toLower((*i)->getAbbr()) == lower)
        {
            return *i;
        }
    }
    return 0L;
}

void
Registry::setDefaultTerrainEngineDriverName(const std::string& name)
{
    _terrainEngineDriver = name;
}

void
Registry::setDefaultCacheDriverName(const std::string& name)
{
    _cacheDriver = name;
}

void
Registry::setStateSetCache( StateSetCache* cache )
{
    _stateSetCache = cache;
}

StateSetCache*
Registry::getStateSetCache() const
{
    return _stateSetCache.get();
}

ProgramSharedRepo*
Registry::getProgramSharedRepo()
{
    return &_programRepo;
}

ObjectIndex*
Registry::getObjectIndex() const
{
    return _objectIndex.get();
}

void
Registry::startActivity(const std::string& activity)
{
    Threading::ScopedMutexLock lock(_activityMutex);
    _activities.insert(Activity(activity,std::string()));
}

void
Registry::startActivity(const std::string& activity,
                        const std::string& value)
{
    Threading::ScopedMutexLock lock(_activityMutex);
    _activities.erase(Activity(activity,std::string()));
    _activities.insert(Activity(activity,value));
}

void
Registry::endActivity(const std::string& activity)
{
    Threading::ScopedMutexLock lock(_activityMutex);
    _activities.erase(Activity(activity,std::string()));
}

void
Registry::getActivities(std::set<std::string>& output)
{
    Threading::ScopedMutexLock lock(_activityMutex);
    for(std::set<Activity,ActivityLess>::const_iterator i = _activities.begin();
        i != _activities.end();
        ++i)
    {
        if ( ! i->second.empty() )
            output.insert( i->first + ": " + i->second );
        else
            output.insert( i->first );
    }
}

std::string 
Registry::getExtensionForMimeType(const std::string& mt)
{            
    std::string mt_lower = osgEarth::toLower(mt);

    const osgDB::Registry::MimeTypeExtensionMap& exmap = osgDB::Registry::instance()->getMimeTypeExtensionMap();
    for( osgDB::Registry::MimeTypeExtensionMap::const_iterator i = exmap.begin(); i != exmap.end(); ++i )
    {
        if ( i->first == mt_lower )
        {
            return i->first;
        }
    }
    return std::string();
}

std::string 
Registry::getMimeTypeForExtension(const std::string& ext)
{            
    std::string ext_lower = osgEarth::toLower(ext);

    const osgDB::Registry::MimeTypeExtensionMap& exmap = osgDB::Registry::instance()->getMimeTypeExtensionMap();
    for( osgDB::Registry::MimeTypeExtensionMap::const_iterator i = exmap.begin(); i != exmap.end(); ++i )
    {
        if ( i->second == ext_lower )
        {
            return i->first;
        }
    }
    return std::string();
}

void
Registry::setTextureImageUnitOffLimits(int unit)
{
    Threading::ScopedMutexLock exclusive(_regMutex);
    _offLimitsTextureImageUnits.insert(unit);
}

const std::set<int>
Registry::getOffLimitsTextureImageUnits() const
{
    Threading::ScopedMutexLock exclusive(_regMutex);
    return _offLimitsTextureImageUnits;
}


//Simple class used to add a file extension alias for the earth_tile to the earth plugin
class RegisterEarthTileExtension
{
public:
    RegisterEarthTileExtension()
    {
        osg::Referenced::setThreadSafeReferenceCounting( true );
        osgDB::Registry::instance()->addFileExtensionAlias("earth_tile", "earth");
    }
};
static RegisterEarthTileExtension s_registerEarthTileExtension;
