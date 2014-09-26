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
#include <osg/Notify>
#include <osg/Version>
#include <osgDB/Registry>
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
_cacheDriver        ( "filesystem" )
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

    // activate KMZ support
    osgDB::Registry::instance()->addArchiveExtension  ( "kmz" );
    osgDB::Registry::instance()->addFileExtensionAlias( "kmz", "kml" );

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

        // see if the environment specifies a default caching driver.
        const char* cacheDriver = ::getenv(OSGEARTH_ENV_CACHE_DRIVER);
        if ( cacheDriver )
        {
            setDefaultCacheDriverName( cacheDriver );
            OE_INFO << LC << "Cache driver set from environment: "
                << getDefaultCacheDriverName() << std::endl;
        }        

        // cache max age?
        const char* cacheMaxAge = ::getenv(OSGEARTH_ENV_CACHE_MAX_AGE);
        if ( cacheMaxAge )
        {
            TimeSpan maxAge = osgEarth::as<long>( std::string(cacheMaxAge), INT_MAX );
            _overrideCachePolicy->maxAge() = maxAge;
        }

        // see if there's a cache in the envvar; if so, create a cache.
        // Note: the value of the OSGEARTH_CACHE_PATH is not used here; rather
        // it's used in the driver(s) itself.
        const char* cachePath = ::getenv(OSGEARTH_ENV_CACHE_PATH);
        if ( cachePath )
        {
            CacheOptions options;
            options.setDriver( getDefaultCacheDriverName() );

            osg::ref_ptr<Cache> cache = CacheFactory::create(options);
            if ( cache->isOK() )
            {
                setCache( cache.get() );
            }
            else
            {
                OE_WARN << LC << "FAILED to initialize cache from environment" << std::endl;
            }
        }
    }

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
    _cache = 0L;
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
    if ( !_overrideCachePolicy.isSet() )
        _defaultCachePolicy->apply(_defaultOptions.get());
    else
        _overrideCachePolicy->apply(_defaultOptions.get());
}

void
Registry::setOverrideCachePolicy( const CachePolicy& value )
{
    _overrideCachePolicy = value;
    _overrideCachePolicy->apply( _defaultOptions.get() );
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

osgEarth::Cache*
Registry::getCache() const
{
	return _cache.get();
}

void
Registry::setCache( osgEarth::Cache* cache )
{
	_cache = cache;
    if ( cache )
        cache->apply( _defaultOptions.get() );
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
    Threading::ScopedWriteLock exclusive(_regMutex);
    _defaultFont = font;
}

osgText::Font*
Registry::getDefaultFont()
{
    Threading::ScopedReadLock shared(_regMutex);
    return _defaultFont.get();
}

UID
Registry::createUID()
{
    //todo: use OpenThreads::Atomic for this
    ScopedLock<Mutex> exclusive( _uidGenMutex );
    return (UID)( _uidGen++ );
}

osgDB::Options*
Registry::cloneOrCreateOptions( const osgDB::Options* input ) const
{
    osgDB::Options* newOptions = 
        input ? static_cast<osgDB::Options*>(input->clone(osg::CopyOp::SHALLOW_COPY)) : 
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

void
Registry::startActivity(const std::string& activity)
{
    Threading::ScopedMutexLock lock(_activityMutex);
    _activities.insert(activity);
}

void
Registry::endActivity(const std::string& activity)
{
    Threading::ScopedMutexLock lock(_activityMutex);
    _activities.erase(activity);
}

void
Registry::getActivities(std::set<std::string>& output)
{
    Threading::ScopedMutexLock lock(_activityMutex);
    output = _activities;
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
