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
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Cube>
#include <osgEarth/ShaderFactory>
#include <osgEarth/ObjectIndex>
#include <osgEarth/HTTPClient>
#include <osgEarth/TerrainEngineNode>

#include <osgText/Font>
#include <osgDB/Registry>

#include <gdal.h>
#include <cpl_conv.h>
#include <cstdlib>

using namespace osgEarth;

#define LC "[Registry] "

void osgEarth::initialize()
{
    osgEarth::Registry::instance()->getCapabilities();
}

namespace
{
    void CPL_STDCALL myCPLErrorHandler(CPLErr errClass, int errNum, const char* msg)
    {
        OE_DEBUG << "[GDAL] " << msg << " (error " << errNum << ")" << std::endl;
    }
}

Registry::Registry() :
_uidGen             ( 0 ),
_caps               ( 0L ),
_defaultFont        ( 0L ),
_terrainEngineDriver( "rex" ),
_cacheDriver        ( "filesystem" ),
_overrideCachePolicyInitialized( false ),
_devicePixelRatio(1.0f),
_maxVertsPerDrawable(UINT_MAX),
_regMutex("Registry(OE)"),
_activityMutex("Reg.Activity(OE)"),
_capsMutex("Reg.Caps(OE)"),
_srsCache("Reg.SRSCache(OE)"),
_blacklist("Reg.BlackList(OE)")
{
    // set up GDAL and OGR.
    OGRRegisterAll();
    GDALAllRegister();

#ifdef OSG_USE_UTF8_FILENAME
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","YES");
#else
    // support Chinese character in the file name and attributes in ESRI's shapefile
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8","NO");
#endif
    CPLSetConfigOption("SHAPE_ENCODING","");

#if GDAL_VERSION_MAJOR>=3
    CPLSetConfigOption("OGR_CT_FORCE_TRADITIONAL_GIS_ORDER", "YES");
#endif

    // Redirect GDAL/OGR console errors to our own handler
    CPLPushErrorHandler(myCPLErrorHandler);

    // Set the GDAL shared block cache size. This defaults to 5% of
    // available memory which is too high.
    GDALSetCacheMax(40 * 1024 * 1024);

    // global initialization for CURL (not thread safe)
    HTTPClient::globalInit();

    // warn if GDAL_DATA is not set
    if (::getenv("GDAL_DATA") == NULL)
        OE_INFO << LC << "Note: GDAL_DATA environment variable is not set" << std::endl;

    // generates the basic shader code for the terrain engine and model layers.
    _shaderLib = new ShaderFactory();

    // shader generator used internally by osgEarth. Can be replaced.
    _shaderGen = new ShaderGenerator();

    // optimizes sharing of state attributes and state sets for
    // performance boost
    _stateSetCache = new StateSetCache();

    // Default unref-after apply policy:
    _unRefImageDataAfterApply = false;

    if (::getenv("OSGEARTH_DISABLE_UNREF_AFTER_APPLY"))
        _unRefImageDataAfterApply = false;

    // Default object index for tracking scene object by UID.
    _objectIndex = new ObjectIndex();

    // activate KMZ support
    osgDB::Registry::instance()->addArchiveExtension( "kmz" );
    osgDB::Registry::instance()->addArchiveExtension( "3tz");
    osgDB::Registry::instance()->addFileExtensionAlias( "3tz", "zip" );
    //osgDB::Registry::instance()->addFileExtensionAlias( "kmz", "kml" );

    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/vnd.google-earth.kml+xml", "kml" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/vnd.google-earth.kml+xml; charset=utf8", "kml");
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/vnd.google-earth.kmz",     "kmz" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/plain",                           "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/xml",                             "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/json",                     "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/json",                            "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "text/x-json",                          "osgb" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "image/jpg",                            "jpg" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "image/dds",                            "dds" );
    // This is not correct, but some versions of readymap can return tif with one f instead of two.
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "image/tif",                            "tif" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "image/webp", "webp");

    // pre-load OSG's ZIP plugin so that we can use it in URIs
    std::string zipLib = osgDB::Registry::instance()->createLibraryNameForExtension( "zip" );
    if ( !zipLib.empty() )
        osgDB::Registry::instance()->loadLibrary( zipLib );

    _defaultOptions = new osgDB::Options();

    const char* teStr = ::getenv(OSGEARTH_ENV_TERRAIN_ENGINE_DRIVER);
    if ( teStr )
    {
        _terrainEngineDriver = std::string(teStr);
        _overrideTerrainEngineDriverName = std::string(teStr);
        OE_INFO << LC << "Terrain engine set from environment: " << _terrainEngineDriver << std::endl;
    }

    // load a default font
    const char* envFont = ::getenv("OSGEARTH_DEFAULT_FONT");
    if ( envFont )
    {
        _defaultFont = osgText::readRefFontFile( std::string(envFont) );
        OE_INFO << LC << "Default font set from environment: " << envFont << std::endl;
    }
    if ( !_defaultFont.valid() )
    {
#ifdef WIN32
        _defaultFont = osgText::readRefFontFile("arial.ttf");
#else
        _defaultFont = osgText::Font::getDefaultFont();
#endif
    }

#if OSG_VERSION_LESS_THAN(3,5,8)
    if ( _defaultFont.valid() )
    {
        // mitigates mipmapping issues that cause rendering artifacts
        // for some fonts/placement
        _defaultFont->setGlyphImageMargin( 2 );
    }
#endif

    const char* maxVerts = getenv("OSGEARTH_MAX_VERTS_PER_DRAWABLE");
    if (maxVerts)
    {
        sscanf(maxVerts, "%u", &_maxVertsPerDrawable);
        if (_maxVertsPerDrawable < 1024)
            _maxVertsPerDrawable = 65536;
    }

    // use the GDAL global mutex?
    if (getenv("OSGEARTH_DISABLE_GDAL_MUTEX"))
    {
        //getGDALMutex().disable();
    }

    // register the system stock Units.
    Units::registerAll( this );

    // Default concurrency for async image layers
    JobArena::setConcurrency("oe.layer.async", 4u);
}

Registry::~Registry()
{
    OE_DEBUG << LC << "Registry shutting down...\n";
    // A heavy hammer, but at this stage, which is usually application
    // shutdown, various osgEarth objects (e.g., VirtualPrograms) are
    // in the OSG cache and will cause a crash when they are deleted later.
    osgDB::Registry::instance()->clearObjectCache();
    OE_DEBUG << LC << "Registry shutdown complete.\n";

    // pop the custom error handler
    CPLPopErrorHandler();
}

static osg::ref_ptr<Registry> s_registry = NULL;

// Destroy the registry explicitly: this is called in an atexit() hook.  See comment in
// Registry::instance(bool reset).
void destroyRegistry()
{
   s_registry->release();
   s_registry = NULL;
}

Registry*
Registry::instance(bool reset)
{
    // Make sure the gdal mutex is created before the Registry so it will still be around when the registry is destroyed statically.
    // This is to prevent crash on exit where the gdal mutex is deleted before the registry is.
    osgEarth::getGDALMutex();

    static bool s_registryInit = false;

    // Create registry the first time through, explicitly rather than depending on static object
    // initialization order, which is undefined in c++ across separate compilation units.  An
    // explicit hook is registered to tear it down on exit.  atexit() hooks are run on exit in
    // the reverse order of their registration during setup.
    if (!s_registryInit)
    {
        s_registryInit = true;
        s_registry = new Registry;
        std::atexit(destroyRegistry);
    }

    if (reset)
    {
        s_registry->release();
        s_registry = new Registry();
    }

    return s_registry.get();
}

void
Registry::releaseGLObjects(osg::State* state) const
{
    // Clear out the state set cache
    if (_stateSetCache.valid())
    {
        _stateSetCache->releaseGLObjects(state);
    }

    // Clear out the VirtualProgram shared program repository
    _programRepo.lock();
    _programRepo.releaseGLObjects(state);
    _programRepo.unlock();
}

void
Registry::release()
{
    // GL resources (all GCs):
    releaseGLObjects(NULL);

    // Clear out the state set cache
    if (_stateSetCache.valid())
    {
        _stateSetCache->clear();
    }

    // SpatialReference cache
    _srsCache.clear();

    // Shared object index
    if (_objectIndex.valid())
        _objectIndex = new ObjectIndex();
}

Threading::RecursiveMutex& osgEarth::getGDALMutex()
{
    static osgEarth::Threading::RecursiveMutex _gdal_mutex("GDAL Mutex");
    return _gdal_mutex;
}

const Profile*
Registry::getGlobalGeodeticProfile() const
{
    // DEPRECATED
    static osg::ref_ptr<const Profile> p = Profile::create(Profile::GLOBAL_GEODETIC);
    return p.get();
}


const Profile*
Registry::getGlobalMercatorProfile() const
{
    // DEPRECATED
    return getSphericalMercatorProfile();
}


const Profile*
Registry::getSphericalMercatorProfile() const
{
    // DEPRECATED
    static osg::ref_ptr<const Profile> p = Profile::create(Profile::SPHERICAL_MERCATOR);
    return p.get();
}

const Profile*
Registry::getNamedProfile( const std::string& name ) const
{
    // DEPRECATED
    return Profile::create(name);
}

osg::ref_ptr<SpatialReference>
Registry::getOrCreateSRS(const SpatialReference::Key& key)
{
    ScopedMutexLock lock(_srsCache);
    osg::ref_ptr<SpatialReference>& srs = _srsCache[key];
    if (!srs.valid())
    {
        srs = SpatialReference::createFromKey(key);
    }
    return srs;
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
                    TimeSpan maxAge = osgEarth::Strings::as<long>( std::string(cacheMaxAge), INT_MAX );
                    _overrideCachePolicy->maxAge() = maxAge;
                    OE_INFO << LC << "Cache max age set from environment: " << cacheMaxAge << std::endl;
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
                    if (_defaultCache.valid() && _defaultCache->getStatus().isError())
                    {
                        OE_WARN << LC << "Cache error: " << _defaultCache->getStatus().toString() << std::endl;
                    }
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
    Threading::ScopedMutexLock sharedLock(_blacklist.mutex());
    return _blacklist.find(filename) != _blacklist.end();
}

void
Registry::blacklist(const std::string& filename)
{
    _blacklist.lock();
    _blacklist.insert(filename);
    OE_DEBUG << "Blacklist size = " << _blacklist.size() << std::endl;
    _blacklist.unlock();
}

void
Registry::clearBlacklist()
{
    _blacklist.lock();
    _blacklist.clear();
    _blacklist.unlock();
}

unsigned int
Registry::getNumBlacklistedFilenames()
{
    Threading::ScopedMutexLock sharedLock(_blacklist.mutex());
    return _blacklist.size();
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
    ScopedMutexLock lock( _capsMutex ); // double-check pattern (see getCapabilities)
    if ( !_caps.valid() )
        _caps = new Capabilities();
}

ShaderFactory*
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
    return _uidGen++;
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

    return newOptions;
}

void
Registry::registerUnits( const Units* units )
{
    Threading::ScopedMutexLock lock(_regMutex);
    _unitsVector.push_back(units);
}

const Units*
Registry::getUnits(const std::string& name) const
{
    Threading::ScopedMutexLock lock(_regMutex);
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

//void
//Registry::setDefaultTerrainEngineDriverName(const std::string& name)
//{
//    _terrainEngineDriver = name;
//}

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

ProgramRepo&
Registry::getProgramRepo()
{
    return _programRepo;
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

float
Registry::getDevicePixelRatio() const
{
    return _devicePixelRatio;
}

void
Registry::setDevicePixelRatio(float devicePixelRatio)
{
    _devicePixelRatio = devicePixelRatio;
}

void
Registry::setMaxNumberOfVertsPerDrawable(unsigned value)
{
    _maxVertsPerDrawable = value;
}

unsigned
Registry::getMaxNumberOfVertsPerDrawable() const
{
    return _maxVertsPerDrawable;
}

namespace
{
    //Simple class used to add a file extension alias for the earth_tile to the earth plugin
    class RegisterEarthTileExtension
    {
    public:
        RegisterEarthTileExtension()
        {
#if OSG_VERSION_LESS_THAN(3,5,4)
            // Method deprecated beyone 3.5.4 since all ref counting is thread-safe by default
            osg::Referenced::setThreadSafeReferenceCounting( true );
#endif
            osgDB::Registry::instance()->addFileExtensionAlias("earth_tile", "earth");
        }
    };
}
static RegisterEarthTileExtension s_registerEarthTileExtension;
