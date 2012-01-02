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
#include <osgEarth/Registry>
#include <osgEarth/Cube>
#include <osgEarth/ShaderComposition>
#include <osgEarthDrivers/cache_filesystem/FileSystemCache>
#include <osg/Notify>
#include <osg/Version>
#include <osgDB/Registry>
#include <gdal_priv.h>
#include <ogr_api.h>
#include <stdlib.h>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define STR_GLOBAL_GEODETIC "global-geodetic"
#define STR_GLOBAL_MERCATOR "global-mercator"
#define STR_CUBE            "cube"
#define STR_LOCAL           "local"

#define LC "[Registry] "

// from MimeTypes.cpp
extern const char* builtinMimeTypeExtMappings[];

Registry::Registry() :
osg::Referenced  ( true ),
_gdal_registered ( false ),
_numGdalMutexGets( 0 ),
_uidGen          ( 0 ),
_caps            ( 0L ),
_defaultFont     ( 0L )
{
    // set up GDAL and OGR.
    OGRRegisterAll();
    GDALAllRegister();

    // add built-in mime-type extension mappings
    for( int i=0; ; i+=2 )
    {
        std::string mimeType = builtinMimeTypeExtMappings[i];
        if ( mimeType.length() == 0 )
            break;
        addMimeTypeExtensionMapping( mimeType, builtinMimeTypeExtMappings[i+1] );
    }

    _shaderLib = new ShaderFactory();
    _taskServiceManager = new TaskServiceManager();

    // activate KMZ support
    osgDB::Registry::instance()->addFileExtensionAlias( "kmz", "kml" );
    osgDB::Registry::instance()->addArchiveExtension( "kmz" );    

#if OSG_MIN_VERSION_REQUIRED(3,0,0)
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/vnd.google-earth.kml+xml", "kml" );
    osgDB::Registry::instance()->addMimeTypeExtensionMapping( "application/vnd.google-earth.kmz", "kmz" );
#endif
    
    // pre-load OSG's ZIP plugin so that we can use it in URIs
    std::string zipLib = osgDB::Registry::instance()->createLibraryNameForExtension( "zip" );
    if ( !zipLib.empty() )
        osgDB::Registry::instance()->loadLibrary( zipLib );    

    // set up our default r/w options to NOT cache archives!
    _defaultOptions = new osgDB::Options();
    _defaultOptions->setObjectCacheHint( osgDB::Options::CACHE_NONE );
    //_defaultOptions->setObjectCacheHint( (osgDB::Options::CacheHintOptions)
    //    ((int)_defaultOptions->getObjectCacheHint() & ~osgDB::Options::CACHE_ARCHIVES) );

    // see if there's a cache in the envvar
    const char* cachePath = ::getenv("OSGEARTH_CACHE_PATH");
    if ( cachePath )
    {
        FileSystemCacheOptions options;
        options.rootPath() = std::string(cachePath);

        osg::ref_ptr<Cache> cache = CacheFactory::create(options);
        if ( cache->isOK() )
        {
            setCache( cache.get() );
            OE_INFO << LC << "CACHE PATH set from environment variable: \"" << cachePath << "\"" << std::endl;
        }
        else
        {
            OE_WARN << LC << "FAILED to initialize cache from env.var." << std::endl;
        }
    }

    // activate cache-only mode from the environment
    if ( ::getenv("OSGEARTH_CACHE_ONLY") )
    {
        _defaultCachePolicy = CachePolicy::CACHE_ONLY;
        OE_INFO << LC << "CACHE-ONLY MODE set from environment variable" << std::endl;
    }

    // activate no-cache mode from the environment
    else if ( ::getenv("OSGEARTH_NO_CACHE") )
    {
        _defaultCachePolicy = CachePolicy::NO_CACHE;
        OE_INFO << LC << "NO-CACHE MODE set from environment variable" << std::endl;
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
    if ( !_global_mercator_profile.valid() )
    {
        GDAL_SCOPED_LOCK;

        if ( !_global_mercator_profile.valid() ) // double-check pattern
        {
            // automatically figure out proper mercator extents:
            const SpatialReference* srs = SpatialReference::create( "spherical-mercator" );
            //double e, dummy;
            //srs->getGeographicSRS()->transform( 180.0, 0.0, srs, e, dummy );            
            /*const_cast<Registry*>(this)->_global_mercator_profile = Profile::create(
                srs, -e, -e, e, e, 0L, 1, 1 );*/
            const_cast<Registry*>(this)->_global_mercator_profile = Profile::create(
                srs, MERC_MINX, MERC_MINY, MERC_MAXX, MERC_MAXY, 0L, 1, 1 );
        }
    }
    return _global_mercator_profile.get();
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
    else if ( name == STR_CUBE )
        return getCubeProfile();
    else
        return NULL;
}

const VerticalSpatialReference*
Registry::getDefaultVSRS() const
{
    if ( !_defaultVSRS.valid() )
        const_cast<Registry*>(this)->_defaultVSRS = new VerticalSpatialReference( Units::METERS );
    return _defaultVSRS.get();
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
        cache->store( _defaultOptions.get() );
}

void Registry::addMimeTypeExtensionMapping(const std::string fromMimeType, const std::string toExt)
{
    _mimeTypeExtMap[fromMimeType] = toExt;
}

osgDB::ReaderWriter* 
Registry::getReaderWriterForMimeType(const std::string& mimeType)
{
    MimeTypeExtensionMap::const_iterator i = _mimeTypeExtMap.find( mimeType );
    return i != _mimeTypeExtMap.end()?
        osgDB::Registry::instance()->getReaderWriterForExtension( i->second ) :
        NULL;
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

static OpenThreads::Mutex s_initCapsMutex;
void
Registry::initCapabilities()
{
    ScopedLock<Mutex> lock( s_initCapsMutex ); // double-check pattern (see getCapabilities)
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
    static Mutex s_uidGenMutex;
    ScopedLock<Mutex> lock( s_uidGenMutex );
    return (UID)( _uidGen++ );
}

osgDB::Options*
Registry::cloneOrCreateOptions( const osgDB::Options* input ) const
{
    osgDB::Options* newOptions = input ? static_cast<osgDB::Options*>(input->clone(osg::CopyOp::SHALLOW_COPY)) : new osgDB::Options();

    // clear the CACHE_ARCHIVES flag because it is evil
    if ( ((int)newOptions->getObjectCacheHint() & osgDB::Options::CACHE_ARCHIVES) != 0 )
    {
        newOptions->setObjectCacheHint( (osgDB::Options::CacheHintOptions)
            ((int)newOptions->getObjectCacheHint() & ~osgDB::Options::CACHE_ARCHIVES) );
    }

    return newOptions;
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
