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


#include <osgEarth/Registry>
#include <osg/Notify>
#include <gdal_priv.h>
#include <ogr_api.h>
#include <stdlib.h>

using namespace osgEarth;
using namespace OpenThreads;

#define STR_GLOBAL_GEODETIC "global-geodetic"
#define STR_GLOBAL_MERCATOR "global-mercator"
#define STR_CUBE            "cube"
#define STR_LOCAL           "local"

// from MimeTypes.cpp
extern const char* builtinMimeTypeExtMappings[];

Registry::Registry() :
osg::Referenced(true),
_gdal_registered( false ),
_numGdalMutexGets( 0 )
{
    OGRRegisterAll();
    GDALAllRegister();

    //getGlobalGeodeticProfile();
    //getGlobalMercatorProfile();
    //getCubeProfile();

    // add built-in mime-type extension mappings
    for( int i=0; ; i+=2 )
    {
        std::string mimeType = builtinMimeTypeExtMappings[i];
        if ( mimeType.length() == 0 )
            break;
        addMimeTypeExtensionMapping( mimeType, builtinMimeTypeExtMappings[i+1] );
    }
}

Registry::~Registry()
{
}

Registry* Registry::instance(bool erase)
{
    static osg::ref_ptr<Registry> s_registry = new Registry;

    if (erase) 
    {   
        s_registry->destruct();
        s_registry = 0;
    }

    return s_registry.get(); // will return NULL on erase
}

void Registry::destruct()
{
    _cacheOverride = 0;
}


OpenThreads::ReentrantMutex&
Registry::getGDALMutex()
{
    //_numGdalMutexGets++;
    //osg::notify(osg::NOTICE) << "GDAL = " << _numGdalMutexGets << std::endl;
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
            double e, dummy;
            srs->getGeographicSRS()->transform( 180.0, 0.0, srs, e, dummy );
            
            const_cast<Registry*>(this)->_global_mercator_profile = Profile::create(
                srs, -e, -e, e, e, 1, 1 );
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
            const SpatialReference* srs = SpatialReference::create( "epsg:4326" );
            const_cast<Registry*>(this)->_cube_profile = Profile::createCube( srs );
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

osgEarth::Cache*
Registry::getCacheOverride() const
{
	return _cacheOverride.get();
}

void
Registry::setCacheOverride( osgEarth::Cache* cacheOverride )
{
	_cacheOverride = cacheOverride;
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

//Simple class used to add a file extension alias for the earth_tile to the earth plugin
class RegisterEarthTileExtension
{
public:
    RegisterEarthTileExtension()
    {
        osg::Referenced::setThreadSafeReferenceCounting( true );

        osgDB::Registry::instance()->addFileExtensionAlias("earth_tile", "earth");

        Registry::instance()->getGlobalGeodeticProfile();
        Registry::instance()->getGlobalMercatorProfile();
        Registry::instance()->getCubeProfile();
    }
};
static RegisterEarthTileExtension s_registerEarthTileExtension;
